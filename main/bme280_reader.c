#include "bme280_reader.h"
#include "bme280_sensor.h"
#include "sensor_status.h"
#include "mqtt_manager.h"
#include "sensor_config.h"

#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BME280_SAMPLE_MS 1000
// Serial display only; every successful read is still published.
#define BME280_LOG_INTERVAL_US 10000000LL

static TaskHandle_t s_reader_task;

static void bme280_reader_task(void *arg)
{
    (void)arg;
    esp_err_t err = bme280_sensor_init();
    sensor_status_record_init(SENSOR_BME280, err);
    if (err != ESP_OK) {
        ESP_LOGE("BME280", "Initialization failed: %s; other sensors continue", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    TickType_t next_due = xTaskGetTickCount();
    TickType_t last_sample = next_due;
    TickType_t period = pdMS_TO_TICKS(BME280_SAMPLE_MS);
    uint32_t control = 0;
    bool pending_update = false;
    int64_t next_log_us = 0;
    esp_err_t last_status = ESP_OK;
    ESP_LOGI("BME280", "Reader running | poll=%d ms", BME280_SAMPLE_MS);
    while (true) {
        // Same bounded, ordered handoff as GY-511/RC522. Only this task commits
        // the acquisition gate/schedule, between complete measurements.
        for (unsigned commands = 0; commands < 4; ++commands) {
            if (!pending_update &&
                xTaskNotifyWait(0, UINT32_MAX, &control, 0) != pdTRUE) break;
            pending_update = false;
            mqtt_config_update_t update = {.sensor = SENSOR_BME280};
            if (control & CONFIG_ENABLED_FLAG) {
                update.operation = MQTT_CONFIG_ENABLED;
                update.enabled = (control & 1) != 0;
            } else {
                update.operation = MQTT_CONFIG_INTERVAL;
                update.interval_ms = control;
            }
            sensor_status_t status;
            if (!config_can_apply(&update, &status)) continue;
            if (update.operation == MQTT_CONFIG_ENABLED) {
                if (sensor_status_set_enabled(SENSOR_BME280, update.enabled) != ESP_OK) {
                    (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                    continue;
                }
                if (update.enabled && status.state == SENSOR_DISABLED) {
                    // Fresh anchor: no catch-up reads after a pause.
                    last_sample = xTaskGetTickCount();
                    next_due = last_sample + period;
                }
                ESP_LOGI("BME280", "[CONFIG] enabled applied: %s", update.enabled ? "true" : "false");
            } else {
                // Round up to the RTOS tick.
                period = pdMS_TO_TICKS(update.interval_ms + portTICK_PERIOD_MS - 1);
                next_due = last_sample + period;
                TickType_t now = xTaskGetTickCount();
                if ((int32_t)(next_due - now) < 0) next_due = now;
                ESP_LOGI("BME280", "[CONFIG] interval applied: %lu ms", (unsigned long)update.interval_ms);
            }
            (void)mqtt_manager_publish_config_ack(&update);
        }

        if (!sensor_status_can_sample(SENSOR_BME280)) {
            pending_update = xTaskNotifyWait(0, UINT32_MAX, &control, portMAX_DELAY) == pdTRUE;
            continue;
        }
        TickType_t tick_now = xTaskGetTickCount();
        int32_t remaining = (int32_t)(next_due - tick_now);
        if (remaining > 0) {
            pending_update = xTaskNotifyWait(0, UINT32_MAX, &control, (TickType_t)remaining) == pdTRUE;
            continue;
        }
        last_sample = tick_now;
        bme280_data_t reading = {0};
        err = bme280_sensor_read(&reading);
        int64_t now_us = esp_timer_get_time();
        sensor_status_record_read(SENSOR_BME280, err, now_us);
        if (err == ESP_OK) {
            // Zero-wait handoff. Delivery failures appear in MQTT diagnostics.
            (void)mqtt_manager_publish_bme280(reading.temperature_c, reading.pressure_hpa,
                                              reading.humidity_pct, reading.has_humidity);
        }
        if (now_us >= next_log_us || (err != ESP_OK && err != last_status)) {
            next_log_us = now_us + BME280_LOG_INTERVAL_US;
            if (err != ESP_OK) {
                ESP_LOGW("BME280", "Reading failed: %s", esp_err_to_name(err));
            } else if (reading.has_humidity) {
                ESP_LOGI("BME280", "T: %.2f C | P: %.2f hPa | RH: %.1f %%",
                         reading.temperature_c, reading.pressure_hpa, reading.humidity_pct);
            } else {
                ESP_LOGI("BME280", "T: %.2f C | P: %.2f hPa | RH: n/a (BMP280)",
                         reading.temperature_c, reading.pressure_hpa);
            }
        }
        last_status = err;
        // Skip missed deadlines; no busy catch-up after a stall.
        if ((TickType_t)(xTaskGetTickCount() - next_due) >= period) next_due = xTaskGetTickCount();
        next_due += period;
    }
}

void bme280_reader_start(void)
{
    sensor_status_configure(SENSOR_BME280, "bme280", true);
    if (xTaskCreate(bme280_reader_task, "bme280_reader", 4096, NULL, 3, &s_reader_task) != pdPASS) {
        sensor_status_record_unavailable(SENSOR_BME280, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
        ESP_LOGE("BME280", "Reader task creation failed; other sensors continue");
    }
}

TaskHandle_t bme280_reader_task_handle(void)
{
    sensor_status_t status;
    // A failed initialization deletes its task. Never expose that stale handle.
    if (!sensor_status_get(SENSOR_BME280, &status) || !status.initialized ||
        status.state == SENSOR_UNAVAILABLE || status.state == SENSOR_UNINITIALIZED) return NULL;
    return s_reader_task;
}
