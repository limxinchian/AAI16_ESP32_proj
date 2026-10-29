#include "max6675_reader.h"
#include "max6675_sensor.h"
#include "sensor_status.h"
#include "mqtt_manager.h"
#include "spi_bus_manager.h"
#include "sensor_config.h"

#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Matches the sensor's ~0.22 s conversion; never poll faster than 250 ms.
#define MAX6675_SAMPLE_MS 250
// Serial display only; every successful read is still published.
#define MAX6675_LOG_INTERVAL_US 10000000LL

static TaskHandle_t s_reader_task;

static void max6675_reader_task(void *arg)
{
    (void)arg;
    esp_err_t err = max6675_sensor_init();
    sensor_status_record_init(SENSOR_MAX6675, err);
    if (err != ESP_OK) {
        ESP_LOGE("MAX6675", "Initialization failed: %s; other sensors continue", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    TickType_t next_due = xTaskGetTickCount();
    TickType_t last_sample = next_due;
    TickType_t period = pdMS_TO_TICKS(MAX6675_SAMPLE_MS);
    uint32_t control = 0;
    bool pending_update = false;
    int64_t next_log_us = 0;
    esp_err_t last_status = ESP_OK;
    ESP_LOGI("MAX6675", "Reader running | poll=%d ms", MAX6675_SAMPLE_MS);
    while (true) {
        // Same bounded, ordered handoff as GY-511/RC522. Only this task commits
        // the acquisition gate/schedule, between complete measurements.
        for (unsigned commands = 0; commands < 4; ++commands) {
            if (!pending_update &&
                xTaskNotifyWait(0, UINT32_MAX, &control, 0) != pdTRUE) break;
            pending_update = false;
            mqtt_config_update_t update;
            if (!mqtt_manager_take_owner_config(SENSOR_MAX6675, &update)) continue;
            sensor_status_t status;
            if (!config_can_apply(&update, &status)) continue;
            if (update.operation == MQTT_CONFIG_ENABLED) {
                if (sensor_status_set_enabled(SENSOR_MAX6675, update.enabled) != ESP_OK) {
                    (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                    continue;
                }
                if (update.enabled && status.state == SENSOR_DISABLED) {
                    // Fresh anchor: no catch-up reads after a pause.
                    last_sample = xTaskGetTickCount();
                    next_due = last_sample + period;
                }
                ESP_LOGI("MAX6675", "[CONFIG] enabled applied: %s", update.enabled ? "true" : "false");
            } else {
                // Round up to the RTOS tick.
                period = pdMS_TO_TICKS(update.interval_ms + portTICK_PERIOD_MS - 1);
                next_due = last_sample + period;
                TickType_t now = xTaskGetTickCount();
                if ((int32_t)(next_due - now) < 0) next_due = now;
                ESP_LOGI("MAX6675", "[CONFIG] interval applied: %lu ms", (unsigned long)update.interval_ms);
            }
            (void)mqtt_manager_publish_config_ack(&update);
        }

        if (!sensor_status_can_sample(SENSOR_MAX6675)) {
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
        max6675_data_t reading = {0};
        err = max6675_sensor_read(&reading);
        int64_t now_us = esp_timer_get_time();
        sensor_status_record_read(SENSOR_MAX6675, err, now_us);
        if (err == ESP_OK) {
            // Zero-wait handoff. Delivery failures appear in MQTT diagnostics.
            (void)mqtt_manager_publish_max6675(reading.temperature_c);
        }
        if (now_us >= next_log_us || (err != ESP_OK && err != last_status)) {
            next_log_us = now_us + MAX6675_LOG_INTERVAL_US;
            if (err == ESP_ERR_NOT_FOUND) {
                ESP_LOGW("MAX6675", "Thermocouple open or disconnected | raw=0x%04X", reading.raw);
            } else if (err != ESP_OK) {
                ESP_LOGW("MAX6675", "Reading failed: %s | raw=0x%04X", esp_err_to_name(err), reading.raw);
            } else {
                ESP_LOGI("MAX6675", "T: %.2f C | raw=0x%04X", reading.temperature_c, reading.raw);
            }
        }
        last_status = err;
        // Skip missed deadlines; no busy catch-up after a stall.
        if ((TickType_t)(xTaskGetTickCount() - next_due) >= period) next_due = xTaskGetTickCount();
        next_due += period;
    }
}

void max6675_reader_start(void)
{
    sensor_status_configure(SENSOR_MAX6675, "max6675", true);
    esp_err_t err = spi_bus_manager_init();
    if (err != ESP_OK) {
        sensor_status_record_unavailable(SENSOR_MAX6675, SENSOR_STAGE_DEPENDENCY, err);
        ESP_LOGE("MAX6675", "SPI bus unavailable: %s; other sensors continue", esp_err_to_name(err));
        return;
    }
    if (xTaskCreate(max6675_reader_task, "max6675_reader", 4096, NULL, 3, &s_reader_task) != pdPASS) {
        sensor_status_record_unavailable(SENSOR_MAX6675, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
        ESP_LOGE("MAX6675", "Reader task creation failed; other sensors continue");
    }
}

TaskHandle_t max6675_reader_task_handle(void)
{
    sensor_status_t status;
    // A failed initialization deletes its task. Never expose that stale handle.
    if (!sensor_status_get(SENSOR_MAX6675, &status) || !status.initialized ||
        status.state == SENSOR_UNAVAILABLE || status.state == SENSOR_UNINITIALIZED) return NULL;
    return s_reader_task;
}
