#include "rc522_reader.h"
#include "rc522_sensor.h"
#include "sensor_status.h"
#include "spi_bus_manager.h"
#include "mqtt_manager.h"
#include "sensor_config.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define POLL_MS 200

static TaskHandle_t s_reader_task;

static void rc522_reader_task(void *arg)
{
    (void)arg;
    esp_err_t err = rc522_sensor_init();
    sensor_status_record_init(SENSOR_RC522, err);
    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "rc522   | ERROR | Initialization failed: %s; other sensors continue", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    rc522_uid_t last_uid = {0};
    unsigned idle_polls = 0;
    TickType_t next_due = xTaskGetTickCount();
    TickType_t last_sample = next_due;
    TickType_t period = pdMS_TO_TICKS(POLL_MS);
    uint32_t control = 0;
    bool pending_update = false;
    ESP_LOGI("SENSOR", "rc522   | HEALTH | Reader running | poll=%d ms | MQTT card presentation events", POLL_MS);
    while (true) {
        // Same bounded, ordered handoff as GY-511. Only this task commits the
        // acquisition gate/schedule, between complete RFID transactions.
        for (unsigned commands = 0; commands < 4; ++commands) {
            if (!pending_update &&
                xTaskNotifyWait(0, UINT32_MAX, &control, 0) != pdTRUE) break;
            pending_update = false;
            mqtt_config_update_t update;
            if (!mqtt_manager_take_owner_config(SENSOR_RC522, &update)) continue;
            sensor_status_t status;
            if (!config_can_apply(&update, &status)) continue;
            if (update.operation == MQTT_CONFIG_ENABLED) {
                if (sensor_status_set_enabled(SENSOR_RC522, update.enabled) != ESP_OK) {
                    (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                    continue;
                }
                if (update.enabled && status.state == SENSOR_DISABLED) {
                    last_sample = xTaskGetTickCount();
                    next_due = last_sample + period;
                } else if (!update.enabled && status.state != SENSOR_DISABLED) {
                    // A pause is not observed removal. Keep the last UID, but
                    // require three fresh consecutive idle polls after resume.
                    idle_polls = 0;
                }
                ESP_LOGI("SENSOR", "rc522   | HEALTH | config_enabled=%s", update.enabled ? "true" : "false");
            } else {
                period = pdMS_TO_TICKS(update.interval_ms + portTICK_PERIOD_MS - 1);
                next_due = last_sample + period;
                TickType_t now = xTaskGetTickCount();
                if ((int32_t)(next_due - now) < 0) next_due = now;
                ESP_LOGI("SENSOR", "rc522   | HEALTH | config_period_ms=%lu", (unsigned long)update.interval_ms);
            }
            (void)mqtt_manager_publish_config_ack(&update);
        }

        if (!sensor_status_can_sample(SENSOR_RC522)) {
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
        rc522_uid_t uid;
        bool present;
        err = rc522_sensor_poll(&uid, &present);
        int64_t now = esp_timer_get_time();
        if (err != ESP_OK) {
            sensor_status_record_read(SENSOR_RC522, err, now);
            // Do not interpret a communication failure as card removal.
            idle_polls = 0;
        } else if (present) {
            sensor_status_record_read(SENSOR_RC522, ESP_OK, now);
            idle_polls = 0;
            if (uid.length != last_uid.length || memcmp(uid.bytes, last_uid.bytes, uid.length)) {
                // Zero-wait handoff only. Delivery failures belong to MQTT
                // diagnostics, not sensor health; do not retry a held card.
                (void)mqtt_manager_publish_rc522_event(uid.bytes, uid.length);
                // Fixed bounds, including the maximum ten-byte UID and NUL.
                char text[sizeof(uid.bytes) * 2 + 1] = {0};
                static const char hex[] = "0123456789ABCDEF";
                for (unsigned i = 0; i < uid.length && i < sizeof(uid.bytes); ++i) {
                    text[2 * i] = hex[uid.bytes[i] >> 4];
                    text[2 * i + 1] = hex[uid.bytes[i] & 0x0f];
                }
                ESP_LOGI("SENSOR", "rc522   | DATA | event=PRESENT uid=%s uid_len=%u", text, (unsigned)uid.length);
                last_uid = uid;
            }
        } else {
            // Idle does not fabricate a successful UID or increment failures.
            // Three consecutive idle polls re-arm the filter (~600 ms at default).
            if (idle_polls < 3) idle_polls++;
            if (idle_polls == 3) last_uid.length = 0;
        }
        // Skip missed deadlines; no busy catch-up after a long anticollision scan.
        if ((TickType_t)(xTaskGetTickCount() - next_due) >= period) next_due = xTaskGetTickCount();
        next_due += period;
    }
}

void rc522_reader_start(void)
{
    sensor_status_configure(SENSOR_RC522, "rc522", true);
    esp_err_t err = spi_bus_manager_init();
    if (err != ESP_OK) {
        sensor_status_record_unavailable(SENSOR_RC522, SENSOR_STAGE_DEPENDENCY, err);
        ESP_LOGE("SENSOR", "rc522   | ERROR | SPI bus unavailable: %s; other sensors continue", esp_err_to_name(err));
        return;
    }
    if (xTaskCreate(rc522_reader_task, "rc522_reader", 4096, NULL, 3, &s_reader_task) != pdPASS) {
        sensor_status_record_unavailable(SENSOR_RC522, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
        ESP_LOGE("SENSOR", "rc522   | ERROR | Reader task creation failed; other sensors continue");
    }
}

TaskHandle_t rc522_reader_task_handle(void)
{
    sensor_status_t status;
    // A failed initialization deletes its task. Never expose that stale handle.
    // Once initialization succeeds, this owner never deletes itself.
    if (!sensor_status_get(SENSOR_RC522, &status) || !status.initialized ||
        status.state == SENSOR_UNAVAILABLE || status.state == SENSOR_UNINITIALIZED) return NULL;
    return s_reader_task;
}
