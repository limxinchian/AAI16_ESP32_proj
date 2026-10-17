#include "rc522_reader.h"
#include "rc522_sensor.h"
#include "sensor_status.h"
#include "spi_bus_manager.h"
#include "mqtt_manager.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define POLL_MS 200
#define ERROR_LOG_US 10000000LL

static void rc522_reader_task(void *arg)
{
    (void)arg;
    esp_err_t err = rc522_sensor_init();
    sensor_status_record_init(SENSOR_RC522, err);
    if (err != ESP_OK) {
        ESP_LOGE("RC522", "Initialization failed: %s; other sensors continue", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    rc522_uid_t last_uid = {0};
    unsigned idle_polls = 0;
    int64_t next_error_log_us = 0;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(POLL_MS);
    ESP_LOGI("RC522", "Reader running | poll=%d ms | MQTT card presentation events", POLL_MS);
    while (true) {
        rc522_uid_t uid;
        bool present;
        err = rc522_sensor_poll(&uid, &present);
        int64_t now = esp_timer_get_time();
        if (err != ESP_OK) {
            sensor_status_record_read(SENSOR_RC522, err, now);
            // Do not interpret a communication failure as card removal.
            idle_polls = 0;
            if (now >= next_error_log_us) {
                next_error_log_us = now + ERROR_LOG_US;
                ESP_LOGW("RC522", "Poll failed: %s", esp_err_to_name(err));
            }
        } else if (present) {
            sensor_status_record_read(SENSOR_RC522, ESP_OK, now);
            idle_polls = 0;
            if (uid.length != last_uid.length || memcmp(uid.bytes, last_uid.bytes, uid.length)) {
                // Zero-wait handoff only. Delivery failures belong to MQTT
                // diagnostics, not sensor health; do not retry a held card.
                (void)mqtt_manager_publish_rc522_event(uid.bytes, uid.length);
                char text[30]; // Ten hex bytes, nine spaces and NUL.
                size_t used = 0;
                for (unsigned i = 0; i < uid.length; ++i) {
                    used += snprintf(text + used, sizeof(text) - used, "%s%02X", i ? " " : "", uid.bytes[i]);
                }
                ESP_LOGI("RC522", "UID (%u bytes): %s", (unsigned)uid.length, text);
                last_uid = uid;
            }
        } else {
            // Idle does not fabricate a successful UID or increment failures.
            // Three consecutive idle polls (~600 ms) re-arm the duplicate filter.
            if (idle_polls < 3) idle_polls++;
            if (idle_polls == 3) last_uid.length = 0;
        }
        // Skip missed deadlines; no busy catch-up after a long anticollision scan.
        if ((TickType_t)(xTaskGetTickCount() - last_wake) >= period) last_wake = xTaskGetTickCount();
        vTaskDelayUntil(&last_wake, period);
    }
}

void rc522_reader_start(void)
{
    sensor_status_configure(SENSOR_RC522, "rc522", true);
    esp_err_t err = spi_bus_manager_init();
    if (err != ESP_OK) {
        sensor_status_record_unavailable(SENSOR_RC522, SENSOR_STAGE_DEPENDENCY, err);
        ESP_LOGE("RC522", "SPI bus unavailable: %s; other sensors continue", esp_err_to_name(err));
        return;
    }
    if (xTaskCreate(rc522_reader_task, "rc522_reader", 4096, NULL, 3, NULL) != pdPASS) {
        sensor_status_record_unavailable(SENSOR_RC522, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
        ESP_LOGE("RC522", "Reader task creation failed; other sensors continue");
    }
}
