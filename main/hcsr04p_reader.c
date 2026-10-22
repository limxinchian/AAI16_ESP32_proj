#include "hcsr04p_reader.h"
#include "hcsr04p_sensor.h"
#include "sensor_status.h"

#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define READING_LOG_US 1000000LL
#define SUMMARY_US 10000000LL

_Static_assert(HCSR04P_SAMPLE_MS >= HCSR04P_MIN_CYCLE_MS, "Trigger cycle too short");
_Static_assert(HCSR04P_SAMPLE_MS > HCSR04P_ECHO_TIMEOUT_MS, "Echo exceeds sample period");

static void hcsr04p_reader_task(void *arg)
{
    (void)arg;
    esp_err_t err = hcsr04p_sensor_init();
    sensor_status_record_init(SENSOR_HCSR04P, err);
    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "hcsr04p | ERROR | Capture initialization failed: %s; other sensors continue", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI("SENSOR", "hcsr04p | HEALTH | Driver initialized; awaiting valid ECHO. TRIG=4 ECHO=5 period=200 ms timeout=30 ms");
    const int64_t period_us = HCSR04P_SAMPLE_MS * 1000LL;
    const int64_t tick_us = portTICK_PERIOD_MS * 1000LL;
    int64_t next_due = esp_timer_get_time(), previous_start = -1;
    int64_t next_log = next_due, next_summary = next_due + SUMMARY_US;
    int64_t max_read = 0, max_jitter = 0;
    uint32_t timeouts = 0, invalid = 0, overruns = 0, jitter_late = 0;
    while (true) {
        int64_t start = esp_timer_get_time();
        if (start < next_due) {
            vTaskDelay((TickType_t)((next_due - start + tick_us - 1) / tick_us));
            continue;
        }
        if (previous_start >= 0) {
            int64_t jitter = start - previous_start - period_us;
            if (jitter > max_jitter) max_jitter = jitter;
            if (jitter > tick_us) jitter_late++;
        }
        previous_start = start;
        // Anchor to actual start: no short trigger intervals after a late wake.
        next_due = start + period_us;
        hcsr04p_reading_t reading;
        err = hcsr04p_sensor_read(&reading);
        int64_t completed = esp_timer_get_time();
        int64_t read_us = completed - start;
        if (read_us > max_read) max_read = read_us;
        timeouts += err == ESP_ERR_TIMEOUT;
        invalid += err == ESP_ERR_INVALID_RESPONSE;
        sensor_status_record_read(SENSOR_HCSR04P, err, completed);
        if (err == ESP_OK && completed >= next_log) {
            next_log = completed + READING_LOG_US;
            ESP_LOGI("SENSOR", "hcsr04p | DATA | distance=%.1f cm echo=%.1f us read=%.2f ms",
                     reading.distance_cm, reading.echo_us, read_us / 1000.0);
        }
        if (completed >= next_summary) {
            next_summary = completed + SUMMARY_US;
            sensor_status_t status;
            if (sensor_status_get(SENSOR_HCSR04P, &status)) {
                ESP_LOGI("SENSOR", "hcsr04p | HEALTH | current_error=%s ok_total=%" PRIu32 " failed_total=%" PRIu32 " consecutive=%" PRIu32
                         " timeouts_total=%" PRIu32 " invalid_total=%" PRIu32,
                         esp_err_to_name(status.last_error), status.read_successes, status.read_failures, status.consecutive_failures,
                         timeouts, invalid);
                ESP_LOGI("SENSOR", "hcsr04p | HEALTH | period=%d ms overruns_total=%" PRIu32
                         " jitter_late_total=%" PRIu32 " max_jitter=%.2f ms max_read=%.2f ms stack_min=%u B",
                         HCSR04P_SAMPLE_MS, overruns, jitter_late, max_jitter / 1000.0,
                         max_read / 1000.0, (unsigned)uxTaskGetStackHighWaterMark(NULL));
            }
        }
        int64_t now = esp_timer_get_time();
        if (now >= next_due) {
            overruns++;
            next_due = now + period_us; // Skip missed periods, never burst to catch up.
        }
    }
}

void hcsr04p_reader_start(void)
{
    sensor_status_configure(SENSOR_HCSR04P, "hcsr04p", true);
    // One task is needed to keep bounded ECHO waits out of app_main and the
    // existing readers. Same priority as RC522/LD2450; existing priorities stay.
    if (xTaskCreate(hcsr04p_reader_task, "hcsr04p_reader", 4096, NULL, 3, NULL) != pdPASS) {
        sensor_status_record_unavailable(SENSOR_HCSR04P, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
        ESP_LOGE("SENSOR", "hcsr04p | ERROR | Reader task creation failed; other sensors continue");
    }
}
