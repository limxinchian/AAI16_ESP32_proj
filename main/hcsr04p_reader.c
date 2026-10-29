#include "hcsr04p_reader.h"
#include "hcsr04p_sensor.h"
#include "sensor_status.h"
#include "mqtt_manager.h"
#include "sensor_config.h"

#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define READING_LOG_US 1000000LL
#define SUMMARY_US 10000000LL

_Static_assert(HCSR04P_SAMPLE_MS >= HCSR04P_MIN_CYCLE_MS, "Trigger cycle too short");
_Static_assert(HCSR04P_SAMPLE_MS > HCSR04P_ECHO_TIMEOUT_MS, "Echo exceeds sample period");
_Static_assert(HCSR04P_MIN_INTERVAL_MS >= HCSR04P_MIN_CYCLE_MS, "Configured trigger cycle too short");
_Static_assert(HCSR04P_MIN_INTERVAL_MS > HCSR04P_ECHO_TIMEOUT_MS, "Configured period shorter than ECHO wait");

static TaskHandle_t s_hcsr04p_task;

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
    uint32_t interval_ms = HCSR04P_SAMPLE_MS;
    int64_t period_us = interval_ms * 1000LL;
    const int64_t tick_us = portTICK_PERIOD_MS * 1000LL;
    int64_t next_due = esp_timer_get_time(), previous_start = -1;
    int64_t next_log = next_due, next_summary = next_due + SUMMARY_US;
    int64_t max_read = 0, max_jitter = 0;
    uint32_t timeouts = 0, invalid = 0, overruns = 0, jitter_late = 0;
    uint32_t control = 0;
    bool pending_update = false, enabled = true;
    while (true) {
        // The same owner completes/disarms any active ECHO before committing
        // control. Notifications never cancel a driver wait or overwrite one
        // another; the bounded driver wait also bounds command service delay.
        for (unsigned commands = 0; commands < 4; ++commands) {
            if (!pending_update &&
                xTaskNotifyWait(0, UINT32_MAX, &control, 0) != pdTRUE) break;
            pending_update = false;
            mqtt_config_update_t update;
            if (!mqtt_manager_take_owner_config(SENSOR_HCSR04P, &update)) continue;
            sensor_status_t status;
            if (!config_can_apply(&update, &status)) continue;
            if (update.operation == MQTT_CONFIG_ENABLED) {
                if (sensor_status_set_enabled(SENSOR_HCSR04P, update.enabled) != ESP_OK) {
                    (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                    continue;
                }
                mqtt_manager_commit_hcsr04p_enabled(update.enabled);
                if (enabled != update.enabled) {
                    enabled = update.enabled;
                    previous_start = -1; // Deliberate pauses are not sampling jitter.
                    next_due = esp_timer_get_time() + period_us;
                    ESP_LOGI("SENSOR", "hcsr04p | CONFIG | enabled=%s", enabled ? "true" : "false");
                }
            } else {
                interval_ms = update.interval_ms; // Validated by the generic MQTT parser.
                period_us = interval_ms * 1000LL;
                next_due = esp_timer_get_time() + period_us;
                previous_start = -1; // A new schedule starts at this commit, never catches up.
                ESP_LOGI("SENSOR", "hcsr04p | CONFIG | interval applied: %" PRIu32 " ms", interval_ms);
            }
            // Status, acquisition schedule and snapshot gate are committed.
            (void)mqtt_manager_publish_config_ack(&update);
        }
        int64_t start = esp_timer_get_time();
        if (start >= next_summary) {
            next_summary = start + SUMMARY_US;
            sensor_status_t status;
            if (sensor_status_get(SENSOR_HCSR04P, &status)) {
                ESP_LOGI("SENSOR", "hcsr04p | HEALTH | current_error=%s ok_total=%" PRIu32 " failed_total=%" PRIu32 " consecutive=%" PRIu32
                         " timeouts_total=%" PRIu32 " invalid_total=%" PRIu32,
                         esp_err_to_name(status.last_error), status.read_successes, status.read_failures, status.consecutive_failures,
                         timeouts, invalid);
                ESP_LOGI("SENSOR", "hcsr04p | HEALTH | period=%" PRIu32 " ms enabled=%s overruns_total=%" PRIu32
                         " jitter_late_total=%" PRIu32 " max_jitter=%.2f ms max_read=%.2f ms stack_min=%u B",
                         interval_ms, enabled ? "true" : "false", overruns, jitter_late, max_jitter / 1000.0,
                         max_read / 1000.0, (unsigned)uxTaskGetStackHighWaterMark(NULL));
            }
        }
        // Keep diagnostic output time out of measured read duration and
        // anchor the next trigger to the actual read start, as before.
        start = esp_timer_get_time();
        if (!enabled || start < next_due) {
            // Wake on configuration even during a 10 s interval or a pause.
            // Keep periodic diagnostics alive; no indefinite wait or polling loop.
            int64_t wake = !enabled || next_summary < next_due ? next_summary : next_due;
            if (wake > start) {
                pending_update = xTaskNotifyWait(0, UINT32_MAX, &control,
                    (TickType_t)((wake - start + tick_us - 1) / tick_us)) == pdTRUE;
            }
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
        if (err == ESP_OK) {
            // Fixed-size handoff on every success, independently of DATA logs
            // and the MQTT schedule. Transport outcomes never change health.
            (void)mqtt_manager_update_hcsr04p_snapshot(&reading, completed);
        }
        if (err == ESP_OK && completed >= next_log) {
            next_log = completed + READING_LOG_US;
            ESP_LOGI("SENSOR", "hcsr04p | DATA | distance=%.1f cm echo=%.1f us read=%.2f ms",
                     reading.distance_cm, reading.echo_us, read_us / 1000.0);
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
    if (xTaskCreate(hcsr04p_reader_task, "hcsr04p_reader", 4096, NULL, 3, &s_hcsr04p_task) != pdPASS) {
        sensor_status_record_unavailable(SENSOR_HCSR04P, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
        ESP_LOGE("SENSOR", "hcsr04p | ERROR | Reader task creation failed; other sensors continue");
    }
}

TaskHandle_t hcsr04p_reader_task_handle(void)
{
    sensor_status_t status;
    // Startup failure deletes the task. Initialized owners remain available,
    // including while degraded or disabled, to process subsequent commands.
    if (!sensor_status_get(SENSOR_HCSR04P, &status) || !status.initialized ||
        status.state == SENSOR_UNAVAILABLE || status.state == SENSOR_UNINITIALIZED) return NULL;
    return s_hcsr04p_task;
}
