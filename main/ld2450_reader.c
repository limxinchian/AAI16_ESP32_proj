#include "ld2450_reader.h"
#include "ld2450_sensor.h"
#include "sensor_status.h"
#include "mqtt_manager.h"
#include "sensor_config.h"
#include <inttypes.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FIRST_REPORT_US 5000000LL
#define SILENCE_US 2000000LL
#define SUMMARY_US 10000000LL

static TaskHandle_t s_ld2450_task;

static void log_diagnostics(unsigned timeouts, int64_t age_us)
{
    ld2450_diagnostics_t d;
    ld2450_sensor_get_diagnostics(&d);
    ESP_LOGI("SENSOR", "ld2450  | HEALTH | frames_total=%" PRIu32 " rx_bytes_total=%" PRIu32
             " bad_frames_total=%" PRIu32 " partial_total=%" PRIu32 " dropped_known_total=%" PRIu32,
             d.frames, d.rx_bytes, d.framing_errors, d.partial_timeouts, d.discarded_bytes);
    ESP_LOGI("SENSOR", "ld2450  | HEALTH | uart_errors_total=%" PRIu32 " rx_overflow_total=%" PRIu32
             " event_overflow_total=%" PRIu32 " silence_timeouts_total=%u",
             d.uart_errors, d.rx_overflows, d.event_overflows, timeouts);
    ESP_LOGI("SENSOR", "ld2450  | HEALTH | last_valid_age_ms=%" PRId64
             " rx_high=%u max_process_us=%" PRId64 " stack_min=%u B",
             age_us < 0 ? -1 : age_us / 1000,
             (unsigned)d.max_buffered_bytes, d.max_processing_us,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
}

static void log_report(const ld2450_report_t *report)
{
    ESP_LOGI("SENSOR", "ld2450  | DATA | targets=%u", report->count);
    for (unsigned i = 0; i < LD2450_TARGETS; ++i) {
        const ld2450_target_t *t = &report->targets[i];
        if (!t->present) continue;
        unsigned distance = (unsigned)(sqrtf((float)t->x_mm * t->x_mm + (float)t->y_mm * t->y_mm) + 0.5f);
        ESP_LOGI("SENSOR", "ld2450  | DATA | slot=%u x=%d mm y=%d mm distance=%u mm speed=%d cm/s resolution=%u mm",
                 i + 1, t->x_mm, t->y_mm, distance, t->speed_cm_s, t->resolution_mm);
    }
}

static void ld2450_reader_task(void *arg)
{
    (void)arg;
    esp_err_t err = ld2450_sensor_init();
    if (err != ESP_OK) {
        sensor_status_record_init(SENSOR_LD2450, err);
        ESP_LOGE("SENSOR", "ld2450  | ERROR | UART initialization failed: %s; other sensors continue", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    // Keep UNINITIALIZED until an actual report proves communication works.
    int64_t started = esp_timer_get_time(), last_valid = -1;
    int64_t next_silence = started + SILENCE_US, next_warning = 0;
    int64_t next_summary = started + SUMMARY_US, next_change_log = 0;
    int64_t resync_until = 0;
    unsigned displayed_mask = UINT32_MAX, timeouts = 0;
    bool enabled = true, fresh_report = false;
    ld2450_report_t report = {0};
    ESP_LOGI("SENSOR", "ld2450  | HEALTH | Waiting for first valid report (5000 ms); MQTT latest snapshot at ~1 Hz");
    while (true) {
        // Same no-overwrite notification handoff as the other reader owners.
        // UART waits are <=100 ms, so no extra control task/queue is necessary.
        for (unsigned commands = 0; commands < 4; ++commands) {
            uint32_t control;
            if (xTaskNotifyWait(0, UINT32_MAX, &control, 0) != pdTRUE) break;
            mqtt_config_update_t update;
            if (!mqtt_manager_take_owner_config(SENSOR_LD2450, &update)) continue;
            sensor_status_t status;
            if (!config_can_apply(&update, &status)) continue;
            if (update.operation == MQTT_CONFIG_ENABLED && update.enabled && !enabled) {
                // A report assembled from pre-enable bytes must not become a
                // new measurement. Flush only on an actual disabled->enabled edge.
                err = ld2450_sensor_discard_pending();
                if (err != ESP_OK) {
                    ESP_LOGW("SENSOR", "ld2450  | ERROR | Resume input discard failed: %s; remains disabled", esp_err_to_name(err));
                    (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                    continue;
                }
            }
            if (update.operation == MQTT_CONFIG_ENABLED &&
                sensor_status_set_enabled(SENSOR_LD2450, update.enabled) != ESP_OK) {
                (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                continue;
            }
            // Commit the publisher gate/period atomically with mailbox reset.
            // This never waits for the SDK; an already admitted send can finish.
            err = mqtt_manager_commit_ld2450_config(&update);
            if (err != ESP_OK) {
                if (update.operation == MQTT_CONFIG_ENABLED) {
                    (void)sensor_status_set_enabled(SENSOR_LD2450, status.state != SENSOR_DISABLED);
                }
                (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                continue;
            }
            if (update.operation == MQTT_CONFIG_ENABLED) {
                if (update.enabled != enabled) {
                    fresh_report = false;
                    report = (ld2450_report_t){0};
                    next_silence = esp_timer_get_time() + SILENCE_US;
                    resync_until = update.enabled ? next_silence : 0;
                    next_warning = 0;
                    displayed_mask = UINT32_MAX;
                }
                enabled = update.enabled;
                ESP_LOGI("SENSOR", "ld2450  | HEALTH | config_enabled=%s uart_draining=true", enabled ? "true" : "false");
            } else {
                ESP_LOGI("SENSOR", "ld2450  | HEALTH | config_mqtt_interval_ms=%lu uart_rate=unchanged", (unsigned long)update.interval_ms);
            }
            (void)mqtt_manager_publish_config_ack(&update);
        }
        unsigned frames = 0;
        err = ld2450_sensor_receive(&report, &frames);
        int64_t now = esp_timer_get_time();
        if (!enabled) {
            // Keep consuming the bounded UART stream and its driver diagnostics,
            // but freeze health history, suppress reports and create no snapshots.
            if (now >= next_summary) {
                log_diagnostics(timeouts, last_valid < 0 ? -1 : now - last_valid);
                next_summary = now + SUMMARY_US;
            }
            vTaskDelay(1);
            continue;
        }
        if (frames && last_valid < 0) sensor_status_record_init(SENSOR_LD2450, ESP_OK);
        // Resume flushing can split an on-wire frame. Until a fresh report (or
        // the 2 s silence deadline), keep framing resync in driver diagnostics
        // without inventing a health failure from the intentional boundary.
        // Actual UART faults are still recorded immediately.
        bool resync_grace = !fresh_report && now < resync_until &&
                            err == ESP_ERR_INVALID_RESPONSE;
        if (err != ESP_OK && !resync_grace) {
            // Startup errors remain diagnostic evidence until the first-report
            // deadline. Do not falsely mark the unproven sensor initialized.
            sensor_status_record_read(SENSOR_LD2450, err, now);
            // After initialization the registry owns failure/recovery logs.
            if (last_valid < 0 && !frames && now >= next_warning) {
                ESP_LOGW("SENSOR", "ld2450  | ERROR | Receive failed: %s", esp_err_to_name(err));
                next_warning = now + SUMMARY_US;
            }
        }
        if (frames) {
            fresh_report = true;
            for (unsigned i = 0; i < frames; ++i) sensor_status_record_read(SENSOR_LD2450, ESP_OK, now);
            last_valid = now;
            next_silence = now + SILENCE_US;
            // A framing error can coexist with valid reports after resync.
            // Only frames>0 proves report contains new, validated UART data.
            // This copies a small mailbox; it never calls the MQTT SDK.
            (void)mqtt_manager_update_ld2450_snapshot(&report, now);
        } else if (last_valid < 0 && now - started >= FIRST_REPORT_US) {
            sensor_status_record_init(SENSOR_LD2450, ESP_ERR_TIMEOUT);
            ESP_LOGE("SENSOR", "ld2450  | ERROR | No valid report within 5000 ms; unavailable; other sensors continue");
            log_diagnostics(1, -1);
            esp_err_t cleanup = ld2450_sensor_deinit();
            if (cleanup != ESP_OK) ESP_LOGE("SENSOR", "ld2450  | ERROR | UART cleanup failed: %s", esp_err_to_name(cleanup));
            vTaskDelete(NULL);
            return; // No automatic initialization retry.
        } else if (last_valid >= 0 && now >= next_silence) {
            sensor_status_record_read(SENSOR_LD2450, ESP_ERR_TIMEOUT, now);
            timeouts++;
            next_silence = now + SILENCE_US;
        }
        unsigned mask = 0;
        for (unsigned i = 0; i < LD2450_TARGETS; ++i) if (report.targets[i].present) mask |= 1U << i;
        bool summary = now >= next_summary;
        // Coalesce slot-presence changes to at most one group/second. Position
        // and speed changes appear in the 10-second summary, not every frame.
        if (fresh_report && now - last_valid < SILENCE_US &&
            (summary || (mask != displayed_mask && now >= next_change_log))) {
            log_report(&report);
            displayed_mask = mask;
            next_change_log = now + 1000000;
        }
        if (summary) {
            log_diagnostics(timeouts, last_valid < 0 ? -1 : now - last_valid);
            next_summary = now + SUMMARY_US;
        }
        // Always block, even under continuous noise/backlog or SDK errors.
        // One 256-byte batch/tick can drain far more than the normal 300 B/s.
        vTaskDelay(1);
    }
}

void ld2450_reader_start(void)
{
    sensor_status_configure(SENSOR_LD2450, "ld2450", true);
    if (xTaskCreate(ld2450_reader_task, "ld2450_reader", 4096, NULL, 3, &s_ld2450_task) != pdPASS) {
        sensor_status_record_unavailable(SENSOR_LD2450, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
        ESP_LOGE("SENSOR", "ld2450  | ERROR | Reader task creation failed; other sensors continue");
    }
}

TaskHandle_t ld2450_reader_task_handle(void)
{
    sensor_status_t status;
    if (!sensor_status_get(SENSOR_LD2450, &status) || !status.initialized ||
        status.state == SENSOR_UNAVAILABLE || status.state == SENSOR_UNINITIALIZED) return NULL;
    return s_ld2450_task;
}
