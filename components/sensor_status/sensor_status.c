#include "sensor_status.h"

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define STATUS_REPORT_US 60000000LL
#define TRANSITION_LOG_US 10000000LL

static sensor_status_t s_sensors[SENSOR_STATUS_CAPACITY];
// Presentation only: health counters/state always update, even during flapping.
static struct {
    esp_err_t reported_error;
    int64_t next_error_us, next_recovery_us;
    uint32_t coalesced;
} s_read_logs[SENSOR_STATUS_CAPACITY];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static bool valid_id(sensor_id_t id)
{
    return (unsigned)id < SENSOR_STATUS_CAPACITY;
}

static bool can_sample(const sensor_status_t *status)
{
    return status->name != NULL &&
           (status->state == SENSOR_READY || status->state == SENSOR_DEGRADED);
}

esp_err_t sensor_status_configure(sensor_id_t id, const char *name, bool enabled)
{
    if (!valid_id(id) || name == NULL || name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_lock);
    s_sensors[id] = (sensor_status_t) {
        .name = name,
        .state = enabled ? SENSOR_UNINITIALIZED : SENSOR_DISABLED,
        .last_success_us = -1,
    };
    s_read_logs[id].reported_error = ESP_OK;
    s_read_logs[id].next_error_us = s_read_logs[id].next_recovery_us = 0;
    s_read_logs[id].coalesced = 0;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

void sensor_status_record_init(sensor_id_t id, esp_err_t error)
{
    if (!valid_id(id)) return;
    portENTER_CRITICAL(&s_lock);
    sensor_status_t *status = &s_sensors[id];
    if (status->name != NULL && status->state == SENSOR_UNINITIALIZED) {
        status->initialized = error == ESP_OK;
        status->state = error == ESP_OK ? SENSOR_READY : SENSOR_UNAVAILABLE;
        status->error_stage = error == ESP_OK ? SENSOR_STAGE_NONE : SENSOR_STAGE_INIT;
        status->last_error = error;
    }
    portEXIT_CRITICAL(&s_lock);
}

void sensor_status_record_unavailable(sensor_id_t id, sensor_error_stage_t stage,
                                      esp_err_t error)
{
    if (!valid_id(id) || error == ESP_OK ||
        (stage != SENSOR_STAGE_DEPENDENCY && stage != SENSOR_STAGE_INIT &&
         stage != SENSOR_STAGE_TASK)) return;
    portENTER_CRITICAL(&s_lock);
    sensor_status_t *status = &s_sensors[id];
    if (status->name != NULL && status->state != SENSOR_DISABLED) {
        status->state = SENSOR_UNAVAILABLE;
        status->error_stage = stage;
        status->last_error = error;
    }
    portEXIT_CRITICAL(&s_lock);
}

void sensor_status_record_read(sensor_id_t id, esp_err_t error, int64_t completed_us)
{
    if (!valid_id(id)) return;
    bool emit = false;
    sensor_status_t snapshot = {0};
    portENTER_CRITICAL(&s_lock);
    sensor_status_t *status = &s_sensors[id];
    if (can_sample(status)) {
        esp_err_t previous_error = status->last_error;
        status->last_error = error;
        if (error == ESP_OK) {
            status->state = SENSOR_READY;
            status->error_stage = SENSOR_STAGE_NONE;
            status->read_successes++;
            status->consecutive_failures = 0;
            status->last_success_us = completed_us;
        } else {
            status->state = SENSOR_DEGRADED;
            status->error_stage = SENSOR_STAGE_READ;
            status->read_failures++;
            status->consecutive_failures++;
        }
        // First failure and first recovery are immediate. Repeated identical
        // errors are silent; rapid alternation is bounded to one warning and
        // one recovery per 10 s. A pending stable transition logs on the next
        // read after its deadline. Never delay or discard an acquisition.
        int64_t *deadline = error == ESP_OK ? &s_read_logs[id].next_recovery_us :
                                             &s_read_logs[id].next_error_us;
        if (error != s_read_logs[id].reported_error && completed_us >= *deadline) {
            *deadline = completed_us + TRANSITION_LOG_US;
            s_read_logs[id].reported_error = error;
            snapshot = *status;
            emit = true;
        } else if (error != previous_error) {
            s_read_logs[id].coalesced++;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    // ESP-IDF serializes each complete log call; never format or log in a
    // critical section. This API is task-only and called by each sensor owner.
    if (emit && error != ESP_OK) {
        ESP_LOGW("SENSOR", "%-7.16s | ERROR | state=DEGRADED error=%s failed_total=%" PRIu32
                 " consecutive=%" PRIu32,
                 snapshot.name, esp_err_to_name(error), snapshot.read_failures,
                 snapshot.consecutive_failures);
    } else if (emit) {
        ESP_LOGI("SENSOR", "%-7.16s | HEALTH | recovery=READY ok_total=%" PRIu32 " failed_total=%" PRIu32,
                 snapshot.name, snapshot.read_successes, snapshot.read_failures);
    }
}

esp_err_t sensor_status_set_enabled(sensor_id_t id, bool enabled)
{
    if (!valid_id(id)) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_lock);
    sensor_status_t *status = &s_sensors[id];
    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (status->initialized && (can_sample(status) || status->state == SENSOR_DISABLED)) {
        // Disabling is an intentional pause, not a successful read. In
        // particular, re-enabling must not erase a previous read failure.
        status->state = !enabled ? SENSOR_DISABLED :
            (status->error_stage == SENSOR_STAGE_READ ? SENSOR_DEGRADED : SENSOR_READY);
        result = ESP_OK;
    }
    portEXIT_CRITICAL(&s_lock);
    return result;
}

bool sensor_status_can_sample(sensor_id_t id)
{
    if (!valid_id(id)) return false;
    portENTER_CRITICAL(&s_lock);
    bool available = can_sample(&s_sensors[id]);
    portEXIT_CRITICAL(&s_lock);
    return available;
}

bool sensor_status_get(sensor_id_t id, sensor_status_t *out)
{
    if (!valid_id(id) || out == NULL) return false;
    portENTER_CRITICAL(&s_lock);
    bool configured = s_sensors[id].name != NULL;
    if (configured) *out = s_sensors[id];
    portEXIT_CRITICAL(&s_lock);
    return configured;
}

void sensor_status_log_summary(void)
{
    static int64_t next_report_us;
    int64_t now_us = esp_timer_get_time();
    if (now_us < next_report_us) return;
    next_report_us = now_us + STATUS_REPORT_US;

    static const char *const states[] = {
        "DISABLED", "UNINITIALIZED", "READY", "DEGRADED", "UNAVAILABLE"
    };
    static const char *const stages[] = {"NONE", "DEPENDENCY", "INIT", "TASK", "READ"};
    ESP_LOGI("SENSORS", "HEALTH SUMMARY | counters=cumulative last_ok_ms=uptime");
    for (int id = 0; id < SENSOR_STATUS_CAPACITY; ++id) {
        sensor_status_t status;
        if (!sensor_status_get((sensor_id_t)id, &status)) continue;
        // Copy under the lock, then format/log outside it. No driver/SDK locks.
        char last_success[24] = "never";
        if (status.last_success_us >= 0) {
            snprintf(last_success, sizeof(last_success), "%" PRId64,
                     status.last_success_us / 1000);
        }
        uint32_t coalesced;
        portENTER_CRITICAL(&s_lock);
        coalesced = s_read_logs[id].coalesced;
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGI("SENSORS", "%-7.16s | HEALTH | state=%s ok=%" PRIu32 " failed=%" PRIu32
                 " consecutive=%" PRIu32 " last_ok_ms=%s",
                 status.name, states[status.state],
                 status.read_successes, status.read_failures,
                 status.consecutive_failures, last_success);
        ESP_LOGI("SENSORS", "%-7.16s | HEALTH | init=%d stage=%s error=%s transitions_coalesced_total=%" PRIu32,
                 status.name, status.initialized, stages[status.error_stage],
                 esp_err_to_name(status.last_error), coalesced);
    }
}
