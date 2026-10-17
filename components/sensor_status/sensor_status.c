#include "sensor_status.h"

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define STATUS_REPORT_US 60000000LL

static sensor_status_t s_sensors[SENSOR_STATUS_CAPACITY];
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
    portENTER_CRITICAL(&s_lock);
    sensor_status_t *status = &s_sensors[id];
    if (can_sample(status)) {
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
    }
    portEXIT_CRITICAL(&s_lock);
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
        "disabled", "uninitialized", "ready", "degraded", "unavailable"
    };
    static const char *const stages[] = {"none", "dependency", "init", "task", "read"};
    for (int id = 0; id < SENSOR_STATUS_CAPACITY; ++id) {
        sensor_status_t status;
        if (!sensor_status_get((sensor_id_t)id, &status)) continue;
        // Copy under the lock, then format/log outside it. No driver/SDK locks.
        char last_success[24] = "never";
        if (status.last_success_us >= 0) {
            snprintf(last_success, sizeof(last_success), "%" PRId64,
                     status.last_success_us / 1000);
        }
        ESP_LOGI("SENSORS", "%s state=%s init=%d stage=%s error=%s "
                 "ok=%" PRIu32 " failed=%" PRIu32 " consecutive=%" PRIu32 " last_ok_ms=%s",
                 status.name, states[status.state], status.initialized,
                 stages[status.error_stage], esp_err_to_name(status.last_error),
                 status.read_successes, status.read_failures,
                 status.consecutive_failures, last_success);
    }
}
