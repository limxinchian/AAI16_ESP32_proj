#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Fixed slots; add sensor IDs here as drivers are integrated (maximum eight).
typedef enum {
    SENSOR_DHT22,
    SENSOR_GY511,
    SENSOR_RC522,
    SENSOR_LD2450,
    SENSOR_HCSR04P,
    SENSOR_MHZ1911A,
    SENSOR_STATUS_CAPACITY = 8
} sensor_id_t;

typedef enum {
    SENSOR_DISABLED,
    SENSOR_UNINITIALIZED,
    SENSOR_READY,
    SENSOR_DEGRADED,
    SENSOR_UNAVAILABLE
} sensor_state_t;

typedef enum {
    SENSOR_STAGE_NONE,
    SENSOR_STAGE_DEPENDENCY,
    SENSOR_STAGE_INIT,
    SENSOR_STAGE_TASK,
    SENSOR_STAGE_READ
} sensor_error_stage_t;

typedef struct {
    const char *name;
    sensor_state_t state;
    bool initialized; // Remains true if initialization succeeded but task creation failed.
    sensor_error_stage_t error_stage;
    esp_err_t last_error; // Preserved across disable/enable; cleared by a successful read.
    uint32_t read_successes;
    uint32_t read_failures;
    uint32_t consecutive_failures; // Read failures only, not initialization failures.
    int64_t last_success_us; // Acquisition completion uptime; -1 means never.
} sensor_status_t;

// Configure once at startup, before the sensor's worker starts. Name must have
// static lifetime. Disabled slots are reported separately from failed sensors.
// Status/logging only: this does not initialize, stop, or recover hardware.
esp_err_t sensor_status_configure(sensor_id_t id, const char *name, bool enabled);
void sensor_status_record_init(sensor_id_t id, esp_err_t error);
void sensor_status_record_unavailable(sensor_id_t id, sensor_error_stage_t stage,
                                      esp_err_t error);
// Task-only: also logs bounded failure/recovery transitions outside the lock.
// The first error/recovery pair is immediate; flapping is coalesced (10 s).
void sensor_status_record_read(sensor_id_t id, esp_err_t error, int64_t completed_us);
// For owners that establish readiness by communication, after their warm-up.
// Counts every acquisition, including failures before the first success. A first
// failure is UNAVAILABLE; after success, failures are DEGRADED until the given
// consecutive-failure threshold. READ-stage UNAVAILABLE can recover on success.
// Does not revive dependency/init/task failures or DISABLED slots. Threshold > 0.
// Existing record_read semantics and other sensor owners are unchanged.
void sensor_status_record_probe(sensor_id_t id, esp_err_t error, int64_t completed_us,
                                uint32_t unavailable_after);
const char *sensor_status_state_name(sensor_state_t state);
// Call only from the acquisition owner, between reads. This records the gate;
// the owner must stop sampling and reset its deadline on a disabled -> enabled
// transition. Preserves all health history. Uninitialized/unavailable sensors
// reject both operations; enabling never initializes hardware or repairs a task.
esp_err_t sensor_status_set_enabled(sensor_id_t id, bool enabled);
// Probe-owner variant: accepts startup warm-up and READ-stage unavailability.
// A disabled -> enabled edge returns to UNINITIALIZED until a new probe result;
// history stays intact. Does not revive dependency/init/task failures. Caller
// must own a successfully initialized driver and a live acquisition task.
esp_err_t sensor_status_set_probe_enabled(sensor_id_t id, bool enabled);
bool sensor_status_can_sample(sensor_id_t id);
bool sensor_status_get(sensor_id_t id, sensor_status_t *out);

// Call only from app_main, after any due acquisition. Header + two lines per
// configured sensor per 60 s, no MQTT/driver calls, and no dynamic memory.
void sensor_status_log_summary(void);
