#pragma once
#include "motion_core.h"
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef void (*motion_result_callback_t)(const motion_result_t *result);
// Startup only, before reader creation. Callback runs on the ML task and must
// be bounded/nonblocking. Failure never prevents other sensors from starting.
esp_err_t motion_ml_start(uint32_t boot_id, motion_result_callback_t callback);
// Single acquisition owner, task context only. Copies with zero queue wait.
bool motion_ml_submit(const motion_sample_t *sample);
// Owner calls after each applied config and before ACK, even when disabled.
// Invalidates queued/in-flight windows; preserves the reader's global defaults.
void motion_ml_configure(uint32_t interval_ms, uint32_t generation, bool enabled);
// Reader task marks available on entry. Until then, initialization/task failure
// is reported as sensor_unavailable, independent of the configured interval.
void motion_ml_set_sensor_available(bool available);
bool motion_ml_model_ready(void);
#ifdef __cplusplus
}
#endif
