#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motion_model_control.h"
#ifdef __cplusplus
extern "C" {
#endif
enum { MOTION_SAMPLES = 20, MOTION_AXES = 3, MOTION_INPUTS = 60, MOTION_STRIDE = 10 };
typedef enum {
    MOTION_COLLECTING, MOTION_PREDICTION, MOTION_WAIT_100MS,
    MOTION_INVALID_SAMPLE, MOTION_DISCONTINUITY, MOTION_NO_DATA,
    MOTION_QUEUE_OVERFLOW, MOTION_INIT_FAILED, MOTION_INVOKE_FAILED,
    MOTION_SENSOR_DISABLED, MOTION_SENSOR_UNAVAILABLE
} motion_status_t;
typedef struct {
    uint32_t boot_id, sequence, generation, interval_ms;
    int64_t acquired_us;
    float xyz[MOTION_AXES];
    bool valid;
} motion_sample_t;
typedef struct {
    motion_sample_t samples[MOTION_SAMPLES];
    unsigned count;
    uint32_t resets;
} motion_window_t;
// A plain bounded result; serialization belongs to the transport adapter.
typedef struct motion_result {
    char model_version[40];
    motion_model_identity_t model_identity;
    uint32_t model_epoch;
    motion_status_t status;
    bool model_ready;
    int8_t class_id; // -1 when there is no prediction; REST=0, MOVING=1.
    int8_t output_int8[2];
    float logits[2];
    uint32_t boot_id, first_sequence, last_sequence, generation, interval_ms;
    int64_t window_start_us, window_end_us, uptime_us, inference_us;
    uint32_t clipped, clipped_total, queue_drops, window_resets, invoke_failures;
    size_t arena_used_bytes;
} motion_result_t;
void motion_window_reset(motion_window_t *window);
// On success, samples[0..19] are chronological. Call advance after consuming.
bool motion_window_push(motion_window_t *window, const motion_sample_t *sample,
                        motion_status_t *status);
void motion_window_advance(motion_window_t *window);
bool motion_preprocess(const float raw[MOTION_SAMPLES][MOTION_AXES], float features[MOTION_INPUTS]);
bool motion_quantize(const float features[MOTION_INPUTS], float scale, int zero_point,
                     int8_t encoded[MOTION_INPUTS], uint32_t *clipped);
int motion_decode(const int8_t output[2], float scale, int zero_point, float logits[2]);
const char *motion_status_name(motion_status_t status);
#ifdef __cplusplus
}
#endif
