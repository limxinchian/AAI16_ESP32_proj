#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum { MOTION_PACKAGE_HEADER = 256, MOTION_PACKAGE_GOLDEN = 320, MOTION_CANDIDATE_ARENA = 16384 };
typedef struct {
    uint32_t family, profile, model_size;
    char version[40];
    uint8_t model_sha256[32];
    float input_scale, output_scale;
    int32_t input_zero_point, output_zero_point;
} motion_candidate_info_t;
// Read-only validation of external bytes. Caller owns aligned flash mapping and
// separate internal arena; never substitutes or reinitializes the live engine.
bool motion_candidate_validate(const uint8_t *package, size_t size, uint8_t *arena,
                               size_t arena_size, motion_candidate_info_t *info, char detail[64]);
// Serialize TFLM init/Invoke callers: pinned FC wrapper has a shared timer counter.
void motion_runtime_enter(void);
void motion_runtime_leave(void);
#ifdef __cplusplus
}
#endif
