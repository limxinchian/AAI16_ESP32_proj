#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Wire-independent bounded identities. source: 1=bundled, 2=stored.
typedef struct {
    uint32_t family, source, bank;
    char version[40];
    uint8_t sha256[32];
} motion_model_identity_t;
typedef struct {
    motion_model_identity_t active, previous;
    uint32_t epoch;
    bool ready, has_previous, rolled_back;
    char detail[64];
} motion_model_state_t;
typedef struct {
    motion_model_identity_t target;
    uint32_t expected_epoch;
    bool rollback;
    // Borrowed until completion. Runtime copies stored bytes into owned RAM.
    const uint8_t *package;
    size_t package_size;
} motion_model_command_t;
typedef enum {
    MOTION_CHANGE_OK, MOTION_CHANGE_STALE, MOTION_CHANGE_INVALID,
    MOTION_CHANGE_FAILED, MOTION_CHANGE_BUSY
} motion_change_result_t;
typedef struct {
    motion_change_result_t result;
    motion_model_state_t state;
    char detail[64];
} motion_model_reply_t;
void motion_ml_model_state(motion_model_state_t *state);
// Immutable provider identity; never derived from client metadata or live state.
void motion_model_bundled_a_identity(motion_model_identity_t *identity);
// Single model-control worker only. Queue/poll never waits on the sensor task.
// Borrowed command package must remain alive until take_reply succeeds.
bool motion_ml_request_model(const motion_model_command_t *command);
bool motion_ml_take_model_reply(motion_model_reply_t *reply);
#ifdef __cplusplus
}
#endif
