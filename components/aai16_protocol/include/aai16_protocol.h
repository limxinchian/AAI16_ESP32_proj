#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "aai16.pb.h"

#ifdef PB_ENABLE_MALLOC
#error "AAI16 protocol processing must not enable nanopb heap allocation"
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    AAI16_PROTOCOL_VERSION = 1,
    AAI16_CONFIG_FRAME_MAX = 128,
    AAI16_FRAME_MAX = 768,
    AAI16_SNAPSHOT_MAX = 512,
    AAI16_RECORD_MAX = 640,
    AAI16_CHUNK_MAX = 512,
    AAI16_MODEL_MAX = 64 * 1024,
    AAI16_PACKAGE_MAX = 72 * 1024
};
#define AAI16_COMMIT_MARKER UINT32_C(0x41414916)

// Pure protocol APIs: no MQTT, task, driver, flash, heap or global mutable state.
// Caller owns input/output buffers. They must not overlap. On failure decoded
// outputs are zeroed and encoded length is zero; partial bytes must not be used.
// Use these wrappers, not raw pb_decode(), at application trust boundaries.
aai16_ResultCode aai16_protocol_decode(const uint8_t *data, size_t length, aai16_Envelope *out);
// Config ingress only: returns a nonzero request ID for a structurally valid
// frame even when semantic validation fails. The message is still zeroed on
// failure. Malformed/oversized frames never provide an invented identity.
aai16_ResultCode aai16_config_decode(const uint8_t *data, size_t length,
                                    aai16_Envelope *out, uint64_t *request_id);
aai16_ResultCode aai16_protocol_encode(const aai16_Envelope *message,
                                     uint8_t *data, size_t capacity, size_t *length);
aai16_ResultCode aai16_protocol_validate(const aai16_Envelope *message);
aai16_ResultCode aai16_config_validate(const aai16_SensorConfig *config);
aai16_ResultCode aai16_profile_validate(const aai16_SensorProfile *profile);
bool aai16_interval_limits(aai16_SensorId sensor, uint32_t *minimum, uint32_t *maximum);

// Snapshot supplied by the acquisition owner immediately before APPLY, after
// older commands. This is not a dispatcher snapshot and does not change state.
typedef struct {
    bool driver_initialized, task_running, enabled;
    aai16_SensorHealth health;
    bool probe_read_failure; // MH-Z1911A READ-stage unavailability can recover.
    uint32_t generation;
} aai16_owner_state_t;
aai16_ResultCode aai16_config_validate_owner(const aai16_SensorConfig *config,
                                           const aai16_owner_state_t *owner);

aai16_ResultCode aai16_snapshot_validate(const aai16_ConfigSnapshot *snapshot);
aai16_ResultCode aai16_snapshot_decode(const uint8_t *data, size_t length,
                                     aai16_ConfigSnapshot *out);
aai16_ResultCode aai16_snapshot_encode(const aai16_ConfigSnapshot *snapshot,
                                     uint8_t *data, size_t capacity, size_t *length);
// Creates/validates a record in caller-owned RAM only. No persistence backend,
// boot recovery selection or flash writes are implemented in Stage 1.
aai16_ResultCode aai16_record_prepare(const aai16_ConfigSnapshot *snapshot,
                                    uint64_t sequence, aai16_PersistentConfigRecord *out);
aai16_ResultCode aai16_record_encode(const aai16_PersistentConfigRecord *record,
                                   uint8_t *data, size_t capacity, size_t *length);
aai16_ResultCode aai16_record_decode(const uint8_t *data, size_t length,
                                   aai16_PersistentConfigRecord *record,
                                   aai16_ConfigSnapshot *snapshot);

#ifdef __cplusplus
}
#endif
