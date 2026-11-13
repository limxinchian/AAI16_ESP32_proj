#pragma once
/* Config Manager: decode -> validate -> two-phase apply with rollback.
 *
 * Plain C with no ESP-IDF dependency so it builds on the host. Bus engines,
 * NVS and MQTT are reached only through cm_hooks_t.
 *
 * Threading: all cm_* calls must come from one task (the config task). Other
 * tasks must read the running profiles through a copy taken by that task.
 * Instances are large (~20 KB); declare them static, never on a task stack.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aai16.pb.h"
#include "config_limits.h"

typedef struct {
    aai16_Reason reason;
    uint32_t sensor_id;     /* 0 when the fault is request-level */
    uint32_t profile_index; /* index in the request's profile list */
    uint32_t detail;        /* op or decode-field index within that profile */
} cm_error_t;

typedef struct {
    uint32_t generation;
    bool present[CM_BUS_COUNT];                  /* index = aai16_BusType - 1: a profile is stored for the bus */
    bool sleeping[CM_BUS_COUNT];                 /* ... and it is DORMANT (stored, not running) */
    aai16_SensorProfile profile[CM_BUS_COUNT];
} cm_state_t;

typedef struct {
    /* Start (or restart) a bus with this validated profile. Return false if the
     * engine could not start it; the manager then rolls back. */
    bool (*apply)(const aai16_SensorProfile *profile, void *ctx);
    /* Stop whatever runs on this bus. Must not fail. */
    void (*stop)(aai16_BusType bus, void *ctx);
    /* Store the new committed configuration. Return false on failure. */
    bool (*persist)(const aai16_CommittedConfig *cfg, void *ctx);
    /* Deliver an ACK (the caller encodes it with cm_encode_ack). */
    void (*ack)(const aai16_ApplyAck *ack, void *ctx);
    void *ctx;
} cm_hooks_t;

typedef struct {
    cm_hooks_t hooks;
    cm_state_t committed;
    cm_state_t candidate;       /* working copy; only swapped in on success */
    aai16_ConfigApply rx;       /* decode target for cm_handle_bytes */
    aai16_CommittedConfig store;/* scratch for persist / restore */
} config_manager_t;

/* ---- validation (pure functions) ---- */

bool cm_pin_allowed(uint32_t gpio);
/* Validates one profile in isolation; err->detail carries the op/decode index. */
bool cm_validate_profile(const aai16_SensorProfile *p, cm_error_t *err);
/* Validates a whole request (count, per-profile, duplicate ids/buses, pins). */
bool cm_validate_apply(const aai16_ConfigApply *req, cm_error_t *err);

/* ---- lifecycle ---- */

void cm_init(config_manager_t *cm, const cm_hooks_t *hooks);

/* Handle an already-decoded request. Emits ADMITTED then APPLIED, or REJECTED. */
void cm_handle_apply(config_manager_t *cm, const aai16_ConfigApply *req);

/* Decode an MQTT payload and handle it. Undecodable input yields a REJECTED ack
 * with REASON_MALFORMED and changes nothing. */
void cm_handle_bytes(config_manager_t *cm, const uint8_t *buf, size_t len);

/* ---- persistence ---- */

bool cm_encode_committed(config_manager_t *cm, uint8_t *buf, size_t cap, size_t *out_len);
/* Decode + fully re-validate a stored blob; on any problem nothing is loaded. On
 * success the profiles are started through hooks->apply. */
bool cm_restore(config_manager_t *cm, const uint8_t *buf, size_t len);

/* ---- helpers ---- */

bool cm_encode_ack(const aai16_ApplyAck *ack, uint8_t *buf, size_t cap, size_t *out_len);
/* The RUNNING profile of a bus (NULL if none, or if the bus only holds a dormant profile). */
const aai16_SensorProfile *cm_committed_profile(const config_manager_t *cm, aai16_BusType bus);
/* The stored DORMANT profile with this sensor id (enabled = false, dormant = true), or NULL. */
const aai16_SensorProfile *cm_dormant_by_id(const config_manager_t *cm, uint32_t sensor_id);
/* True if a running sensor profile owns this GPIO (used to keep rule actions off sensor pins). */
bool cm_pin_in_use(const config_manager_t *cm, uint32_t gpio);
