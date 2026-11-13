#pragma once
/* Heartbeat construction (handoff 6.4). Pure: the caller supplies the numbers. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aai16.pb.h"

typedef struct {
    const char *node_id;
    uint64_t uptime_ms;
    uint32_t config_generation, rule_generation, bridge_generation;
    const uint32_t *active_sensor_ids;
    uint32_t n_active;
    aai16_ModelState model_state;
    aai16_ModelKind model_kind;
    uint32_t model_version;
    aai16_ModelReason model_reason;
    uint32_t readings_ok, read_errors, overruns, rules_fired, rule_errors, free_heap;
} diag_inputs_t;

void diag_fill_heartbeat(const diag_inputs_t *in, aai16_Heartbeat *out);
bool diag_encode_heartbeat(const aai16_Heartbeat *hb, uint8_t *buf, size_t cap, size_t *len);
