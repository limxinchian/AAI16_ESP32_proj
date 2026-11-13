#pragma once
/* Node-to-node bridge (handoff 6.3): another node's physical sensor appears locally as a VIRTUAL
 * sensor (id 128..255) with its own freshness policy. Rules, the ML selector and telemetry then treat
 * it like any local sensor.
 *
 * Safety: a virtual source may never name this node, ids and (node, sensor) pairs are unique, only
 * physical remote sensors (1..127) can be bridged, and only LOCAL physical readings are ever
 * published back out (callers must publish readings with sensor_id <= 127 only) - so forwarding loops
 * cannot form. Freshness uses the local receive time; the remote timestamp is ignored for policy.
 */
#include <stdbool.h>
#include <stdint.h>

#include "aai16.pb.h"
#include "ml_selector.h"

#define BR_MAX_SOURCES 4
#define BR_VIRTUAL_MIN 128
#define BR_VIRTUAL_MAX 255

typedef struct {
    aai16_VirtualSource src;
    bool have, stale_reported;
    uint64_t last_rx_us;
    uint32_t rx_ok, rejected, stale_events;
} br_slot_t;

typedef struct {
    char node_id[16]; /* 15 characters + NUL: matches the nanopb max_size:16 strings on the wire */
    br_slot_t slot[BR_MAX_SOURCES];
    uint32_t count, generation;
} bridge_t;

typedef enum { BR_OK = 0, BR_UNKNOWN_SOURCE, BR_MALFORMED, BR_SHAPE_MISMATCH, BR_REMOTE_INVALID } br_result_t;

static inline bool bridge_is_virtual(uint32_t sensor_id) { return sensor_id >= BR_VIRTUAL_MIN && sensor_id <= BR_VIRTUAL_MAX; }

void bridge_init(bridge_t *b, const char *node_id);
bool bridge_validate(const bridge_t *b, const aai16_BridgeApply *set, aai16_Reason *reason, uint32_t *index);
bool bridge_apply(bridge_t *b, const aai16_BridgeApply *set, aai16_Reason *reason, uint32_t *index);

/* A Reading payload received from `from_node`'s telemetry topic. On BR_OK, *out is the local virtual
 * reading (timestamp = local receive time). BR_REMOTE_INVALID still fills *out with valid=false. */
br_result_t bridge_on_remote(bridge_t *b, const char *from_node, const uint8_t *payload, size_t len,
                             uint64_t now_us, aai16_Reading *out);

/* Emits one invalid (timeout) reading per source the first time it goes stale. Returns count written. */
uint32_t bridge_poll(bridge_t *b, uint64_t now_us, aai16_Reading *out, uint32_t cap);

/* ml_lookup_fn-compatible: lets the ML selector validate virtual sensors. */
bool bridge_lookup(const bridge_t *b, uint32_t sensor_id, ml_source_t *out);
