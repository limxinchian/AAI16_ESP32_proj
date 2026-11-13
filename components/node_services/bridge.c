/* Node-to-node bridge: another node's reading becomes a virtual sensor (128..255) here, stamped with the local receive
 * time, with its own freshness handling. See bridge.h for the safety rules. */
#include "bridge.h"

#include <string.h>

#include "pb_decode.h"

void bridge_init(bridge_t *b, const char *node_id)
{
    memset(b, 0, sizeof(*b));
    strncpy(b->node_id, node_id, sizeof(b->node_id) - 1);
}

static bool bad(aai16_Reason *r, uint32_t *i, uint32_t idx)
{
    *r = aai16_Reason_REASON_BAD_BRIDGE;
    *i = idx;
    return false;
}

bool bridge_validate(const bridge_t *b, const aai16_BridgeApply *set, aai16_Reason *reason, uint32_t *index)
{
    *reason = aai16_Reason_REASON_NONE;
    *index = 0;
    if (set->generation == 0 || set->generation <= b->generation) {
        *reason = set->generation == b->generation ? aai16_Reason_REASON_DUPLICATE_GENERATION
                                                    : aai16_Reason_REASON_STALE_GENERATION;
        return false;
    }
    if (set->sources_count > BR_MAX_SOURCES) {
        *reason = aai16_Reason_REASON_BAD_BRIDGE;
        *index = set->sources_count;
        return false;
    }
    for (uint32_t i = 0; i < set->sources_count; i++) {
        const aai16_VirtualSource *s = &set->sources[i];
        if (!bridge_is_virtual(s->local_sensor_id) || s->remote_node[0] == '\0' ||
            strcmp(s->remote_node, b->node_id) == 0 /* would loop back to ourselves */ ||
            s->remote_sensor_id < 1 || s->remote_sensor_id > 127 || /* only physical sensors are bridged */
            s->max_age_ms < 100 || s->max_age_ms > 600000 || s->period_ms < 10 || s->period_ms > 3600000 ||
            s->kinds_count < 1 || s->kinds_count > 8) {
            return bad(reason, index, i);
        }
        for (uint32_t k = 0; k < s->kinds_count; k++) {
            if (s->kinds[k] < aai16_ValueKind_VALUE_ACCEL_X_G || s->kinds[k] > aai16_ValueKind_VALUE_RADAR_DISTANCE_MM) {
                return bad(reason, index, i);
            }
        }
        for (uint32_t j = 0; j < i; j++) {
            const aai16_VirtualSource *o = &set->sources[j];
            if (o->local_sensor_id == s->local_sensor_id ||
                (strcmp(o->remote_node, s->remote_node) == 0 && o->remote_sensor_id == s->remote_sensor_id)) {
                return bad(reason, index, i);
            }
        }
    }
    return true;
}

bool bridge_apply(bridge_t *b, const aai16_BridgeApply *set, aai16_Reason *reason, uint32_t *index)
{
    if (!bridge_validate(b, set, reason, index)) {
        return false;
    }
    memset(b->slot, 0, sizeof(b->slot)); /* old values must not leak into the new mapping */
    for (uint32_t i = 0; i < set->sources_count; i++) {
        b->slot[i].src = set->sources[i];
    }
    b->count = set->sources_count;
    b->generation = set->generation;
    return true;
}

br_result_t bridge_on_remote(bridge_t *b, const char *from_node, const uint8_t *payload, size_t len,
                             uint64_t now_us, aai16_Reading *out)
{
    static aai16_Reading in; /* decode target; single owner task */
    memset(&in, 0, sizeof(in));
    pb_istream_t is = pb_istream_from_buffer(payload, len);
    if (!pb_decode(&is, aai16_Reading_fields, &in)) {
        return BR_MALFORMED;
    }
    for (uint32_t i = 0; i < b->count; i++) {
        br_slot_t *s = &b->slot[i];
        if (strcmp(s->src.remote_node, from_node) != 0 || s->src.remote_sensor_id != in.sensor_id) {
            continue;
        }
        memset(out, 0, sizeof(*out));
        out->sensor_id = s->src.local_sensor_id;
        out->generation = b->generation;
        out->timestamp_us = now_us; /* local receive time drives freshness */
        if (!in.valid) {
            out->valid = false;
            out->error = in.error ? in.error : aai16_ReadError_READ_BAD_FRAME;
            s->rejected++;
            return BR_REMOTE_INVALID;
        }
        if (in.values_count != s->src.kinds_count) {
            s->rejected++;
            return BR_SHAPE_MISMATCH;
        }
        out->valid = true;
        out->values_count = in.values_count;
        memcpy(out->values, in.values, sizeof(float) * in.values_count);
        s->have = true;
        s->stale_reported = false;
        s->last_rx_us = now_us;
        s->rx_ok++;
        return BR_OK;
    }
    return BR_UNKNOWN_SOURCE;
}

uint32_t bridge_poll(bridge_t *b, uint64_t now_us, aai16_Reading *out, uint32_t cap)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < b->count && n < cap; i++) {
        br_slot_t *s = &b->slot[i];
        if (s->have && !s->stale_reported && now_us - s->last_rx_us > (uint64_t)s->src.max_age_ms * 1000u) {
            s->stale_reported = true;
            s->stale_events++;
            memset(&out[n], 0, sizeof(out[n]));
            out[n].sensor_id = s->src.local_sensor_id;
            out[n].generation = b->generation;
            out[n].timestamp_us = now_us;
            out[n].valid = false;
            out[n].error = aai16_ReadError_READ_TIMEOUT;
            n++;
        }
    }
    return n;
}

bool bridge_lookup(const bridge_t *b, uint32_t sensor_id, ml_source_t *out)
{
    for (uint32_t i = 0; i < b->count; i++) {
        if (b->slot[i].src.local_sensor_id == sensor_id) {
            memset(out, 0, sizeof(*out));
            out->active = true;
            out->period_ms = b->slot[i].src.period_ms;
            out->output_count = b->slot[i].src.kinds_count;
            for (uint32_t k = 0; k < b->slot[i].src.kinds_count && k < 8; k++) out->kinds[k] = b->slot[i].src.kinds[k];
            return true;
        }
    }
    return false;
}
