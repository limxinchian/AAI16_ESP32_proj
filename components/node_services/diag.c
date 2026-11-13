/* Heartbeat message construction; see diag.h. */
#include "diag.h"

#include <string.h>

#include "pb_encode.h"

void diag_fill_heartbeat(const diag_inputs_t *in, aai16_Heartbeat *out)
{
    memset(out, 0, sizeof(*out));
    strncpy(out->node_id, in->node_id ? in->node_id : "", sizeof(out->node_id) - 1);
    out->uptime_ms = in->uptime_ms;
    out->config_generation = in->config_generation;
    out->rule_generation = in->rule_generation;
    out->bridge_generation = in->bridge_generation;
    const uint32_t cap = sizeof(out->active_sensor_ids) / sizeof(out->active_sensor_ids[0]);
    out->active_sensor_ids_count = in->n_active < cap ? in->n_active : cap;
    for (uint32_t i = 0; i < out->active_sensor_ids_count; i++) {
        out->active_sensor_ids[i] = in->active_sensor_ids[i];
    }
    out->model_state = in->model_state;
    out->model_kind = in->model_kind;
    out->model_version = in->model_version;
    out->model_reason = in->model_reason;
    out->readings_ok = in->readings_ok;
    out->read_errors = in->read_errors;
    out->overruns = in->overruns;
    out->rules_fired = in->rules_fired;
    out->rule_errors = in->rule_errors;
    out->free_heap = in->free_heap;
}

bool diag_encode_heartbeat(const aai16_Heartbeat *hb, uint8_t *buf, size_t cap, size_t *len)
{
    pb_ostream_t os = pb_ostream_from_buffer(buf, cap);
    if (!pb_encode(&os, aai16_Heartbeat_fields, hb)) {
        return false;
    }
    *len = os.bytes_written;
    return true;
}
