#include "aai16_protocol.h"
#include <math.h>
#include <string.h>
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"

#define R(name) aai16_ResultCode_RESULT_##name
#define S(name) aai16_SensorId_SENSOR_##name
#define P(name) aai16_Phase_PHASE_##name
#define REQUIRE(condition) do { if (!(condition)) return R(MISSING_FIELD); } while (0)
#define CHECK(call) do { aai16_ResultCode result_ = (call); if (result_ != R(OK)) return result_; } while (0)

_Static_assert(aai16_Envelope_size <= AAI16_FRAME_MAX, "Envelope exceeds transport budget");
_Static_assert(aai16_ConfigSnapshot_size <= AAI16_SNAPSHOT_MAX, "Snapshot exceeds record budget");
_Static_assert(aai16_PersistentConfigRecord_size <= AAI16_RECORD_MAX, "Record exceeds budget");

static bool sensor_valid(aai16_SensorId id) { return id >= S(DHT22) && id <= S(MAX6675); }
static bool phase_valid(aai16_Phase phase) { return phase >= P(VALIDATE) && phase <= P(COMMIT); }
static bool result_valid(aai16_ResultCode result) { return result >= R(OK) && result <= R(INTERNAL_ERROR); }
static bool text_valid(const char *text, size_t capacity)
{
    return memchr(text, '\0', capacity) != NULL;
}
static bool version_valid(const char *version)
{
    if (!text_valid(version, 40) || !version[0]) return false;
    for (size_t i = 0; version[i]; ++i) {
        char c = version[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.')) return false;
    }
    return true;
}

bool aai16_interval_limits(aai16_SensorId sensor, uint32_t *minimum, uint32_t *maximum)
{
    // Kept independent of ESP-IDF headers; equivalence is tested against the
    // current MQTT rules and driver constants. LD2450 controls publication only.
    static const uint32_t limits[8][2] = {
        {2000,10000}, {100,10000}, {200,2000}, {500,10000},
        {100,10000}, {5000,60000}, {500,10000}, {250,10000}
    };
    if (!sensor_valid(sensor) || !minimum || !maximum) return false;
    *minimum = limits[(unsigned)sensor - 1][0];
    *maximum = limits[(unsigned)sensor - 1][1];
    return true;
}
static bool interval_valid(aai16_SensorId sensor, uint32_t interval)
{
    uint32_t low, high;
    return aai16_interval_limits(sensor, &low, &high) && interval >= low && interval <= high;
}

aai16_ResultCode aai16_config_validate(const aai16_SensorConfig *c)
{
    REQUIRE(c && c->has_sensor);
    if (!sensor_valid(c->sensor)) return R(UNSUPPORTED_SENSOR);
    if (c->has_enabled && c->has_interval_ms) return R(AMBIGUOUS_OPERATION);
    REQUIRE(c->has_enabled || c->has_interval_ms);
    if (c->has_interval_ms && !interval_valid(c->sensor, c->interval_ms)) return R(OUT_OF_RANGE);
    return R(OK);
}
aai16_ResultCode aai16_profile_validate(const aai16_SensorProfile *p)
{
    REQUIRE(p && p->has_sensor && p->has_enabled && p->has_interval_ms);
    if (!sensor_valid(p->sensor)) return R(UNSUPPORTED_SENSOR);
    return interval_valid(p->sensor, p->interval_ms) ? R(OK) : R(OUT_OF_RANGE);
}
aai16_ResultCode aai16_config_validate_owner(const aai16_SensorConfig *c,
                                           const aai16_owner_state_t *owner)
{
    CHECK(aai16_config_validate(c));
    REQUIRE(owner);
    if (!owner->driver_initialized || !owner->task_running) return R(SENSOR_UNAVAILABLE);
    bool probe = c->sensor == S(MHZ1911A);
    if (owner->health < aai16_SensorHealth_HEALTH_DISABLED ||
        owner->health > aai16_SensorHealth_HEALTH_UNAVAILABLE) return R(SENSOR_UNAVAILABLE);
    if ((owner->health == aai16_SensorHealth_HEALTH_UNINITIALIZED && !probe) ||
        (owner->health == aai16_SensorHealth_HEALTH_UNAVAILABLE &&
         !(probe && owner->probe_read_failure))) return R(SENSOR_UNAVAILABLE);
    if (c->has_expected_generation && c->expected_generation != owner->generation)
        return R(STALE_GENERATION);
    if (c->has_interval_ms && !owner->enabled) return R(SENSOR_DISABLED);
    return R(OK);
}

// Reject ambiguous duplicate singular fields/oneofs BEFORE nanopb's usual
// last-one-wins decoding, as the existing JSON config rejects duplicates too.
// v1 deliberately rejects unknown fields at every level, rather than silently
// ignoring an unsupported setting. Any schema extension requires negotiation.
static bool wire_check(pb_istream_t *stream, const pb_msgdesc_t *desc, unsigned depth)
{
    if (depth > 8 || desc->field_count > 64) return false;
    uint64_t seen = 0;
    bool oneof_seen = false;
    while (stream->bytes_left) {
        pb_wire_type_t wire;
        uint32_t tag;
        bool eof;
        pb_field_iter_t field;
        if (!pb_decode_tag(stream, &wire, &tag, &eof) || eof || !tag ||
            !pb_field_iter_begin(&field, desc, NULL) || !pb_field_iter_find(&field, tag)) return false;
        uint64_t bit = UINT64_C(1) << field.index;
        if (PB_HTYPE(field.type) != PB_HTYPE_REPEATED && (seen & bit)) return false;
        seen |= bit;
        if (PB_HTYPE(field.type) == PB_HTYPE_ONEOF) {
            if (oneof_seen) return false; // Each v1 message has at most one oneof group.
            oneof_seen = true;
        }
        if (PB_LTYPE_IS_SUBMSG(field.type)) {
            pb_istream_t sub;
            if (wire != PB_WT_STRING || !pb_make_string_substream(stream, &sub)) return false;
            if (!wire_check(&sub, field.submsg_desc, depth + 1) ||
                !pb_close_string_substream(stream, &sub)) return false;
        } else if (PB_LTYPE(field.type) == PB_LTYPE_BOOL) {
            uint64_t value;
            if (wire != PB_WT_VARINT || !pb_decode_varint(stream, &value) || value > 1) return false;
        } else if (PB_LTYPE(field.type) == PB_LTYPE_STRING) {
            pb_istream_t sub;
            if (wire != PB_WT_STRING || !pb_make_string_substream(stream, &sub)) return false;
            while (sub.bytes_left) {
                pb_byte_t value;
                if (!pb_read(&sub, &value, 1) || value == 0) return false;
            }
            if (!pb_close_string_substream(stream, &sub)) return false;
        } else if (!pb_skip_field(stream, wire)) return false;
    }
    return true;
}
static aai16_ResultCode decode(const uint8_t *data, size_t length, size_t maximum,
                               const pb_msgdesc_t *desc, void *out, size_t object_size)
{
    if (!out) return R(MISSING_FIELD);
    memset(out, 0, object_size);
    if (!data || !length) return R(MALFORMED);
    if (length > maximum) return R(TOO_LARGE);
    pb_istream_t wire = pb_istream_from_buffer(data, length);
    if (!wire_check(&wire, desc, 0)) return R(MALFORMED);
    pb_istream_t input = pb_istream_from_buffer(data, length);
    if (!pb_decode(&input, desc, out) || input.bytes_left) {
        memset(out, 0, object_size);
        return R(MALFORMED);
    }
    return R(OK);
}
static aai16_ResultCode encode(const pb_msgdesc_t *desc, const void *message,
                               uint8_t *data, size_t capacity, size_t maximum, size_t *length)
{
    if (length) *length = 0;
    REQUIRE(message && data && length);
    size_t required;
    if (!pb_get_encoded_size(&required, desc, message)) return R(MALFORMED);
    if (required > capacity || required > maximum) return R(TOO_LARGE);
    pb_ostream_t output = pb_ostream_from_buffer(data, capacity);
    if (!pb_encode(&output, desc, message)) return R(MALFORMED);
    *length = output.bytes_written;
    return R(OK);
}

static aai16_ResultCode identity_validate(const aai16_ModelIdentity *m)
{
    REQUIRE(m->has_family && m->has_version && m->has_sha256 && m->has_source);
    if ((m->family != aai16_ModelFamily_MODEL_A && m->family != aai16_ModelFamily_MODEL_B) ||
        !version_valid(m->version) || m->sha256.size != 32) return R(OUT_OF_RANGE);
    if (m->source == aai16_ModelSource_SOURCE_BUNDLED)
        return !m->has_bank ? R(OK) : R(OUT_OF_RANGE);
    if (m->source != aai16_ModelSource_SOURCE_STORED || !m->has_bank || m->bank > 1) return R(OUT_OF_RANGE);
    return R(OK);
}
static bool vector_valid(const aai16_Vector3 *v)
{
    return v->has_x && v->has_y && v->has_z && isfinite(v->x) && isfinite(v->y) && isfinite(v->z);
}
static aai16_ResultCode telemetry_validate(const aai16_SensorTelemetry *t)
{
    REQUIRE(t->has_sensor && t->has_acquired_us && t->has_uptime_us);
    if (!sensor_valid(t->sensor)) return R(UNSUPPORTED_SENSOR);
    if (t->acquired_us > t->uptime_us || t->uptime_us > INT64_MAX ||
        (t->has_health && (t->health < aai16_SensorHealth_HEALTH_DISABLED ||
                          t->health > aai16_SensorHealth_HEALTH_UNAVAILABLE)) ||
        (t->has_interval_ms && !interval_valid(t->sensor, t->interval_ms))) return R(OUT_OF_RANGE);
    if (t->which_reading != (pb_size_t)(9 + t->sensor)) return R(OUT_OF_RANGE);
    switch (t->sensor) {
    case S(DHT22): {
        const aai16_Dht22Reading *r = &t->reading.dht22;
        REQUIRE(r->has_temperature_c && r->has_humidity_pct);
        return isfinite(r->temperature_c) && isfinite(r->humidity_pct) &&
               r->temperature_c >= -40.0f && r->temperature_c <= 80.0f &&
               r->humidity_pct >= 0.0f && r->humidity_pct <= 100.0f ? R(OK) : R(OUT_OF_RANGE);
    }
    case S(GY511): {
        const aai16_Gy511Reading *r = &t->reading.gy511;
        REQUIRE(r->has_acceleration_g && r->has_magnetic_field_ut && r->has_jitter_us);
        return vector_valid(&r->acceleration_g) && vector_valid(&r->magnetic_field_ut) ? R(OK) : R(OUT_OF_RANGE);
    }
    case S(RC522): {
        const aai16_Rc522Event *r = &t->reading.rc522;
        REQUIRE(r->has_uid);
        return r->uid.size == 4 || r->uid.size == 7 || r->uid.size == 10 ? R(OK) : R(OUT_OF_RANGE);
    }
    case S(LD2450): {
        const aai16_Ld2450Reading *r = &t->reading.ld2450;
        if (r->targets_count != 3) return R(OUT_OF_RANGE);
        unsigned slots = 0;
        for (unsigned i = 0; i < 3; ++i) {
            const aai16_RadarTarget *x = &r->targets[i];
            REQUIRE(x->has_slot && x->has_present && x->has_x_mm && x->has_y_mm &&
                    x->has_speed_cm_s && x->has_resolution_mm);
            if (x->slot < 1 || x->slot > 3 || (slots & (1U << x->slot)) ||
                x->x_mm < -32767 || x->x_mm > 32767 || x->y_mm < -32767 || x->y_mm > 32767 ||
                x->speed_cm_s < -32767 || x->speed_cm_s > 32767 || x->resolution_mm > 65535 ||
                (!x->present && (x->x_mm || x->y_mm || x->speed_cm_s || x->resolution_mm))) return R(OUT_OF_RANGE);
            slots |= 1U << x->slot;
        }
        return R(OK);
    }
    case S(HCSR04P): {
        const aai16_Hcsr04pReading *r = &t->reading.hcsr04p;
        REQUIRE(r->has_distance_cm && r->has_echo_us);
        return isfinite(r->distance_cm) && isfinite(r->echo_us) && r->distance_cm >= 2 &&
               r->distance_cm <= 400 && r->echo_us > 0 ? R(OK) : R(OUT_OF_RANGE);
    }
    case S(MHZ1911A): {
        const aai16_Mhz1911aReading *r = &t->reading.mhz1911a;
        REQUIRE(r->has_co2_ppm);
        return r->co2_ppm >= 400 && r->co2_ppm <= 5000 ? R(OK) : R(OUT_OF_RANGE);
    }
    case S(BME280): {
        const aai16_Bme280Reading *r = &t->reading.bme280;
        REQUIRE(r->has_temperature_c && r->has_pressure_hpa);
        return isfinite(r->temperature_c) && isfinite(r->pressure_hpa) &&
               (!r->has_humidity_pct || isfinite(r->humidity_pct)) ? R(OK) : R(OUT_OF_RANGE);
    }
    case S(MAX6675): {
        const aai16_Max6675Reading *r = &t->reading.max6675;
        REQUIRE(r->has_temperature_c);
        return isfinite(r->temperature_c) && r->temperature_c >= 0.0f && r->temperature_c <= 1023.75f ?
               R(OK) : R(OUT_OF_RANGE);
    }
    default: return R(UNSUPPORTED_SENSOR);
    }
}

static aai16_ResultCode prediction_validate(const aai16_MlPrediction *p)
{
    REQUIRE(p->has_model && p->has_model_epoch && p->has_status && p->has_model_ready &&
            p->has_classification && p->has_interval_ms && p->has_uptime_us);
    CHECK(identity_validate(&p->model));
    if (p->status < aai16_MotionStatus_MOTION_COLLECTING || p->status > aai16_MotionStatus_MOTION_SWITCHING ||
        !interval_valid(S(GY511), p->interval_ms) || p->uptime_us > INT64_MAX || p->logits_count > 2 ||
        (p->has_backend && (p->backend < aai16_Backend_BACKEND_TFLM_REFERENCE ||
                           p->backend > aai16_Backend_BACKEND_ESP_NN_OPTIMIZED)) ||
        (p->has_clipped && p->clipped > 60) ||
        (p->has_arena_used_bytes && p->has_arena_reserved_bytes && p->arena_used_bytes > p->arena_reserved_bytes) ||
        (p->has_inference_us && p->inference_us > p->uptime_us)) return R(OUT_OF_RANGE);
    for (unsigned i = 0; i < p->logits_count; ++i) if (!isfinite(p->logits[i])) return R(OUT_OF_RANGE);
    if (p->status == aai16_MotionStatus_MOTION_PREDICTION) {
        REQUIRE(p->has_first_sequence && p->has_last_sequence && p->has_window_start_us &&
                p->has_window_end_us && p->has_inference_us && p->has_output_int8);
        if (!p->model_ready || p->interval_ms != 100 || p->output_int8.size != 2 || p->logits_count != 2 ||
            p->window_start_us > p->window_end_us || p->window_end_us > p->uptime_us ||
            (uint32_t)(p->last_sequence - p->first_sequence) != 19 ||
            (p->classification != aai16_MotionClass_CLASS_REST &&
             p->classification != aai16_MotionClass_CLASS_MOVING)) return R(OUT_OF_RANGE);
    } else if (p->classification != aai16_MotionClass_CLASS_UNKNOWN || p->has_output_int8 ||
               p->logits_count || p->has_first_sequence || p->has_last_sequence ||
               p->has_window_start_us || p->has_window_end_us ||
               (p->has_inference_us && p->inference_us)) return R(OUT_OF_RANGE);
    return R(OK);
}
static aai16_ResultCode control_validate(const aai16_ModelControl *c)
{
    REQUIRE(c->has_operation);
    bool begin = c->operation == aai16_ModelOperation_MODEL_OP_BEGIN_UPLOAD;
    bool upload = begin || c->operation == aai16_ModelOperation_MODEL_OP_FINISH_UPLOAD ||
                  c->operation == aai16_ModelOperation_MODEL_OP_ABORT_UPLOAD;
    bool activate = c->operation == aai16_ModelOperation_MODEL_OP_ACTIVATE;
    bool change = activate || c->operation == aai16_ModelOperation_MODEL_OP_ROLLBACK;
    if (c->operation < aai16_ModelOperation_MODEL_OP_BEGIN_UPLOAD ||
        c->operation > aai16_ModelOperation_MODEL_OP_LIST ||
        c->has_upload != begin || c->has_transfer_id != upload ||
        c->has_target != activate || c->has_expected_model_epoch != change) return R(OUT_OF_RANGE);
    if (upload && c->transfer_id.size != 16) return R(OUT_OF_RANGE);
    if (activate) CHECK(identity_validate(&c->target));
    if (begin) {
        const aai16_UploadManifest *m = &c->upload;
        REQUIRE(m->has_family && m->has_version && m->has_package_size && m->has_package_sha256 &&
                m->has_model_size && m->has_model_sha256 && m->has_compatibility_profile);
        if ((m->family != aai16_ModelFamily_MODEL_A && m->family != aai16_ModelFamily_MODEL_B) ||
            !version_valid(m->version) || m->package_sha256.size != 32 || m->model_sha256.size != 32 ||
            !m->model_size || m->model_size > AAI16_MODEL_MAX || m->package_size < m->model_size ||
            m->package_size > AAI16_PACKAGE_MAX || m->compatibility_profile != 1) return R(OUT_OF_RANGE);
    }
    return R(OK);
}

aai16_ResultCode aai16_protocol_validate(const aai16_Envelope *e)
{
    REQUIRE(e && e->has_protocol_version);
    if (e->protocol_version != AAI16_PROTOCOL_VERSION) return R(UNSUPPORTED_VERSION);
    if ((e->has_phase && !phase_valid(e->phase)) || (e->has_request_id && !e->request_id)) return R(OUT_OF_RANGE);
    switch (e->which_body) {
    case aai16_Envelope_sensor_config_tag:
        REQUIRE(e->has_request_id && e->has_phase);
        if (e->phase != P(VALIDATE)) return R(OUT_OF_RANGE);
        return aai16_config_validate(&e->body.sensor_config);
    case aai16_Envelope_apply_config_tag:
        REQUIRE(e->has_request_id && e->has_boot_id && e->has_generation && e->has_phase &&
                e->body.apply_config.has_profile);
        if (e->phase != P(APPLY) && e->phase != P(ROLLBACK)) return R(OUT_OF_RANGE);
        return aai16_profile_validate(&e->body.apply_config.profile);
    case aai16_Envelope_config_ack_tag: {
        const aai16_ConfigAck *a = &e->body.config_ack;
        REQUIRE(e->has_request_id && e->has_boot_id && e->has_generation && e->has_phase && a->has_result);
        if (!result_valid(a->result) || (a->has_sensor && !sensor_valid(a->sensor)) ||
            (a->has_detail && !text_valid(a->detail, sizeof(a->detail)))) return R(OUT_OF_RANGE);
        if (a->result == R(OK)) REQUIRE(a->has_sensor);
        // Failed ACKs may echo an out-of-range attempted value. Never APPLY it.
        if (a->has_requested) {
            if (!a->has_sensor || !a->requested.has_sensor || a->requested.sensor != a->sensor) return R(OUT_OF_RANGE);
            if (a->result == R(OK)) CHECK(aai16_config_validate(&a->requested));
        }
        if (a->has_effective) {
            if (a->result != R(OK) || (e->phase != P(APPLY) && e->phase != P(ROLLBACK)) ||
                !a->has_sensor || a->effective.sensor != a->sensor) return R(OUT_OF_RANGE);
            CHECK(aai16_profile_validate(&a->effective));
        }
        return R(OK);
    }
    case aai16_Envelope_sensor_telemetry_tag:
        REQUIRE(e->has_boot_id && e->has_sequence);
        return telemetry_validate(&e->body.sensor_telemetry);
    case aai16_Envelope_ml_prediction_tag:
        REQUIRE(e->has_boot_id && e->has_generation);
        return prediction_validate(&e->body.ml_prediction);
    case aai16_Envelope_model_control_tag:
        REQUIRE(e->has_request_id);
        CHECK(control_validate(&e->body.model_control));
        if (e->body.model_control.operation <= aai16_ModelOperation_MODEL_OP_ROLLBACK)
            REQUIRE(e->has_boot_id && e->has_generation);
        return R(OK);
    case aai16_Envelope_model_selection_tag: {
        const aai16_ModelSelection *s = &e->body.model_selection;
        REQUIRE(e->has_request_id && s->has_operation);
        // Selection is a current-state intent, not a partially specified CAS.
        // Reject metadata that would otherwise look like checked preconditions.
        if (e->has_boot_id || e->has_generation || e->has_phase || e->has_sequence)
            return R(OUT_OF_RANGE);
        if (s->operation == aai16_ModelOperation_MODEL_OP_ACTIVATE) {
            REQUIRE(s->has_family);
            return s->family == aai16_ModelFamily_MODEL_A || s->family == aai16_ModelFamily_MODEL_B ? R(OK) : R(OUT_OF_RANGE);
        }
        return s->operation == aai16_ModelOperation_MODEL_OP_ROLLBACK && !s->has_family ? R(OK) : R(OUT_OF_RANGE);
    }
    case aai16_Envelope_model_upload_chunk_tag: {
        const aai16_ModelUploadChunk *c = &e->body.model_upload_chunk;
        REQUIRE(e->has_request_id && e->has_boot_id && e->has_generation && e->has_sequence &&
                c->has_transfer_id && c->has_offset && c->has_data);
        if (c->transfer_id.size != 16 || !c->data.size || c->data.size > AAI16_CHUNK_MAX ||
            c->offset > AAI16_PACKAGE_MAX || c->data.size > AAI16_PACKAGE_MAX - c->offset) return R(OUT_OF_RANGE);
        return R(OK); // Session length, ordering and generation are future manager checks.
    }
    case aai16_Envelope_model_upload_ack_tag: {
        const aai16_ModelUploadAck *a = &e->body.model_upload_ack;
        REQUIRE(e->has_request_id && e->has_boot_id && e->has_generation && e->has_sequence &&
                a->has_transfer_id && a->has_result && a->has_state && a->has_next_offset && a->has_max_chunk_bytes);
        if (a->transfer_id.size != 16 || !result_valid(a->result) || a->state < aai16_UploadState_UPLOAD_ACCEPTED ||
            a->state > aai16_UploadState_UPLOAD_REJECTED || a->next_offset > AAI16_PACKAGE_MAX ||
            !a->max_chunk_bytes || a->max_chunk_bytes > AAI16_CHUNK_MAX ||
            (a->has_detail && !text_valid(a->detail, sizeof(a->detail)))) return R(OUT_OF_RANGE);
        return R(OK);
    }
    case aai16_Envelope_model_status_tag: {
        const aai16_ModelStatus *s = &e->body.model_status;
        REQUIRE(e->has_boot_id && s->has_result && s->has_state && s->has_model_epoch);
        if (!result_valid(s->result) || s->state < aai16_ModelState_MODEL_STATE_RUNNING ||
            s->state > aai16_ModelState_MODEL_STATE_UNAVAILABLE || s->stored_count > 4 ||
            (s->has_backend && (s->backend < aai16_Backend_BACKEND_TFLM_REFERENCE ||
                               s->backend > aai16_Backend_BACKEND_ESP_NN_OPTIMIZED)) ||
            (s->has_detail && !text_valid(s->detail, sizeof(s->detail)))) return R(OUT_OF_RANGE);
        if (s->has_active) CHECK(identity_validate(&s->active));
        if (s->has_previous) CHECK(identity_validate(&s->previous));
        for (unsigned i = 0; i < s->stored_count; ++i) {
            CHECK(identity_validate(&s->stored[i]));
            if (s->stored[i].source != aai16_ModelSource_SOURCE_STORED) return R(OUT_OF_RANGE);
            for (unsigned j = 0; j < i; ++j)
                if (s->stored[j].family == s->stored[i].family && s->stored[j].bank == s->stored[i].bank)
                    return R(OUT_OF_RANGE);
        }
        return R(OK);
    }
    default: return R(NOT_SUPPORTED);
    }
}

aai16_ResultCode aai16_protocol_decode(const uint8_t *data, size_t length, aai16_Envelope *out)
{
    aai16_ResultCode result = decode(data, length, AAI16_FRAME_MAX, aai16_Envelope_fields, out, sizeof(*out));
    if (result == R(OK) && out->which_body == aai16_Envelope_sensor_config_tag && length > AAI16_CONFIG_FRAME_MAX)
        result = R(TOO_LARGE);
    if (result == R(OK)) result = aai16_protocol_validate(out);
    if (result != R(OK) && out) memset(out, 0, sizeof(*out));
    return result;
}
aai16_ResultCode aai16_config_decode(const uint8_t *data, size_t length,
                                    aai16_Envelope *out, uint64_t *request_id)
{
    if (request_id) *request_id = 0;
    aai16_ResultCode result = decode(data, length, AAI16_CONFIG_FRAME_MAX,
                                    aai16_Envelope_fields, out, sizeof(*out));
    if (result == R(OK)) {
        if (request_id && out->has_request_id) *request_id = out->request_id;
        result = aai16_protocol_validate(out);
        if (result == R(OK) && (out->which_body != aai16_Envelope_sensor_config_tag ||
                               out->has_generation || out->has_sequence)) result = R(NOT_SUPPORTED);
    }
    if (result != R(OK) && out) memset(out, 0, sizeof(*out));
    return result;
}
aai16_ResultCode aai16_protocol_encode(const aai16_Envelope *e, uint8_t *data, size_t capacity, size_t *length)
{
    if (length) *length = 0;
    CHECK(aai16_protocol_validate(e));
    return encode(aai16_Envelope_fields, e, data, capacity,
                  e->which_body == aai16_Envelope_sensor_config_tag ? AAI16_CONFIG_FRAME_MAX : AAI16_FRAME_MAX, length);
}
aai16_ResultCode aai16_snapshot_validate(const aai16_ConfigSnapshot *s)
{
    REQUIRE(s && s->has_schema_version && s->has_generation);
    if (s->schema_version != 1) return R(UNSUPPORTED_VERSION);
    if (s->sensors_count != 8) return R(OUT_OF_RANGE);
    unsigned seen = 0;
    for (unsigned i = 0; i < s->sensors_count; ++i) {
        CHECK(aai16_profile_validate(&s->sensors[i]));
        unsigned mask = 1U << s->sensors[i].sensor;
        if (seen & mask) return R(OUT_OF_RANGE);
        seen |= mask;
    }
    return R(OK);
}
aai16_ResultCode aai16_snapshot_decode(const uint8_t *data, size_t length, aai16_ConfigSnapshot *out)
{
    aai16_ResultCode result = decode(data, length, AAI16_SNAPSHOT_MAX, aai16_ConfigSnapshot_fields, out, sizeof(*out));
    if (result == R(OK)) result = aai16_snapshot_validate(out);
    if (result != R(OK) && out) memset(out, 0, sizeof(*out));
    return result;
}
aai16_ResultCode aai16_snapshot_encode(const aai16_ConfigSnapshot *s, uint8_t *data, size_t capacity, size_t *length)
{
    if (length) *length = 0;
    CHECK(aai16_snapshot_validate(s));
    return encode(aai16_ConfigSnapshot_fields, s, data, capacity, AAI16_SNAPSHOT_MAX, length);
}
static uint32_t crc_byte(uint32_t crc, uint8_t value)
{
    crc ^= value;
    for (unsigned i = 0; i < 8; ++i) crc = (crc >> 1) ^ (UINT32_C(0xEDB88320) & (0U - (crc & 1U)));
    return crc;
}
static uint32_t record_crc(const aai16_PersistentConfigRecord *r)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < 4; ++i) crc = crc_byte(crc, (uint8_t)(r->format_version >> (8 * i)));
    for (unsigned i = 0; i < 8; ++i) crc = crc_byte(crc, (uint8_t)(r->record_sequence >> (8 * i)));
    for (unsigned i = 0; i < 4; ++i) crc = crc_byte(crc, (uint8_t)((uint32_t)r->snapshot.size >> (8 * i)));
    for (unsigned i = 0; i < r->snapshot.size; ++i) crc = crc_byte(crc, r->snapshot.bytes[i]);
    return crc ^ UINT32_MAX;
}
static aai16_ResultCode record_validate(const aai16_PersistentConfigRecord *r, aai16_ConfigSnapshot *s)
{
    REQUIRE(r && r->has_format_version && r->has_record_sequence && r->has_snapshot && r->has_crc32 && r->has_commit_marker);
    if (r->format_version != 1) return R(UNSUPPORTED_VERSION);
    if (!r->record_sequence || !r->snapshot.size || r->snapshot.size > AAI16_SNAPSHOT_MAX ||
        r->commit_marker != AAI16_COMMIT_MARKER) return R(OUT_OF_RANGE);
    if (record_crc(r) != r->crc32) return R(CHECKSUM_MISMATCH);
    return aai16_snapshot_decode(r->snapshot.bytes, r->snapshot.size, s);
}
aai16_ResultCode aai16_record_prepare(const aai16_ConfigSnapshot *s, uint64_t sequence, aai16_PersistentConfigRecord *r)
{
    REQUIRE(r);
    memset(r, 0, sizeof(*r));
    if (!sequence) return R(OUT_OF_RANGE);
    size_t length;
    aai16_ResultCode result = aai16_snapshot_encode(s, r->snapshot.bytes, sizeof(r->snapshot.bytes), &length);
    if (result != R(OK)) { memset(r, 0, sizeof(*r)); return result; }
    r->has_format_version = r->has_record_sequence = r->has_snapshot = r->has_crc32 = r->has_commit_marker = true;
    r->format_version = 1;
    r->record_sequence = sequence;
    r->snapshot.size = (pb_size_t)length;
    r->commit_marker = AAI16_COMMIT_MARKER;
    r->crc32 = record_crc(r);
    return R(OK);
}
aai16_ResultCode aai16_record_encode(const aai16_PersistentConfigRecord *r, uint8_t *data, size_t capacity, size_t *length)
{
    if (length) *length = 0;
    aai16_ConfigSnapshot scratch;
    CHECK(record_validate(r, &scratch));
    return encode(aai16_PersistentConfigRecord_fields, r, data, capacity, AAI16_RECORD_MAX, length);
}
aai16_ResultCode aai16_record_decode(const uint8_t *data, size_t length,
                                   aai16_PersistentConfigRecord *r, aai16_ConfigSnapshot *s)
{
    if (s) memset(s, 0, sizeof(*s));
    if (r) memset(r, 0, sizeof(*r));
    REQUIRE(r && s);
    aai16_ResultCode result = decode(data, length, AAI16_RECORD_MAX, aai16_PersistentConfigRecord_fields, r, sizeof(*r));
    if (result == R(OK)) result = record_validate(r, s);
    if (result != R(OK)) { memset(r, 0, sizeof(*r)); memset(s, 0, sizeof(*s)); }
    return result;
}
