/* ML input selection: maps sensor values to model channels, checks value kind and sample period, builds windows (motion,
 * environment) or slot statistics (anomaly, partial mappings allowed) from live readings, and says when an inference is due. */
#include "ml_selector.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef aai16_ModelReason reason_t;
#define R(x) aai16_ModelReason_MODEL_REASON_##x

static void reset_window(ml_selector_t *s)
{
    s->count = 0;
    s->since_infer = 0;
    s->fresh = true;
    s->last_row_ts = 0;
    memset(s->have, 0, sizeof(s->have));
    memset(s->slot, 0, sizeof(s->slot)); /* anomaly slots: forget older readings too */
    memset(s->slot_last_ts, 0, sizeof(s->slot_last_ts));
    s->last_infer_ts = 0;
    s->active_mask = 0;
}

void mlsel_init(ml_selector_t *s)
{
    memset(s, 0, sizeof(*s));
    s->state = R(NO_ACTIVE_MODEL);
    s->fresh = true;
}

void mlsel_set_contract(ml_selector_t *s, const aai16_ModelContract *c)
{
    bool same = s->has_contract && s->contract.channels_count == c->channels_count;
    for (uint32_t i = 0; same && i < c->channels_count; i++) {
        same = s->contract.channels[i] == c->channels[i];
    }
    s->contract = *c;
    s->has_contract = true;
    if (!same) {
        s->mapped = false; /* a different input meaning needs an explicit new mapping */
        memset(s->map_set, 0, sizeof(s->map_set));
    }
    reset_window(s);
    s->state = s->mapped ? R(NONE) : R(NO_INPUTS);
}

static reason_t check_source(const aai16_ModelContract *c, uint32_t slot, uint32_t sensor_id,
                             uint32_t value_index, ml_lookup_fn lookup, void *ctx)
{
    ml_source_t src;
    if (!lookup(sensor_id, &src, ctx) || !src.active) {
        return R(SENSOR_NOT_ACTIVE);
    }
    if (value_index >= src.output_count || value_index >= 8) {
        return R(VALUE_INDEX_RANGE);
    }
    if (src.kinds[value_index] != c->channels[slot]) {
        return R(VALUE_KIND_MISMATCH);
    }
    if (slot == 0) { /* this channel paces the window */
        const uint32_t expect = c->sample_period_ms;
        const uint32_t diff = src.period_ms > expect ? src.period_ms - expect : expect - src.period_ms;
        if ((uint64_t)diff * 100u > (uint64_t)expect * c->period_tolerance_pct) {
            return R(PERIOD_MISMATCH);
        }
    } else if (src.period_ms > c->max_age_ms) { /* sampled "latest value": must be fresh enough */
        return R(PERIOD_MISMATCH);
    }
    return R(NONE);
}

/* ------------------------------------------------------------------ MODEL_ANOMALY (ml/ANOMALY_SPEC.md) */
static bool is_anomaly(const ml_selector_t *s) { return s->has_contract && s->contract.kind == aai16_ModelKind_MODEL_ANOMALY; }

static reason_t check_source_anom(const aai16_ModelContract *c, uint32_t channel, uint32_t sensor_id,
                                  uint32_t value_index, ml_lookup_fn lookup, void *ctx)
{
    ml_source_t src;
    if (!lookup(sensor_id, &src, ctx) || !src.active) {
        return R(SENSOR_NOT_ACTIVE);
    }
    if (value_index >= src.output_count || value_index >= 8) {
        return R(VALUE_INDEX_RANGE);
    }
    if (src.kinds[value_index] != c->channels[channel]) {
        return R(VALUE_KIND_MISMATCH);
    }
    if (src.period_ms > c->max_age_ms) { /* every slot is a "latest value": it must refresh within max_age */
        return R(PERIOD_MISMATCH);
    }
    return R(NONE);
}

/* Features of the slots that currently have fresh data; absent slots stay zero. Returns the slot mask. */
static uint32_t anomaly_features(const ml_selector_t *s, uint64_t now_us, float *out)
{
    memset(out, 0, ML_ANOMALY_FEATURES * sizeof(float));
    const uint64_t max_age_us = (uint64_t)s->contract.max_age_ms * 1000u;
    uint32_t mask = 0;
    for (uint32_t slot = 0; slot < ML_SLOT_COUNT; slot++) {
        if (s->slot[slot].n >= ML_SLOT_MIN_READINGS && now_us >= s->slot_last_ts[slot] &&
            now_us - s->slot_last_ts[slot] <= max_age_us) {
            ml_slot_features(&s->slot[slot], slot, out + slot * ML_SLOT_FEATURES);
            mask |= 1u << slot;
        }
    }
    return mask;
}

static reason_t set_inputs_anomaly(ml_selector_t *s, const aai16_InputSource *in, uint32_t n, ml_lookup_fn lookup,
                                   void *ctx)
{
    const uint32_t channels = s->contract.channels_count;
    if (n < 1 || n > channels || channels > ML_MAX_CHANNELS) {
        return R(INPUT_COUNT_MISMATCH);
    }
    uint32_t seen = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (in[i].feature_slot >= channels) {
            return R(BAD_SLOT);
        }
        if (seen & (1u << in[i].feature_slot)) {
            return R(DUPLICATE_SLOT);
        }
        seen |= 1u << in[i].feature_slot;
    }
    if ((seen & 7u) != 0 && (seen & 7u) != 7u) { /* the motion slot needs all three acceleration axes */
        return R(INPUT_COUNT_MISMATCH);
    }
    for (uint32_t i = 0; i < n; i++) {
        const reason_t r = check_source_anom(&s->contract, in[i].feature_slot, in[i].sensor_id, in[i].value_index,
                                             lookup, ctx);
        if (r != R(NONE)) {
            return r; /* nothing committed: the previous mapping stays in force */
        }
    }
    memset(s->map_set, 0, sizeof(s->map_set));
    for (uint32_t i = 0; i < n; i++) {
        s->map[in[i].feature_slot].sensor_id = in[i].sensor_id;
        s->map[in[i].feature_slot].value_index = in[i].value_index;
        s->map_set[in[i].feature_slot] = true;
    }
    s->mapped = true;
    s->suspended = false;
    reset_window(s);
    s->state = R(NONE);
    return R(NONE);
}

reason_t mlsel_set_inputs(ml_selector_t *s, const aai16_InputSource *in, uint32_t n, ml_lookup_fn lookup,
                          void *ctx)
{
    if (!s->has_contract) {
        return R(NO_ACTIVE_MODEL);
    }
    if (is_anomaly(s)) {
        return set_inputs_anomaly(s, in, n, lookup, ctx);
    }
    const uint32_t channels = s->contract.channels_count;
    if (n != channels || channels > ML_MAX_CHANNELS) {
        return R(INPUT_COUNT_MISMATCH);
    }
    uint32_t seen = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (in[i].feature_slot >= channels) {
            return R(BAD_SLOT);
        }
        if (seen & (1u << in[i].feature_slot)) {
            return R(DUPLICATE_SLOT);
        }
        seen |= 1u << in[i].feature_slot;
    }
    for (uint32_t i = 0; i < n; i++) {
        const reason_t r = check_source(&s->contract, in[i].feature_slot, in[i].sensor_id, in[i].value_index,
                                        lookup, ctx);
        if (r != R(NONE)) {
            return r; /* nothing committed: the previous mapping stays in force */
        }
    }
    for (uint32_t i = 0; i < n; i++) {
        s->map[in[i].feature_slot].sensor_id = in[i].sensor_id;
        s->map[in[i].feature_slot].value_index = in[i].value_index;
    }
    s->mapped = true;
    s->suspended = false;
    reset_window(s);
    s->state = R(NONE);
    return R(NONE);
}

static reason_t check_map(const ml_selector_t *s, ml_lookup_fn lookup, void *ctx)
{
    if (!s->has_contract) {
        return R(NO_ACTIVE_MODEL);
    }
    if (!s->mapped) {
        return R(NO_INPUTS);
    }
    if (is_anomaly(s)) { /* healthy while at least one mapped sensor is running; the others just drop out of the mask */
        reason_t first = R(NONE);
        bool any = false;
        for (uint32_t i = 0; i < s->contract.channels_count; i++) {
            if (!s->map_set[i]) {
                continue;
            }
            const reason_t r = check_source_anom(&s->contract, i, s->map[i].sensor_id, s->map[i].value_index, lookup, ctx);
            if (r == R(NONE)) {
                any = true;
            } else if (first == R(NONE)) {
                first = r;
            }
        }
        return any ? R(NONE) : (first == R(NONE) ? R(NO_INPUTS) : first);
    }
    for (uint32_t i = 0; i < s->contract.channels_count; i++) {
        const reason_t r = check_source(&s->contract, i, s->map[i].sensor_id, s->map[i].value_index, lookup, ctx);
        if (r != R(NONE)) {
            return r;
        }
    }
    return R(NONE);
}

reason_t mlsel_revalidate(ml_selector_t *s, ml_lookup_fn lookup, void *ctx)
{
    const reason_t r = check_map(s, lookup, ctx);
    if (is_anomaly(s) && r == R(NONE)) { /* a sensor that vanished: forget its slot so it re-warms when it returns */
        for (uint32_t c = 0; c < s->contract.channels_count; c++) {
            if (s->map_set[c] && check_source_anom(&s->contract, c, s->map[c].sensor_id, s->map[c].value_index, lookup, ctx) != R(NONE)) {
                const uint32_t slot = ml_anomaly_slot_of_channel(c);
                ml_slot_reset(&s->slot[slot]);
                s->slot_last_ts[slot] = 0;
                s->have[c] = false;
            }
        }
    }
    if (r != R(NONE) || s->suspended) {
        reset_window(s); /* suspend, or resume with an empty window: never reuse older samples */
    }
    s->suspended = (r != R(NONE));
    s->state = r;
    return r;
}

bool mlsel_channel_mapped(const ml_selector_t *s, uint32_t ch)
{
    if (!s->mapped || ch >= s->contract.channels_count) {
        return false;
    }
    return is_anomaly(s) ? s->map_set[ch] : true;
}

bool mlsel_current_features(ml_selector_t *s, float features[ML_MAX_FEATURES])
{
    if (is_anomaly(s)) {
        if (!s->mapped || s->suspended) {
            return false;
        }
        uint64_t now = 0;
        for (uint32_t i = 0; i < ML_SLOT_COUNT; i++) {
            now = s->slot_last_ts[i] > now ? s->slot_last_ts[i] : now;
        }
        s->active_mask = anomaly_features(s, now, features);
        return s->active_mask != 0;
    }
    if (!s->has_contract || !s->mapped || s->suspended || s->count != s->contract.window_samples) {
        return false;
    }
    const uint32_t window = s->contract.window_samples, channels = s->contract.channels_count;
    float flat[ML_MAX_WINDOW * ML_MAX_CHANNELS];
    for (uint32_t i = 0; i < window; i++) {
        for (uint32_t c = 0; c < channels; c++) {
            flat[i * channels + c] = s->rows[i][c];
        }
    }
    return ml_compute_features(s->contract.preprocessing, flat, window, channels, features, ML_MAX_FEATURES);
}

static reason_t push_anomaly(ml_selector_t *s, uint32_t sensor_id, uint64_t ts_us, bool valid, const float *values,
                              uint32_t n_values, ml_lookup_fn lookup, void *ctx, bool *ready,
                              float features[ML_MAX_FEATURES])
{
    const reason_t m = check_map(s, lookup, ctx);
    if (m != R(NONE)) {
        reset_window(s);
        s->suspended = true;
        s->state = m;
        return m;
    }
    if (s->suspended) { /* the mapping is valid again: start with empty slots */
        reset_window(s);
        s->suspended = false;
    }
    reason_t state = R(NONE);
    uint32_t touched = 0;
    for (uint32_t c = 0; c < s->contract.channels_count; c++) {
        if (!s->map_set[c] || s->map[c].sensor_id != sensor_id) {
            continue;
        }
        const uint32_t slot = ml_anomaly_slot_of_channel(c);
        if (!valid || s->map[c].value_index >= n_values || !isfinite(values[s->map[c].value_index])) {
            s->have[c] = false; /* an invalid reading removes only this slot; it re-warms with fresh readings */
            ml_slot_reset(&s->slot[slot]);
            s->slot_last_ts[slot] = 0;
            state = R(READING_INVALID);
            continue;
        }
        s->latest[c] = values[s->map[c].value_index];
        s->latest_ts[c] = ts_us;
        s->have[c] = true;
        touched |= 1u << slot;
    }
    for (uint32_t slot = 0; slot < ML_SLOT_COUNT; slot++) {
        if (!(touched & (1u << slot))) {
            continue;
        }
        double x;
        if (slot == 0) { /* motion: magnitude of the three axes */
            if (!(s->have[0] && s->have[1] && s->have[2])) {
                continue;
            }
            x = sqrt((double)s->latest[0] * s->latest[0] + (double)s->latest[1] * s->latest[1] +
                     (double)s->latest[2] * s->latest[2]);
        } else {
            x = s->latest[slot + 2]; /* channels 3..9 are slots 1..7 */
        }
        ml_slot_update(&s->slot[slot], x, (double)ts_us / 1e6);
        s->slot_last_ts[slot] = ts_us;
    }
    const uint64_t interval = (uint64_t)s->contract.sample_period_ms * 1000u;
    if (touched && (s->last_infer_ts == 0 || (ts_us >= s->last_infer_ts && ts_us - s->last_infer_ts >= interval))) {
        const uint32_t mask = anomaly_features(s, ts_us, features);
        if (mask) {
            s->active_mask = mask;
            s->last_infer_ts = ts_us;
            *ready = true;
        }
    }
    s->state = state;
    return state;
}

reason_t mlsel_push(ml_selector_t *s, uint32_t sensor_id, uint64_t ts_us, bool valid, const float *values,
                    uint32_t n_values, ml_lookup_fn lookup, void *ctx, bool *ready,
                    float features[ML_MAX_FEATURES])
{
    *ready = false;
    if (is_anomaly(s)) {
        return push_anomaly(s, sensor_id, ts_us, valid, values, n_values, lookup, ctx, ready, features);
    }
    const reason_t m = check_map(s, lookup, ctx);
    if (m != R(NONE)) {
        reset_window(s);
        s->suspended = true;
        s->state = m;
        return m;
    }
    if (s->suspended) { /* mapping is valid again after a suspension: start from an empty window */
        reset_window(s);
        s->suspended = false;
    }
    reason_t transient = R(NONE);

    const uint32_t channels = s->contract.channels_count;
    bool relevant = false;
    for (uint32_t c = 0; c < channels; c++) {
        if (s->map[c].sensor_id != sensor_id) {
            continue;
        }
        relevant = true;
        if (!valid || s->map[c].value_index >= n_values) {
            reset_window(s);
            s->state = R(READING_INVALID);
            return s->state;
        }
        s->latest[c] = values[s->map[c].value_index];
        s->latest_ts[c] = ts_us;
        s->have[c] = true;
    }
    if (!relevant || s->map[0].sensor_id != sensor_id) {
        return R(NONE); /* either unrelated, or only refreshed a "latest value" channel */
    }

    const uint64_t max_age_us = (uint64_t)s->contract.max_age_ms * 1000u;
    for (uint32_t c = 0; c < channels; c++) {
        const bool fresh_enough = s->have[c] && (ts_us <= s->latest_ts[c] || ts_us - s->latest_ts[c] <= max_age_us);
        if (!fresh_enough) {
            reset_window(s);
            s->state = R(INPUTS_STALE);
            return s->state;
        }
    }
    if (s->count > 0) {
        const uint64_t expect = (uint64_t)s->contract.sample_period_ms * 1000u;
        const uint64_t dt = ts_us > s->last_row_ts ? ts_us - s->last_row_ts : 0;
        const uint64_t err = dt > expect ? dt - expect : expect - dt;
        if (err * 100u > expect * s->contract.period_tolerance_pct) {
            reset_window(s); /* gap or jitter beyond tolerance: restart the window with this sample */
            transient = R(PERIOD_MISMATCH);
            for (uint32_t c = 0; c < channels; c++) {
                s->have[c] = (s->map[c].sensor_id == sensor_id);
            }
        }
    }

    const uint32_t window = s->contract.window_samples;
    if (s->count == window) {
        memmove(&s->rows[0], &s->rows[1], sizeof(s->rows[0]) * (window - 1));
        s->count--;
    }
    for (uint32_t c = 0; c < channels; c++) {
        s->rows[s->count][c] = s->latest[c];
    }
    s->count++;
    s->last_row_ts = ts_us;
    s->since_infer++;

    const uint32_t stride = s->contract.stride_samples ? s->contract.stride_samples : window;
    if (s->count == window && (s->fresh || s->since_infer >= stride)) {
        float flat[ML_MAX_WINDOW * ML_MAX_CHANNELS];
        for (uint32_t i = 0; i < window; i++) {
            for (uint32_t c = 0; c < channels; c++) {
                flat[i * channels + c] = s->rows[i][c];
            }
        }
        if (!ml_compute_features(s->contract.preprocessing, flat, window, channels, features, ML_MAX_FEATURES)) {
            return R(BAD_PACKAGE);
        }
        s->since_infer = 0;
        s->fresh = false;
        *ready = true;
    }
    s->state = transient; /* PERIOD_MISMATCH here means "window restarted by this sample" */
    return transient;
}
