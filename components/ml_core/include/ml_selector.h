#pragma once
/* ML input selector: maps sensor values onto a model's input channels, rejects incompatible
 * mappings BEFORE inference (handoff 5.4), assembles time-ordered windows and produces features.
 *
 * Policy when a mapping becomes invalid later (e.g. the I2C sensor is swapped GY-511 -> BME280 while
 * the motion model is active): the model and the last mapping are kept, inference is SUSPENDED with a
 * reason, the window is discarded, and no stale value is ever fed to the model.
 * Pure C; all sensor knowledge arrives through the lookup callback.
 */
#include <stdbool.h>
#include <stdint.h>

#include "aai16.pb.h"
#include "ml_features.h"

#define ML_MAX_CHANNELS 10
#define ML_MAX_WINDOW   64

typedef struct {
    bool active;              /* part of the running configuration */
    uint32_t period_ms;
    uint32_t output_count;
    aai16_ValueKind kinds[8]; /* meaning of each output */
} ml_source_t;

/* Look up a sensor in the committed configuration. Return false if it is not active. */
typedef bool (*ml_lookup_fn)(uint32_t sensor_id, ml_source_t *out, void *ctx);

typedef struct {
    aai16_ModelContract contract;
    bool has_contract;
    bool mapped;
    struct {
        uint32_t sensor_id;
        uint32_t value_index;
    } map[ML_MAX_CHANNELS];

    aai16_ModelReason state; /* last result: NONE = running; otherwise why it is suspended or restarting */
    bool suspended;          /* mapping currently invalid (sticky until it is valid again) */

    float latest[ML_MAX_CHANNELS];
    uint64_t latest_ts[ML_MAX_CHANNELS];
    bool have[ML_MAX_CHANNELS];
    float rows[ML_MAX_WINDOW][ML_MAX_CHANNELS];
    uint32_t count;          /* rows currently in the window (<= window) */
    uint64_t last_row_ts;
    uint32_t since_infer;
    bool fresh;              /* first full window after a (re)start infers immediately */

    /* MODEL_ANOMALY only (ml/ANOMALY_SPEC.md): partial mappings are allowed, slots without fresh data are absent */
    bool map_set[ML_MAX_CHANNELS]; /* channel has a mapped source */
    ml_slot_t slot[ML_SLOT_COUNT];
    uint64_t slot_last_ts[ML_SLOT_COUNT];
    uint64_t last_infer_ts;
    uint32_t active_mask;    /* slots present at the last feature vector (bit s = slot s) */
} ml_selector_t;

void mlsel_init(ml_selector_t *s);

/* A model became active. Keeps the existing mapping only if the channel kinds are unchanged. */
void mlsel_set_contract(ml_selector_t *s, const aai16_ModelContract *c);

/* ModelControl(MODEL_SET_INPUTS): validate everything, then commit; on any failure nothing changes. */
aai16_ModelReason mlsel_set_inputs(ml_selector_t *s, const aai16_InputSource *in, uint32_t n,
                                   ml_lookup_fn lookup, void *ctx);

/* Re-check the current mapping against the current configuration (call after every ConfigApply). */
aai16_ModelReason mlsel_revalidate(ml_selector_t *s, ml_lookup_fn lookup, void *ctx);

/* Features of the current window (for a rule-requested inference). False if the window is not full
 * (anomaly models: false if no slot has fresh data; otherwise updates active_mask). */
bool mlsel_current_features(ml_selector_t *s, float features[ML_MAX_FEATURES]);

/* True if channel `ch` has a mapped source (always true for a classification model once mapped). */
bool mlsel_channel_mapped(const ml_selector_t *s, uint32_t ch);

/* Feed one reading. Returns the reason (NONE when healthy). *ready is set when a new feature vector
 * is available in `features` (capacity ML_MAX_FEATURES). */
aai16_ModelReason mlsel_push(ml_selector_t *s, uint32_t sensor_id, uint64_t ts_us, bool valid,
                             const float *values, uint32_t n_values, ml_lookup_fn lookup, void *ctx,
                             bool *ready, float features[ML_MAX_FEATURES]);
