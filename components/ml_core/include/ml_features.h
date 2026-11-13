#pragma once
/* Reference-matched preprocessing and int8 glue for the two models (pure C, host-testable).
 * The algorithms are specified in ml/common.py; golden vectors pin them together. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aai16.pb.h"

#define ML_MOTION_WINDOW   64
#define ML_MOTION_CHANNELS 3
#define ML_MOTION_FEATURES 11
#define ML_ENV_WINDOW      20
#define ML_ENV_CHANNELS    2
#define ML_ENV_FEATURES    6
#define ML_MAX_FEATURES    32

/* Model C (anomaly): 10 value channels in 8 role slots, 4 features per slot (ml/ANOMALY_SPEC.md). */
#define ML_SLOT_COUNT        8
#define ML_SLOT_FEATURES     4
#define ML_ANOMALY_CHANNELS  10
#define ML_ANOMALY_FEATURES  32
#define ML_SLOT_MIN_READINGS 5

typedef struct {
    double fast, mid, slow, jit; /* exponential statistics (tau 3 s, 15 s, 90 s, 8 s) */
    double t_prev;               /* seconds */
    uint32_t n;                  /* readings seen */
} ml_slot_t;

/* Slot that a channel belongs to (channels 0..2 are the three acceleration axes of slot 0). */
uint32_t ml_anomaly_slot_of_channel(uint32_t channel);
void ml_slot_reset(ml_slot_t *s);
/* Feed one scalar slot value taken at t_s seconds (rate independent). */
void ml_slot_update(ml_slot_t *s, double x, double t_s);
/* The 4 features of slot `slot` (writes ML_SLOT_FEATURES floats). */
void ml_slot_features(const ml_slot_t *s, uint32_t slot, float *out);

/* Score of an anomaly inference from the int8 tensors only: mean squared difference (dequantized) over the
 * features of the present slots (slot_mask bit s = slot s present). Returns false on bad arguments. */
bool ml_anomaly_score(const aai16_ModelContract *c, const int8_t *in, const int8_t *out, uint32_t slot_mask,
                      float *score, float slot_err[ML_SLOT_COUNT]);

/* Window shape / feature count a preprocessing kind requires. Returns false for unknown kinds. */
bool ml_preproc_shape(aai16_Preprocessing p, uint32_t *window, uint32_t *channels, uint32_t *features);

/* rows: window x channels, row-major (time-ordered, oldest first). Writes `features` floats. */
bool ml_compute_features(aai16_Preprocessing p, const float *rows, uint32_t window, uint32_t channels,
                         float *out, uint32_t out_cap);

/* float -> int8 with the model's input quantization (round half to even, saturating). */
int8_t ml_quantize(float v, float scale, int32_t zero_point);
/* int8 outputs -> dequantized scores; returns the argmax index (first maximum). */
uint32_t ml_decode_output(const int8_t *q, uint32_t n, float scale, int32_t zero_point, float *scores);
