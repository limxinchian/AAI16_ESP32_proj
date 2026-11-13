/* ML feature extraction: motion and environment window statistics, Model C slot statistics (rate-independent
 * exponential averages per sensor slot), int8 quantisation, and the anomaly score computed from the int8 tensors. */
#include "ml_features.h"

#include <math.h>

#define ZCR_DEADBAND_G 0.002

bool ml_preproc_shape(aai16_Preprocessing p, uint32_t *window, uint32_t *channels, uint32_t *features)
{
    switch (p) {
    case aai16_Preprocessing_PREPROC_MOTION_STATS_V1:
        *window = ML_MOTION_WINDOW; *channels = ML_MOTION_CHANNELS; *features = ML_MOTION_FEATURES;
        return true;
    case aai16_Preprocessing_PREPROC_ENV_TREND_V1:
        *window = ML_ENV_WINDOW; *channels = ML_ENV_CHANNELS; *features = ML_ENV_FEATURES;
        return true;
    case aai16_Preprocessing_PREPROC_SLOT_STATS_V1:
        *window = 1; *channels = ML_ANOMALY_CHANNELS; *features = ML_ANOMALY_FEATURES;
        return true;
    default:
        return false;
    }
}

static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* PREPROC_MOTION_STATS_V1: see ml/common.py::motion_features */
static void motion(const float *rows, float *out)
{
    const int n = ML_MOTION_WINDOW;
    double std[3], mad[3], zcr[3];
    for (int a = 0; a < 3; a++) {
        double mean = 0;
        for (int i = 0; i < n; i++) {
            mean += rows[i * 3 + a];
        }
        mean /= n;
        double ss = 0, ad = 0;
        int crossings = 0, prev = 0;
        for (int i = 0; i < n; i++) {
            const double c = rows[i * 3 + a] - mean;
            ss += c * c;
            if (i > 0) {
                ad += fabs((double)rows[i * 3 + a] - (double)rows[(i - 1) * 3 + a]);
            }
            const int s = c > ZCR_DEADBAND_G ? 1 : (c < -ZCR_DEADBAND_G ? -1 : 0);
            if (s * prev == -1) {
                crossings++;
            }
            prev = s;
        }
        std[a] = sqrt(ss / n);
        mad[a] = ad / (n - 1);
        zcr[a] = (double)crossings / (n - 1);
    }
    for (int a = 0; a < 3; a++) {
        out[a * 3 + 0] = (float)clampd(std[a] / 0.5, 0, 4);
        out[a * 3 + 1] = (float)clampd(mad[a] / 0.25, 0, 4);
        out[a * 3 + 2] = (float)clampd(zcr[a], 0, 4);
    }
    const double tot = sqrt(std[0] * std[0] + std[1] * std[1] + std[2] * std[2]);
    out[9] = (float)clampd(tot / 0.5, 0, 4);
    out[10] = (float)clampd((mad[0] + mad[1] + mad[2]) / (std[0] + std[1] + std[2] + 1e-3) / 4.0, 0, 4);
}

/* PREPROC_ENV_TREND_V1: see ml/common.py::env_features */
static void environment(const float *rows, float *out)
{
    static const double scale[2][3] = {{2.0, 0.3, 2.0}, {8.0, 1.5, 8.0}};
    const int n = ML_ENV_WINDOW;
    for (int ch = 0; ch < 2; ch++) {
        double mean = 0, lo = rows[ch], hi = rows[ch];
        for (int i = 0; i < n; i++) {
            const double y = rows[i * 2 + ch];
            mean += y;
            lo = y < lo ? y : lo;
            hi = y > hi ? y : hi;
        }
        mean /= n;
        double sty = 0, stt = 0;
        for (int i = 0; i < n; i++) {
            const double t = i - (n - 1) / 2.0;
            sty += t * rows[i * 2 + ch];
            stt += t * t;
        }
        const double slope = sty / stt;
        double ss = 0;
        for (int i = 0; i < n; i++) {
            const double t = i - (n - 1) / 2.0;
            const double r = rows[i * 2 + ch] - mean - slope * t;
            ss += r * r;
        }
        out[ch * 3 + 0] = (float)clampd(slope * (n - 1) / scale[ch][0], -4, 4);
        out[ch * 3 + 1] = (float)clampd(sqrt(ss / n) / scale[ch][1], 0, 4);
        out[ch * 3 + 2] = (float)clampd((hi - lo) / scale[ch][2], 0, 4);
    }
}

bool ml_compute_features(aai16_Preprocessing p, const float *rows, uint32_t window, uint32_t channels,
                         float *out, uint32_t out_cap)
{
    uint32_t w, c, f;
    if (p == aai16_Preprocessing_PREPROC_SLOT_STATS_V1) {
        return false; /* slot statistics are built by the selector from live readings, not from a window */
    }
    if (!ml_preproc_shape(p, &w, &c, &f) || w != window || c != channels || out_cap < f) {
        return false;
    }
    for (uint32_t i = 0; i < window * channels; i++) {
        if (!isfinite(rows[i])) {
            return false;
        }
    }
    if (p == aai16_Preprocessing_PREPROC_MOTION_STATS_V1) {
        motion(rows, out);
    } else {
        environment(rows, out);
    }
    return true;
}

/* ------------------------------------------------------------------ Model C: slot statistics */
static const uint8_t SLOT_OF_CHANNEL[ML_ANOMALY_CHANNELS] = {0, 0, 0, 1, 2, 3, 4, 5, 6, 7};
/* center, s0 (level), s1 (fast - slow), s2 (jitter), s3 (fast - mid): must equal ml/anomaly.py::CONST */
static const double SLOT_CONST[ML_SLOT_COUNT][5] = {
    {1.0, 0.5, 0.5, 0.2, 0.5},           {25.0, 15.0, 2.0, 0.5, 1.0},       {60.0, 40.0, 10.0, 2.0, 4.0},
    {101300.0, 3000.0, 100.0, 30.0, 60.0}, {700.0, 1000.0, 300.0, 60.0, 150.0}, {100.0, 100.0, 20.0, 5.0, 20.0},
    {1500.0, 1500.0, 400.0, 150.0, 400.0}, {40.0, 40.0, 3.0, 0.5, 1.5},
};
#define TAU_FAST 3.0
#define TAU_MID  15.0
#define TAU_SLOW 90.0
#define TAU_JIT  8.0

uint32_t ml_anomaly_slot_of_channel(uint32_t channel)
{
    return channel < ML_ANOMALY_CHANNELS ? SLOT_OF_CHANNEL[channel] : 0;
}

void ml_slot_reset(ml_slot_t *s)
{
    s->fast = s->mid = s->slow = s->jit = s->t_prev = 0.0;
    s->n = 0;
}

void ml_slot_update(ml_slot_t *s, double x, double t_s)
{
    if (s->n == 0) {
        s->fast = s->mid = s->slow = x;
        s->jit = 0.0;
    } else {
        const double dt = t_s - s->t_prev;
        if (dt <= 0.0) {
            return;
        }
        s->fast += (1.0 - exp(-dt / TAU_FAST)) * (x - s->fast);
        s->mid += (1.0 - exp(-dt / TAU_MID)) * (x - s->mid);
        s->slow += (1.0 - exp(-dt / TAU_SLOW)) * (x - s->slow);
        s->jit += (1.0 - exp(-dt / TAU_JIT)) * (fabs(x - s->fast) - s->jit);
    }
    s->n++;
    s->t_prev = t_s;
}

void ml_slot_features(const ml_slot_t *s, uint32_t slot, float *out)
{
    if (slot >= ML_SLOT_COUNT) {
        out[0] = out[1] = out[2] = out[3] = 0.0f;
        return;
    }
    const double *k = SLOT_CONST[slot];
    out[0] = (float)clampd((s->fast - k[0]) / k[1], -4, 4);
    out[1] = (float)clampd((s->fast - s->slow) / k[2], -4, 4);
    out[2] = (float)clampd(s->jit / k[3], 0, 4);
    out[3] = (float)clampd((s->fast - s->mid) / k[4], -4, 4);
}

bool ml_anomaly_score(const aai16_ModelContract *c, const int8_t *in, const int8_t *out, uint32_t slot_mask,
                      float *score, float slot_err[ML_SLOT_COUNT])
{
    if (!c || c->feature_count != ML_ANOMALY_FEATURES || (slot_mask & ((1u << ML_SLOT_COUNT) - 1u)) == 0) {
        return false;
    }
    double total = 0.0;
    uint32_t present = 0;
    for (uint32_t s = 0; s < ML_SLOT_COUNT; s++) {
        double e = 0.0;
        if (slot_mask & (1u << s)) {
            for (uint32_t f = 0; f < ML_SLOT_FEATURES; f++) {
                const uint32_t i = s * ML_SLOT_FEATURES + f;
                const double x = ((double)in[i] - (double)c->input_zero_point) * (double)c->input_scale;
                const double y = ((double)out[i] - (double)c->output_zero_point) * (double)c->output_scale;
                e += (x - y) * (x - y);
            }
            total += e;
            present++;
        }
        slot_err[s] = (float)(e / ML_SLOT_FEATURES);
    }
    *score = (float)(total / (double)(ML_SLOT_FEATURES * present));
    return true;
}

int8_t ml_quantize(float v, float scale, int32_t zero_point)
{
    const float q = nearbyintf(v / scale) + (float)zero_point;
    return (int8_t)(q < -128.0f ? -128 : (q > 127.0f ? 127 : (int)q));
}

uint32_t ml_decode_output(const int8_t *q, uint32_t n, float scale, int32_t zero_point, float *scores)
{
    uint32_t best = 0;
    for (uint32_t i = 0; i < n; i++) {
        scores[i] = (float)((int32_t)q[i] - zero_point) * scale;
        if (q[i] > q[best]) {
            best = i;
        }
    }
    return best;
}
