#include "motion_core.h"
#include <math.h>
#include <string.h>

void motion_window_reset(motion_window_t *w)
{
    w->count = 0;
    ++w->resets;
}

bool motion_window_push(motion_window_t *w, const motion_sample_t *s, motion_status_t *status)
{
    *status = MOTION_COLLECTING;
    if (s->interval_ms != 100) {
        motion_window_reset(w);
        *status = MOTION_WAIT_100MS;
        return false;
    }
    if (!s->valid || s->acquired_us < 0 || !isfinite(s->xyz[0]) ||
        !isfinite(s->xyz[1]) || !isfinite(s->xyz[2])) {
        motion_window_reset(w);
        *status = MOTION_INVALID_SAMPLE;
        return false;
    }
    if (w->count) {
        const motion_sample_t *last = &w->samples[w->count - 1];
        int64_t dt = s->acquired_us - last->acquired_us;
        if (s->boot_id != last->boot_id || s->generation != last->generation ||
            s->sequence != last->sequence + UINT32_C(1) || dt <= 0 || dt > 150000) {
            motion_window_reset(w);
            *status = MOTION_DISCONTINUITY;
        }
    }
    // Defensive bound even if a consumer forgets to advance after a window.
    if (w->count == MOTION_SAMPLES) {
        motion_window_reset(w);
        *status = MOTION_DISCONTINUITY;
    }
    w->samples[w->count++] = *s;
    return w->count == MOTION_SAMPLES;
}

void motion_window_advance(motion_window_t *w)
{
    if (w->count != MOTION_SAMPLES) { motion_window_reset(w); return; }
    memmove(w->samples, w->samples + MOTION_STRIDE,
            (MOTION_SAMPLES - MOTION_STRIDE) * sizeof(w->samples[0]));
    w->count = MOTION_SAMPLES - MOTION_STRIDE;
}

bool motion_preprocess(const float raw[MOTION_SAMPLES][MOTION_AXES], float features[MOTION_INPUTS])
{
    float mean[3] = {0};
    for (unsigned i = 0; i < MOTION_SAMPLES; ++i)
        for (unsigned a = 0; a < 3; ++a) {
            if (!isfinite(raw[i][a])) return false;
            mean[a] += raw[i][a];
        }
    for (unsigned a = 0; a < 3; ++a) {
        mean[a] /= 20.0f;
        if (!isfinite(mean[a])) return false;
    }
    for (unsigned i = 0; i < MOTION_SAMPLES; ++i)
        for (unsigned a = 0; a < 3; ++a) {
            float centered = raw[i][a] - mean[a];
            features[3 * i + a] = fminf(2.0f, fmaxf(-2.0f, centered)) / 2.0f;
        }
    return true;
}

bool motion_quantize(const float features[60], float scale, int zp, int8_t encoded[60], uint32_t *clipped)
{
    if (!isfinite(scale) || scale <= 0 || zp < -128 || zp > 127) return false;
    *clipped = 0;
    for (unsigned i = 0; i < 60; ++i) {
        float value = features[i] / scale;
        if (!isfinite(value)) return false;
        // Explicit nearest/ties-to-even, independent of the MCU rounding mode.
        float lower = floorf(value);
        float fraction = value - lower;
        float rounded = lower;
        if (fraction > 0.5f || (fraction == 0.5f && fmodf(lower, 2.0f) != 0)) rounded += 1.0f;
        float q = rounded + (float)zp;
        if (q < -128 || q > 127) ++*clipped;
        encoded[i] = (int8_t)fminf(127.0f, fmaxf(-128.0f, q));
    }
    return true;
}

int motion_decode(const int8_t output[2], float scale, int zp, float logits[2])
{
    if (!isfinite(scale) || scale <= 0 || zp < -128 || zp > 127) return -1;
    for (unsigned i = 0; i < 2; ++i) logits[i] = (float)((int)output[i] - zp) * scale;
    return output[1] > output[0] ? 1 : 0;
}

const char *motion_status_name(motion_status_t status)
{
    static const char *const names[] = {"collecting", "prediction", "requires_100ms",
        "invalid_sample", "discontinuity", "no_data", "queue_overflow", "init_failed",
        "invoke_failed", "sensor_disabled", "sensor_unavailable"};
    return (unsigned)status < sizeof(names) / sizeof(names[0]) ? names[status] : "unknown";
}
