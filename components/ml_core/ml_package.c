/* Model package parsing: "AMDL" | length | ModelContract | .tflite, with contract validation (kind, shapes, channels, anomaly threshold). */
#include "ml_package.h"

#include <math.h>
#include <string.h>

#include "ml_features.h"
#include "pb_decode.h"
#include "sha256.h"

static aai16_ModelReason check_contract(const aai16_ModelContract *c)
{
    uint32_t window, channels, features;
    if (c->contract_version != 1 || c->format != aai16_ModelFormat_FORMAT_TFLITE) {
        return aai16_ModelReason_MODEL_REASON_UNSUPPORTED_FORMAT;
    }
    if (!ml_preproc_shape(c->preprocessing, &window, &channels, &features)) {
        return aai16_ModelReason_MODEL_REASON_UNSUPPORTED_PREPROC;
    }
    const bool ok =
        (c->kind == aai16_ModelKind_MODEL_MOTION || c->kind == aai16_ModelKind_MODEL_ENVIRONMENT ||
         c->kind == aai16_ModelKind_MODEL_ANOMALY) &&
        ((c->kind == aai16_ModelKind_MODEL_ANOMALY) == (c->preprocessing == aai16_Preprocessing_PREPROC_SLOT_STATS_V1)) &&
        (c->kind != aai16_ModelKind_MODEL_ANOMALY || (isfinite(c->anomaly_threshold) && c->anomaly_threshold > 0 &&
                                                      c->output_count == features)) &&
        (c->kind == aai16_ModelKind_MODEL_ANOMALY || c->output_count == 0 || c->output_count == c->labels_count) &&
        c->window_samples == window && c->channels_count == channels && c->feature_count == features &&
        features <= ML_MAX_FEATURES && c->sample_period_ms >= 1 && c->sample_period_ms <= 60000 &&
        c->period_tolerance_pct <= 100 && c->max_age_ms >= 1 && c->max_age_ms <= 600000 &&
        c->stride_samples <= window && c->tensor_arena_bytes >= 1024 && c->tensor_arena_bytes <= (512u * 1024u) &&
        isfinite(c->input_scale) && c->input_scale > 0 && isfinite(c->output_scale) && c->output_scale > 0 &&
        c->input_zero_point >= -128 && c->input_zero_point <= 127 && c->output_zero_point >= -128 &&
        c->output_zero_point <= 127 && c->labels_count >= 2 && c->tflite_sha256.size == 32;
    if (!ok) {
        return aai16_ModelReason_MODEL_REASON_BAD_PACKAGE;
    }
    for (uint32_t i = 0; i < c->channels_count; i++) {
        if (c->channels[i] < aai16_ValueKind_VALUE_ACCEL_X_G || c->channels[i] > aai16_ValueKind_VALUE_RADAR_DISTANCE_MM) {
            return aai16_ModelReason_MODEL_REASON_BAD_PACKAGE;
        }
    }
    return aai16_ModelReason_MODEL_REASON_NONE;
}

aai16_ModelReason ml_package_parse(const uint8_t *data, size_t len, ml_package_t *out)
{
    if (len < 8 || memcmp(data, ML_PACKAGE_MAGIC, 4) != 0) {
        return aai16_ModelReason_MODEL_REASON_BAD_PACKAGE;
    }
    const uint32_t clen = (uint32_t)data[4] | (uint32_t)data[5] << 8 | (uint32_t)data[6] << 16 | (uint32_t)data[7] << 24;
    if (clen == 0 || clen > ML_CONTRACT_MAX_BYTES || clen > len - 8) {
        return aai16_ModelReason_MODEL_REASON_BAD_PACKAGE;
    }
    const size_t tlen = len - 8 - clen;
    const uint8_t *tfl = data + 8 + clen;
    if (tlen < 8 || memcmp(tfl + 4, "TFL3", 4) != 0) { /* flatbuffer file identifier */
        return aai16_ModelReason_MODEL_REASON_BAD_PACKAGE;
    }

    memset(out, 0, sizeof(*out));
    pb_istream_t is = pb_istream_from_buffer(data + 8, clen);
    if (!pb_decode(&is, aai16_ModelContract_fields, &out->contract)) {
        return aai16_ModelReason_MODEL_REASON_BAD_PACKAGE;
    }
    const aai16_ModelReason r = check_contract(&out->contract);
    if (r != aai16_ModelReason_MODEL_REASON_NONE) {
        return r;
    }
    uint8_t digest[32];
    sha256_ctx_t h;
    sha256_init(&h);
    sha256_update(&h, tfl, tlen);
    sha256_final(&h, digest);
    if (memcmp(digest, out->contract.tflite_sha256.bytes, 32) != 0) {
        return aai16_ModelReason_MODEL_REASON_DIGEST_MISMATCH;
    }
    out->tflite = tfl;
    out->tflite_len = tlen;
    return aai16_ModelReason_MODEL_REASON_NONE;
}
