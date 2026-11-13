#pragma once
/* Model package:  "AMDL" | u32 LE contract_len | ModelContract (nanopb) | .tflite
 * `data` must be contiguous (RAM, or a memory-mapped flash partition). */
#include <stddef.h>
#include <stdint.h>

#include "aai16.pb.h"

#define ML_PACKAGE_MAGIC "AMDL"
#define ML_CONTRACT_MAX_BYTES 512

typedef struct {
    aai16_ModelContract contract;
    const uint8_t *tflite; /* points into the package data */
    size_t tflite_len;
} ml_package_t;

/* Checks magic, lengths, contract decode, contract sanity (shape matches its preprocessing, channel
 * kinds, window/period/stride/limits, scales, labels), the "TFL3" flatbuffer marker and the SHA-256 of
 * the embedded .tflite. Returns MODEL_REASON_NONE on success. */
aai16_ModelReason ml_package_parse(const uint8_t *data, size_t len, ml_package_t *out);
