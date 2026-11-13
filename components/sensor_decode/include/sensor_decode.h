#pragma once
/* Configuration-driven decoding: bit extraction, frame checks and RPN calc steps,
 * all taken from the SensorProfile. No sensor-specific code. Plain C, no ESP-IDF.
 *
 * Assumes the profile passed cm_validate_profile(); it still bounds-checks the raw
 * buffer lengths it is given, since those come from the bus at run time.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aai16.pb.h"
#include "config_limits.h"

typedef enum {
    DECODE_OK = 0,
    DECODE_BAD_FRAME,  /* a FrameCheck failed or the buffer is shorter than the profile needs */
    DECODE_CALC_ERROR, /* divide by zero, sqrt of a negative, or a non-finite result */
} decode_status_t;

/* Registers loaded from init_ops data; they persist across cycles. */
typedef struct {
    double reg[CM_REG_COUNT];
} decode_state_t;

/* Extract one field to a double (value = extracted * scale + bias). Returns false if
 * the field does not fit in `len` bytes. Exposed for unit tests. */
bool decode_extract(const aai16_DecodeField *d, const uint8_t *raw, size_t len, double *out);

/* Run the SRC_INIT decode fields over the bytes the init_ops produced. */
decode_status_t decode_init(const aai16_SensorProfile *p, const uint8_t *init_raw, size_t init_len,
                            decode_state_t *st);

/* Run checks, SRC_CYCLE decode fields and calc steps over one cycle's bytes.
 * Writes p->output_count values (r0..) to out[] (capacity CM_OUTPUT_MAX). */
decode_status_t decode_cycle(const aai16_SensorProfile *p, const decode_state_t *st,
                             const uint8_t *raw, size_t len, float out[CM_OUTPUT_MAX]);
