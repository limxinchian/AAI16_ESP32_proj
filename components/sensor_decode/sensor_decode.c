/* Configuration-driven decoding of raw bus bytes: bit fields, signs, byte order, frame checks and RPN calculation steps. */
#include "sensor_decode.h"

#include <math.h>
#include <string.h>

bool decode_extract(const aai16_DecodeField *d, const uint8_t *raw, size_t len, double *out)
{
    if (d->width < 1 || d->width > 4 || d->src_offset > len || d->width > len - d->src_offset ||
        d->shift >= d->width * 8u) {
        return false;
    }
    const uint32_t avail = d->width * 8u - d->shift;
    const uint32_t bits = d->value_bits ? d->value_bits : avail;
    if (bits < 1 || bits > avail) {
        return false;
    }

    uint32_t v = 0;
    for (uint32_t i = 0; i < d->width; i++) {
        const uint32_t byte = raw[d->src_offset + (d->big_endian ? i : d->width - 1 - i)];
        v = (v << 8) | byte;
    }
    v >>= d->shift;
    if (bits < 32) {
        v &= (1u << bits) - 1u;
    }

    int64_t value = v;
    if (d->sign != aai16_SignMode_SIGN_UNSIGNED && bits >= 2) {
        const uint32_t sign_bit = 1u << (bits - 1);
        const bool set = (v & sign_bit) != 0;
        if (d->sign == aai16_SignMode_SIGN_TWOS_COMPLEMENT) {
            if (set) {
                value = (int64_t)v - ((int64_t)1 << bits);
            }
        } else {
            const int64_t mag = v & (sign_bit - 1u);
            const bool negative = d->sign == aai16_SignMode_SIGN_MAGNITUDE ? set : !set;
            value = negative ? -mag : mag;
        }
    }
    *out = (double)value * (double)d->scale + (double)d->bias;
    return true;
}

decode_status_t decode_init(const aai16_SensorProfile *p, const uint8_t *init_raw, size_t init_len,
                            decode_state_t *st)
{
    memset(st, 0, sizeof(*st));
    for (uint32_t i = 0; i < p->decode_count; i++) {
        const aai16_DecodeField *d = &p->decode[i];
        if (d->source != aai16_DataSource_SRC_INIT) {
            continue;
        }
        if (!decode_extract(d, init_raw, init_len, &st->reg[d->dest_reg])) {
            return DECODE_BAD_FRAME;
        }
    }
    return DECODE_OK;
}

static bool frame_check_ok(const aai16_FrameCheck *c, const uint8_t *raw, size_t len)
{
    if (c->start > len || c->length > len - c->start || c->check_offset >= len) {
        return false;
    }
    uint32_t sum = 0;
    uint8_t x = 0;
    for (uint32_t i = 0; i < c->length; i++) {
        sum += raw[c->start + i];
        x ^= raw[c->start + i];
    }
    const uint8_t got = raw[c->check_offset];
    switch (c->kind) {
    case aai16_CheckKind_CHECK_SUM8:     return got == (uint8_t)(sum & 0xFFu);
    case aai16_CheckKind_CHECK_SUM8_NEG: return got == (uint8_t)((0x100u - (sum & 0xFFu)) & 0xFFu);
    case aai16_CheckKind_CHECK_XOR8:     return got == x;
    default:                             return false;
    }
}

static bool run_step(const aai16_SensorProfile *p, uint32_t first, uint32_t count,
                     const double *reg, double *result)
{
    double stack[CM_STACK_MAX];
    uint32_t sp = 0;
    for (uint32_t t = 0; t < count; t++) {
        const aai16_CalcToken *tk = &p->program[first + t];
        switch (tk->op) {
        case aai16_CalcOp_CALC_PUSH_CONST: stack[sp++] = (double)tk->value; break;
        case aai16_CalcOp_CALC_PUSH_REG:   stack[sp++] = reg[tk->reg]; break;
        case aai16_CalcOp_CALC_ADD: stack[sp - 2] += stack[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_SUB: stack[sp - 2] -= stack[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_MUL: stack[sp - 2] *= stack[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_DIV:
            if (stack[sp - 1] == 0.0) {
                return false;
            }
            stack[sp - 2] /= stack[sp - 1];
            sp--;
            break;
        case aai16_CalcOp_CALC_MIN:
            stack[sp - 2] = fmin(stack[sp - 2], stack[sp - 1]);
            sp--;
            break;
        case aai16_CalcOp_CALC_MAX:
            stack[sp - 2] = fmax(stack[sp - 2], stack[sp - 1]);
            sp--;
            break;
        case aai16_CalcOp_CALC_NEG: stack[sp - 1] = -stack[sp - 1]; break;
        case aai16_CalcOp_CALC_ABS: stack[sp - 1] = fabs(stack[sp - 1]); break;
        case aai16_CalcOp_CALC_SQRT:
            if (stack[sp - 1] < 0.0) {
                return false;
            }
            stack[sp - 1] = sqrt(stack[sp - 1]);
            break;
        default:
            return false;
        }
        if (sp > 0 && !isfinite(stack[sp - 1])) {
            return false;
        }
    }
    if (sp != 1) {
        return false;
    }
    *result = stack[0];
    return true;
}

decode_status_t decode_cycle(const aai16_SensorProfile *p, const decode_state_t *st,
                             const uint8_t *raw, size_t len, float out[CM_OUTPUT_MAX])
{
    double reg[CM_REG_COUNT];
    memcpy(reg, st->reg, sizeof(reg));

    for (uint32_t i = 0; i < p->checks_count; i++) {
        if (!frame_check_ok(&p->checks[i], raw, len)) {
            return DECODE_BAD_FRAME;
        }
    }
    for (uint32_t i = 0; i < p->decode_count; i++) {
        const aai16_DecodeField *d = &p->decode[i];
        if (d->source == aai16_DataSource_SRC_CYCLE &&
            !decode_extract(d, raw, len, &reg[d->dest_reg])) {
            return DECODE_BAD_FRAME;
        }
    }
    uint32_t next = 0;
    for (uint32_t s = 0; s < p->calc_count; s++) {
        double r;
        if (!run_step(p, next, p->calc[s].token_count, reg, &r)) {
            return DECODE_CALC_ERROR;
        }
        reg[p->calc[s].dest_reg] = r;
        next += p->calc[s].token_count;
    }
    for (uint32_t i = 0; i < p->output_count && i < CM_OUTPUT_MAX; i++) {
        if (!isfinite(reg[i]) || fabs(reg[i]) > 3.0e38) {
            return DECODE_CALC_ERROR;
        }
        out[i] = (float)reg[i];
    }
    return DECODE_OK;
}
