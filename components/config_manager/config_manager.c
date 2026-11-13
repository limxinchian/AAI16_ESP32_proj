/* Config Manager: validation of sensor profiles (pins, operations, decoding, calculation steps), request-level checks,
 * two-phase apply with rollback, persistence/restore, and dormant profiles (stored but not running). */
#include "config_manager.h"

#include <math.h>
#include <string.h>

#include "pb_decode.h"
#include "pb_encode.h"

/* Keep config_limits.h and protocol/aai16.options in step. */
#define ARRAY_LEN(m, T, f) (sizeof(((T *)0)->f) / sizeof(((T *)0)->f[0]))
_Static_assert(ARRAY_LEN(m, aai16_Reading, values) == CM_OUTPUT_MAX, "Reading.values != CM_OUTPUT_MAX");
_Static_assert(ARRAY_LEN(m, aai16_SensorProfile, decode) == CM_DECODE_MAX, "decode != CM_DECODE_MAX");
_Static_assert(ARRAY_LEN(m, aai16_SensorProfile, init_ops) == CM_INIT_OPS_MAX, "init_ops != CM_INIT_OPS_MAX");
_Static_assert(ARRAY_LEN(m, aai16_SensorProfile, calc) == CM_CALC_STEPS_MAX, "calc != CM_CALC_STEPS_MAX");
_Static_assert(ARRAY_LEN(m, aai16_SensorProfile, program) == CM_PROGRAM_MAX, "program != CM_PROGRAM_MAX");
_Static_assert(ARRAY_LEN(m, aai16_SensorProfile, checks) == CM_CHECKS_MAX, "checks != CM_CHECKS_MAX");
_Static_assert(sizeof(((aai16_Op *)0)->tail.bytes) == CM_TAIL_MAX, "Op.tail != CM_TAIL_MAX");
_Static_assert(ARRAY_LEN(m, aai16_SensorProfile, output_kinds) == CM_OUTPUT_MAX, "output_kinds != CM_OUTPUT_MAX");
_Static_assert(sizeof(((aai16_Reading *)0)->raw.bytes) == CM_MAX_RX_BYTES,
               "Reading.raw max_size != CM_MAX_RX_BYTES");
_Static_assert(sizeof(((aai16_ConfigApply *)0)->profiles) / sizeof(aai16_SensorProfile) == CM_BUS_COUNT,
               "ConfigApply.profiles max_count != CM_BUS_COUNT");
_Static_assert(sizeof(((aai16_CommittedConfig *)0)->profiles) / sizeof(aai16_SensorProfile) == CM_BUS_COUNT,
               "CommittedConfig.profiles max_count != CM_BUS_COUNT");

#define MAX_PROFILE_PINS 6

static inline size_t bus_slot(aai16_BusType bus) { return (size_t)bus - 1; }

static bool fail(cm_error_t *err, aai16_Reason reason, uint32_t sensor_id, uint32_t detail)
{
    if (err) {
        err->reason = reason;
        err->sensor_id = sensor_id;
        err->detail = detail;
    }
    return false;
}

/* ESP32-S3-N16R8: GPIO26-32 are SPI flash, 33-37 octal PSRAM, 19/20 USB, 43/44 console,
 * 0/3/45/46 strapping. This is the team's conservative choice - verify against the board. */
bool cm_pin_allowed(uint32_t g)
{
    return g == 1 || g == 2 || (g >= 4 && g <= 18) || g == 21 || (g >= 38 && g <= 42) ||
           g == 47 || g == 48;
}

static bool in_range(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }

/* Collect the pins a (enabled) profile claims. */
static size_t profile_pins(const aai16_SensorProfile *p, uint32_t out[MAX_PROFILE_PINS])
{
    size_t n = 0;
    switch (p->which_params) {
    case aai16_SensorProfile_i2c_tag:
        out[n++] = p->params.i2c.sda;
        out[n++] = p->params.i2c.scl;
        break;
    case aai16_SensorProfile_uart_tag:
        out[n++] = p->params.uart.tx;
        out[n++] = p->params.uart.rx;
        break;
    case aai16_SensorProfile_spi_tag:
        out[n++] = p->params.spi.sclk;
        out[n++] = p->params.spi.mosi;
        out[n++] = p->params.spi.miso;
        out[n++] = p->params.spi.cs;
        break;
    case aai16_SensorProfile_gpio_tag:
        out[n++] = p->params.gpio.data_pin;
        if (p->params.gpio.aux_pin != 0) {
            out[n++] = p->params.gpio.aux_pin;
        }
        break;
    default:
        break;
    }
    return n;
}

static bool params_match_bus(const aai16_SensorProfile *p)
{
    switch (p->bus) {
    case aai16_BusType_BUS_I2C:  return p->which_params == aai16_SensorProfile_i2c_tag;
    case aai16_BusType_BUS_UART: return p->which_params == aai16_SensorProfile_uart_tag;
    case aai16_BusType_BUS_SPI:  return p->which_params == aai16_SensorProfile_spi_tag;
    case aai16_BusType_BUS_GPIO: return p->which_params == aai16_SensorProfile_gpio_tag;
    default: return false;
    }
}

static bool validate_params(const aai16_SensorProfile *p, cm_error_t *err)
{
    const uint32_t id = p->sensor_id;
    uint32_t pins[MAX_PROFILE_PINS];
    const size_t n = profile_pins(p, pins);

    for (size_t i = 0; i < n; i++) {
        if (!cm_pin_allowed(pins[i])) {
            return fail(err, aai16_Reason_REASON_FORBIDDEN_PIN, id, pins[i]);
        }
        for (size_t j = 0; j < i; j++) {
            if (pins[i] == pins[j]) {
                return fail(err, aai16_Reason_REASON_PIN_CONFLICT, id, pins[i]);
            }
        }
    }

    switch (p->bus) {
    case aai16_BusType_BUS_I2C:
        if (!in_range(p->params.i2c.addr, CM_I2C_ADDR_MIN, CM_I2C_ADDR_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_ADDRESS, id, p->params.i2c.addr);
        }
        if (!in_range(p->params.i2c.clock_hz, CM_I2C_HZ_MIN, CM_I2C_HZ_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_CLOCK, id, p->params.i2c.clock_hz);
        }
        break;
    case aai16_BusType_BUS_UART:
        if (!in_range(p->params.uart.port, CM_UART_PORT_MIN, CM_UART_PORT_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_PORT, id, p->params.uart.port);
        }
        if (!in_range(p->params.uart.baud, CM_UART_BAUD_MIN, CM_UART_BAUD_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_BAUD, id, p->params.uart.baud);
        }
        break;
    case aai16_BusType_BUS_SPI:
        if (!in_range(p->params.spi.clock_hz, CM_SPI_HZ_MIN, CM_SPI_HZ_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_CLOCK, id, p->params.spi.clock_hz);
        }
        if (p->params.spi.mode > CM_SPI_MODE_MAX) {
            return fail(err, aai16_Reason_REASON_BAD_SPI_MODE, id, p->params.spi.mode);
        }
        break;
    default:
        break;
    }
    return true;
}

static bool op_allowed_on_bus(aai16_BusType bus, aai16_OpType t)
{
    switch (t) {
    case aai16_OpType_OP_DELAY:
        return true;
    case aai16_OpType_OP_WRITE:
    case aai16_OpType_OP_READ:
    case aai16_OpType_OP_WRITE_READ:
        return bus == aai16_BusType_BUS_I2C || bus == aai16_BusType_BUS_UART ||
               bus == aai16_BusType_BUS_SPI;
    case aai16_OpType_OP_GPIO_SET:
    case aai16_OpType_OP_GPIO_PULSE:
    case aai16_OpType_OP_GPIO_PULSE_WIDTH:
    case aai16_OpType_OP_GPIO_BIT_FRAME:
        return bus == aai16_BusType_BUS_GPIO;
    case aai16_OpType_OP_UART_SYNC_READ:
        return bus == aai16_BusType_BUS_UART;
    default:
        return false;
    }
}

static bool gpio_op_pin_ok(const aai16_SensorProfile *p, uint32_t pin)
{
    return pin == p->params.gpio.data_pin ||
           (p->params.gpio.aux_pin != 0 && pin == p->params.gpio.aux_pin);
}

/* Validates one op; adds the bytes it produces to *rx_total. */
static bool validate_op(const aai16_SensorProfile *p, const aai16_Op *op, uint32_t idx,
                        uint32_t *rx_total, cm_error_t *err)
{
    const uint32_t id = p->sensor_id;

    if (op->type < aai16_OpType_OP_WRITE || op->type > aai16_OpType_OP_UART_SYNC_READ) {
        return fail(err, aai16_Reason_REASON_UNKNOWN_OP, id, idx);
    }
    if (!op_allowed_on_bus(p->bus, op->type)) {
        return fail(err, aai16_Reason_REASON_OP_NOT_ALLOWED_ON_BUS, id, idx);
    }
    if (op->timeout_us > CM_TIMEOUT_US_MAX) {
        return fail(err, aai16_Reason_REASON_BAD_TIMEOUT, id, idx);
    }

    switch (op->type) {
    case aai16_OpType_OP_WRITE:
        if (op->tx.size == 0) {
            return fail(err, aai16_Reason_REASON_BAD_LENGTH, id, idx);
        }
        break;
    case aai16_OpType_OP_READ:
        if (!in_range(op->read_len, 1, CM_MAX_RX_BYTES)) {
            return fail(err, aai16_Reason_REASON_BAD_LENGTH, id, idx);
        }
        *rx_total += op->read_len;
        break;
    case aai16_OpType_OP_WRITE_READ:
        if (op->tx.size == 0 || !in_range(op->read_len, 1, CM_MAX_RX_BYTES)) {
            return fail(err, aai16_Reason_REASON_BAD_LENGTH, id, idx);
        }
        *rx_total += op->read_len;
        break;
    case aai16_OpType_OP_DELAY:
        if (!in_range(op->delay_us, 1, CM_DELAY_US_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_DELAY, id, idx);
        }
        break;
    case aai16_OpType_OP_GPIO_SET:
        if (!gpio_op_pin_ok(p, op->pin)) {
            return fail(err, aai16_Reason_REASON_PIN_NOT_CONFIGURED, id, idx);
        }
        break;
    case aai16_OpType_OP_GPIO_PULSE:
        if (!gpio_op_pin_ok(p, op->pin)) {
            return fail(err, aai16_Reason_REASON_PIN_NOT_CONFIGURED, id, idx);
        }
        if (!in_range(op->delay_us, 1, CM_DELAY_US_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_DELAY, id, idx);
        }
        break;
    case aai16_OpType_OP_GPIO_PULSE_WIDTH:
        if (!gpio_op_pin_ok(p, op->pin)) {
            return fail(err, aai16_Reason_REASON_PIN_NOT_CONFIGURED, id, idx);
        }
        if (!in_range(op->timeout_us, 1, CM_TIMEOUT_US_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_TIMEOUT, id, idx);
        }
        *rx_total += 4;
        break;
    case aai16_OpType_OP_GPIO_BIT_FRAME:
        if (!gpio_op_pin_ok(p, op->pin)) {
            return fail(err, aai16_Reason_REASON_PIN_NOT_CONFIGURED, id, idx);
        }
        if (!in_range(op->read_len, 1, CM_MAX_BIT_FRAME_BYTES)) {
            return fail(err, aai16_Reason_REASON_BAD_LENGTH, id, idx);
        }
        if (!in_range(op->timeout_us, 1, CM_TIMEOUT_US_MAX) ||
            !in_range(op->threshold_us, 1, CM_THRESHOLD_US_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_TIMEOUT, id, idx);
        }
        *rx_total += op->read_len;
        break;
    case aai16_OpType_OP_UART_SYNC_READ:
        if (op->tx.size == 0 || !in_range(op->read_len, op->tx.size, CM_MAX_RX_BYTES) ||
            op->tail.size > op->read_len) {
            return fail(err, aai16_Reason_REASON_BAD_LENGTH, id, idx);
        }
        if (!in_range(op->timeout_us, 1, CM_TIMEOUT_US_MAX)) {
            return fail(err, aai16_Reason_REASON_BAD_TIMEOUT, id, idx);
        }
        *rx_total += op->read_len;
        break;
    default:
        break;
    }

    if (*rx_total > CM_MAX_RX_BYTES) {
        return fail(err, aai16_Reason_REASON_BAD_LENGTH, id, idx);
    }
    return true;
}

/* Decode fields: bit extraction must fit the bytes, sources must exist, one writer per register. */
static bool validate_decode(const aai16_SensorProfile *p, uint32_t cycle_rx, uint32_t init_rx,
                            uint32_t *defined, cm_error_t *err)
{
    const uint32_t id = p->sensor_id;
    uint32_t written = 0;

    if (p->decode_count > CM_DECODE_MAX) {
        return fail(err, aai16_Reason_REASON_BAD_DECODE, id, p->decode_count);
    }
    for (uint32_t i = 0; i < p->decode_count; i++) {
        const aai16_DecodeField *d = &p->decode[i];
        const uint32_t src_len = d->source == aai16_DataSource_SRC_INIT ? init_rx : cycle_rx;
        bool ok = d->dest_reg < CM_REG_COUNT && in_range(d->width, 1, 4) &&
                  (d->source == aai16_DataSource_SRC_CYCLE || d->source == aai16_DataSource_SRC_INIT) &&
                  d->sign <= aai16_SignMode_SIGN_MAGNITUDE_SET_IS_POSITIVE &&
                  d->src_offset <= src_len && d->width <= src_len - d->src_offset &&
                  isfinite(d->scale) && isfinite(d->bias);
        if (ok) {
            ok = d->shift < d->width * 8u;
        }
        if (ok) {
            const uint32_t avail = d->width * 8u - d->shift;
            const uint32_t bits = d->value_bits ? d->value_bits : avail;
            ok = bits >= 1 && bits <= avail &&
                 (d->sign == aai16_SignMode_SIGN_UNSIGNED || bits >= 2) &&
                 !(written & (1u << d->dest_reg));
        }
        if (!ok) {
            return fail(err, d->source == aai16_DataSource_SRC_INIT ? aai16_Reason_REASON_BAD_INIT
                                                                    : aai16_Reason_REASON_BAD_DECODE,
                        id, i);
        }
        written |= 1u << d->dest_reg;
    }
    *defined = written; /* init registers persist; cycle registers are loaded before calc */
    return true;
}

static bool validate_checks(const aai16_SensorProfile *p, uint32_t cycle_rx, cm_error_t *err)
{
    if (p->checks_count > CM_CHECKS_MAX) {
        return fail(err, aai16_Reason_REASON_BAD_CHECK, p->sensor_id, p->checks_count);
    }
    for (uint32_t i = 0; i < p->checks_count; i++) {
        const aai16_FrameCheck *c = &p->checks[i];
        if (c->kind < aai16_CheckKind_CHECK_SUM8 || c->kind > aai16_CheckKind_CHECK_XOR8 ||
            c->length == 0 || c->start > cycle_rx || c->length > cycle_rx - c->start ||
            c->check_offset >= cycle_rx) {
            return fail(err, aai16_Reason_REASON_BAD_CHECK, p->sensor_id, i);
        }
    }
    return true;
}

/* Static analysis of the RPN program: every step must be stack-safe, end with exactly one
 * value, and read only registers that already hold a value. */
static bool validate_calc(const aai16_SensorProfile *p, uint32_t *defined, cm_error_t *err)
{
    const uint32_t id = p->sensor_id;
    if (p->calc_count > CM_CALC_STEPS_MAX || p->program_count > CM_PROGRAM_MAX) {
        return fail(err, aai16_Reason_REASON_BAD_CALC, id, p->calc_count);
    }
    uint32_t next = 0;
    for (uint32_t s = 0; s < p->calc_count; s++) {
        const aai16_CalcStep *st = &p->calc[s];
        if (st->dest_reg >= CM_REG_COUNT || st->token_count == 0 ||
            st->token_count > p->program_count - next) {
            return fail(err, aai16_Reason_REASON_BAD_CALC, id, s);
        }
        uint32_t depth = 0;
        for (uint32_t t = 0; t < st->token_count; t++) {
            const aai16_CalcToken *tk = &p->program[next + t];
            switch (tk->op) {
            case aai16_CalcOp_CALC_PUSH_CONST:
                if (!isfinite(tk->value) || depth >= CM_STACK_MAX) {
                    return fail(err, aai16_Reason_REASON_BAD_CALC, id, s);
                }
                depth++;
                break;
            case aai16_CalcOp_CALC_PUSH_REG:
                if (tk->reg >= CM_REG_COUNT || !(*defined & (1u << tk->reg)) || depth >= CM_STACK_MAX) {
                    return fail(err, aai16_Reason_REASON_BAD_CALC, id, s);
                }
                depth++;
                break;
            case aai16_CalcOp_CALC_ADD:
            case aai16_CalcOp_CALC_SUB:
            case aai16_CalcOp_CALC_MUL:
            case aai16_CalcOp_CALC_DIV:
            case aai16_CalcOp_CALC_MIN:
            case aai16_CalcOp_CALC_MAX:
                if (depth < 2) {
                    return fail(err, aai16_Reason_REASON_BAD_CALC, id, s);
                }
                depth--;
                break;
            case aai16_CalcOp_CALC_NEG:
            case aai16_CalcOp_CALC_ABS:
            case aai16_CalcOp_CALC_SQRT:
                if (depth < 1) {
                    return fail(err, aai16_Reason_REASON_BAD_CALC, id, s);
                }
                break;
            default: /* CALC_NOP and unknown values */
                return fail(err, aai16_Reason_REASON_BAD_CALC, id, s);
            }
        }
        if (depth != 1) {
            return fail(err, aai16_Reason_REASON_BAD_CALC, id, s);
        }
        next += st->token_count;
        *defined |= 1u << st->dest_reg;
    }
    if (next != p->program_count) { /* no orphan tokens */
        return fail(err, aai16_Reason_REASON_BAD_CALC, id, p->calc_count);
    }
    return true;
}

bool cm_validate_profile(const aai16_SensorProfile *p, cm_error_t *err)
{
    const uint32_t id = p->sensor_id;

    if (err) {
        memset(err, 0, sizeof(*err));
    }
    if (!in_range(id, CM_SENSOR_ID_MIN, CM_SENSOR_ID_MAX)) {
        return fail(err, aai16_Reason_REASON_BAD_SENSOR_ID, 0, id);
    }
    if (p->bus < aai16_BusType_BUS_GPIO || p->bus > aai16_BusType_BUS_SPI) {
        return fail(err, aai16_Reason_REASON_UNKNOWN_BUS, id, (uint32_t)p->bus);
    }
    if (!p->enabled && !p->dormant) {
        return true; /* "stop this bus": nothing else is used */
    }
    if (p->enabled && p->dormant) {
        return fail(err, aai16_Reason_REASON_BAD_DORMANT, id, 0);
    }
    if (!params_match_bus(p)) {
        return fail(err, aai16_Reason_REASON_PARAMS_MISMATCH, id, 0);
    }
    if (!validate_params(p, err)) {
        return false;
    }
    if (!in_range(p->period_ms, CM_PERIOD_MS_MIN, CM_PERIOD_MS_MAX)) {
        return fail(err, aai16_Reason_REASON_BAD_PERIOD, id, p->period_ms);
    }
    if (p->ops_count == 0) {
        return fail(err, aai16_Reason_REASON_NO_OPS, id, 0);
    }
    if (p->ops_count > sizeof(p->ops) / sizeof(p->ops[0])) {
        return fail(err, aai16_Reason_REASON_TOO_MANY_OPS, id, p->ops_count);
    }

    uint32_t rx_total = 0;
    for (uint32_t i = 0; i < p->ops_count; i++) {
        if (!validate_op(p, &p->ops[i], i, &rx_total, err)) {
            return false;
        }
    }

    uint32_t init_rx = 0;
    if (p->init_ops_count > CM_INIT_OPS_MAX) {
        return fail(err, aai16_Reason_REASON_BAD_INIT, id, p->init_ops_count);
    }
    for (uint32_t i = 0; i < p->init_ops_count; i++) {
        if (!validate_op(p, &p->init_ops[i], i, &init_rx, NULL)) {
            return fail(err, aai16_Reason_REASON_BAD_INIT, id, i);
        }
    }

    uint32_t defined = 0; /* registers holding a value at this point of the pipeline */
    if (!validate_decode(p, rx_total, init_rx, &defined, err)) {
        return false;
    }
    if (!validate_checks(p, rx_total, err)) {
        return false;
    }
    if (!validate_calc(p, &defined, err)) {
        return false;
    }
    if (p->output_count > CM_OUTPUT_MAX) {
        return fail(err, aai16_Reason_REASON_BAD_OUTPUT, id, p->output_count);
    }
    for (uint32_t i = 0; i < p->output_count; i++) {
        if (!(defined & (1u << i))) {
            return fail(err, aai16_Reason_REASON_BAD_OUTPUT, id, i);
        }
    }
    /* Every output must say what it means (ML input compatibility depends on it). */
    if (p->output_kinds_count != p->output_count) {
        return fail(err, aai16_Reason_REASON_BAD_OUTPUT, id, p->output_kinds_count);
    }
    for (uint32_t i = 0; i < p->output_kinds_count; i++) {
        if (p->output_kinds[i] < aai16_ValueKind_VALUE_ACCEL_X_G ||
            p->output_kinds[i] > aai16_ValueKind_VALUE_RADAR_DISTANCE_MM) {
            return fail(err, aai16_Reason_REASON_BAD_OUTPUT, id, i);
        }
    }
    return true;
}

/* A running or dormant profile reserves its pins (a dormant sensor must always be startable). */
static bool claims_pins(const aai16_SensorProfile *p) { return p->enabled || p->dormant; }

/* Duplicate sensor ids, duplicate buses and pin collisions across a set of profiles. */
static bool check_set(const aai16_SensorProfile *const *ps, size_t n, cm_error_t *err,
                      uint32_t *bad_index)
{
    for (size_t i = 0; i < n; i++) {
        uint32_t pins_i[MAX_PROFILE_PINS];
        const size_t ni = claims_pins(ps[i]) ? profile_pins(ps[i], pins_i) : 0;

        for (size_t j = 0; j < i; j++) {
            if (ps[i]->sensor_id == ps[j]->sensor_id) {
                *bad_index = (uint32_t)i;
                return fail(err, aai16_Reason_REASON_DUPLICATE_SENSOR_ID, ps[i]->sensor_id, 0);
            }
            if (ps[i]->bus == ps[j]->bus) {
                *bad_index = (uint32_t)i;
                return fail(err, aai16_Reason_REASON_DUPLICATE_BUS, ps[i]->sensor_id, 0);
            }
            if (!claims_pins(ps[j])) {
                continue;
            }
            uint32_t pins_j[MAX_PROFILE_PINS];
            const size_t nj = profile_pins(ps[j], pins_j);
            for (size_t a = 0; a < ni; a++) {
                for (size_t b = 0; b < nj; b++) {
                    if (pins_i[a] == pins_j[b]) {
                        *bad_index = (uint32_t)i;
                        return fail(err, aai16_Reason_REASON_PIN_CONFLICT, ps[i]->sensor_id,
                                    pins_i[a]);
                    }
                }
            }
        }
    }
    return true;
}

bool cm_validate_apply(const aai16_ConfigApply *req, cm_error_t *err)
{
    if (err) {
        memset(err, 0, sizeof(*err));
    }
    if (req->profiles_count == 0) {
        return fail(err, aai16_Reason_REASON_EMPTY_REQUEST, 0, 0);
    }
    if (req->profiles_count > CM_BUS_COUNT) {
        return fail(err, aai16_Reason_REASON_TOO_MANY_PROFILES, 0, req->profiles_count);
    }

    const aai16_SensorProfile *ps[CM_BUS_COUNT];
    for (uint32_t i = 0; i < req->profiles_count; i++) {
        if (!cm_validate_profile(&req->profiles[i], err)) {
            if (err) {
                err->profile_index = i;
            }
            return false;
        }
        ps[i] = &req->profiles[i];
    }
    uint32_t bad = 0;
    if (!check_set(ps, req->profiles_count, err, &bad)) {
        if (err) {
            err->profile_index = bad;
        }
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ state */

static void emit_ack(config_manager_t *cm, uint32_t gen, aai16_ApplyPhase phase,
                     const cm_error_t *e)
{
    if (!cm->hooks.ack) {
        return;
    }
    aai16_ApplyAck ack = aai16_ApplyAck_init_zero;
    ack.generation = gen;
    ack.phase = phase;
    if (e) {
        ack.reason = e->reason;
        ack.sensor_id = e->sensor_id;
        ack.profile_index = e->profile_index;
        ack.detail = e->detail;
    }
    cm->hooks.ack(&ack, cm->hooks.ctx);
}

static void reject(config_manager_t *cm, uint32_t gen, aai16_Reason reason, uint32_t sensor_id,
                   uint32_t profile_index)
{
    cm_error_t e = {.reason = reason, .sensor_id = sensor_id, .profile_index = profile_index};
    emit_ack(cm, gen, aai16_ApplyPhase_PHASE_REJECTED, &e);
}

void cm_init(config_manager_t *cm, const cm_hooks_t *hooks)
{
    memset(cm, 0, sizeof(*cm));
    if (hooks) {
        cm->hooks = *hooks;
    }
}

/* candidate = committed overlaid with the request. */
static void build_candidate(config_manager_t *cm, const aai16_ConfigApply *req)
{
    cm->candidate = cm->committed;
    cm->candidate.generation = req->generation;
    for (uint32_t i = 0; i < req->profiles_count; i++) {
        const aai16_SensorProfile *p = &req->profiles[i];
        const size_t s = bus_slot(p->bus);
        cm->candidate.present[s] = p->enabled || p->dormant;
        cm->candidate.sleeping[s] = !p->enabled && p->dormant;
        if (p->enabled || p->dormant) {
            cm->candidate.profile[s] = *p;
        } else {
            memset(&cm->candidate.profile[s], 0, sizeof(cm->candidate.profile[s]));
        }
    }
}

/* The merged configuration must itself be consistent (ids unique, pins disjoint). */
static bool check_candidate(config_manager_t *cm, cm_error_t *err)
{
    const aai16_SensorProfile *ps[CM_BUS_COUNT];
    size_t n = 0;
    for (size_t s = 0; s < CM_BUS_COUNT; s++) {
        if (cm->candidate.present[s]) {
            ps[n++] = &cm->candidate.profile[s];
        }
    }
    uint32_t bad = 0;
    return check_set(ps, n, err, &bad);
}

static void fill_store(config_manager_t *cm, const cm_state_t *st)
{
    memset(&cm->store, 0, sizeof(cm->store));
    cm->store.schema_version = CM_SCHEMA_VERSION;
    cm->store.generation = st->generation;
    for (size_t s = 0; s < CM_BUS_COUNT; s++) {
        if (st->present[s]) {
            cm->store.profiles[cm->store.profiles_count++] = st->profile[s];
        }
    }
}

static void rollback(config_manager_t *cm, const bool switched[CM_BUS_COUNT])
{
    for (size_t s = 0; s < CM_BUS_COUNT; s++) {
        if (!switched[s]) {
            continue;
        }
        const aai16_BusType bus = (aai16_BusType)(s + 1);
        if (cm->committed.present[s] && !cm->committed.sleeping[s]) {
            if (cm->hooks.apply) {
                (void)cm->hooks.apply(&cm->committed.profile[s], cm->hooks.ctx);
            }
        } else if (cm->hooks.stop) {
            cm->hooks.stop(bus, cm->hooks.ctx);
        }
    }
}

void cm_handle_apply(config_manager_t *cm, const aai16_ConfigApply *req)
{
    cm_error_t e;
    const uint32_t gen = req->generation;

    if (gen == 0 || gen < cm->committed.generation) {
        reject(cm, gen, aai16_Reason_REASON_STALE_GENERATION, 0, 0);
        return;
    }
    if (gen == cm->committed.generation) {
        reject(cm, gen, aai16_Reason_REASON_DUPLICATE_GENERATION, 0, 0);
        return;
    }
    if (!cm_validate_apply(req, &e)) {
        emit_ack(cm, gen, aai16_ApplyPhase_PHASE_REJECTED, &e);
        return;
    }

    build_candidate(cm, req);
    if (!check_candidate(cm, &e)) {
        emit_ack(cm, gen, aai16_ApplyPhase_PHASE_REJECTED, &e);
        return;
    }

    emit_ack(cm, gen, aai16_ApplyPhase_PHASE_ADMITTED, NULL);

    bool switched[CM_BUS_COUNT] = {false};
    for (uint32_t i = 0; i < req->profiles_count; i++) {
        const aai16_SensorProfile *p = &req->profiles[i];
        const size_t s = bus_slot(p->bus);
        bool ok = true;
        switched[s] = true;
        if (p->enabled) {
            ok = cm->hooks.apply ? cm->hooks.apply(p, cm->hooks.ctx) : true;
        } else if (cm->hooks.stop) {
            cm->hooks.stop(p->bus, cm->hooks.ctx);
        }
        if (!ok) {
            rollback(cm, switched);
            reject(cm, gen, aai16_Reason_REASON_APPLY_FAILED, p->sensor_id, i);
            return;
        }
    }

    if (cm->hooks.persist) {
        fill_store(cm, &cm->candidate);
        if (!cm->hooks.persist(&cm->store, cm->hooks.ctx)) {
            rollback(cm, switched);
            reject(cm, gen, aai16_Reason_REASON_PERSIST_FAILED, 0, 0);
            return;
        }
    }

    cm->committed = cm->candidate;
    emit_ack(cm, gen, aai16_ApplyPhase_PHASE_APPLIED, NULL);
}

void cm_handle_bytes(config_manager_t *cm, const uint8_t *buf, size_t len)
{
    memset(&cm->rx, 0, sizeof(cm->rx));
    pb_istream_t is = pb_istream_from_buffer(buf, len);
    if (!pb_decode(&is, aai16_ConfigApply_fields, &cm->rx)) {
        reject(cm, 0, aai16_Reason_REASON_MALFORMED, 0, 0);
        return;
    }
    cm_handle_apply(cm, &cm->rx);
}

/* ------------------------------------------------------------ persistence */

bool cm_encode_committed(config_manager_t *cm, uint8_t *buf, size_t cap, size_t *out_len)
{
    fill_store(cm, &cm->committed);
    pb_ostream_t os = pb_ostream_from_buffer(buf, cap);
    if (!pb_encode(&os, aai16_CommittedConfig_fields, &cm->store)) {
        return false;
    }
    *out_len = os.bytes_written;
    return true;
}

bool cm_restore(config_manager_t *cm, const uint8_t *buf, size_t len)
{
    memset(&cm->store, 0, sizeof(cm->store));
    pb_istream_t is = pb_istream_from_buffer(buf, len);
    if (!pb_decode(&is, aai16_CommittedConfig_fields, &cm->store)) {
        return false;
    }
    if (cm->store.schema_version != CM_SCHEMA_VERSION || cm->store.generation == 0) {
        return false;
    }

    /* Re-run the same validation as a live request; stored data is not trusted. */
    memset(&cm->rx, 0, sizeof(cm->rx));
    cm->rx.generation = cm->store.generation;
    cm->rx.profiles_count = cm->store.profiles_count;
    for (uint32_t i = 0; i < cm->store.profiles_count; i++) {
        if (!cm->store.profiles[i].enabled && !cm->store.profiles[i].dormant) {
            return false; /* committed state only ever holds running or dormant profiles */
        }
        cm->rx.profiles[i] = cm->store.profiles[i];
    }
    /* An empty committed set is legitimate (everything was cleared); it still carries the generation. */
    if (cm->rx.profiles_count > 0 && !cm_validate_apply(&cm->rx, NULL)) {
        return false;
    }

    /* NOT a stack local: four profiles are tens of KB and this runs on the 8 KB main task at boot
     * (found on the real board: stack overflow -> corrupted log lock -> interrupt WDT -> boot loop). */
    cm_state_t *const fresh_p = &cm->candidate;
    memset(fresh_p, 0, sizeof(*fresh_p));
    fresh_p->generation = cm->store.generation;
    for (uint32_t i = 0; i < cm->rx.profiles_count; i++) {
        const size_t s = bus_slot(cm->rx.profiles[i].bus);
        fresh_p->present[s] = true;
        fresh_p->sleeping[s] = !cm->rx.profiles[i].enabled;
        fresh_p->profile[s] = cm->rx.profiles[i];
    }

    bool started[CM_BUS_COUNT] = {false};
    for (size_t s = 0; s < CM_BUS_COUNT; s++) {
        if (!fresh_p->present[s] || fresh_p->sleeping[s]) {
            continue; /* dormant sensors stay asleep after a reboot */
        }
        started[s] = true;
        if (cm->hooks.apply && !cm->hooks.apply(&fresh_p->profile[s], cm->hooks.ctx)) {
            for (size_t k = 0; k < CM_BUS_COUNT; k++) {
                if (started[k] && cm->hooks.stop) {
                    cm->hooks.stop((aai16_BusType)(k + 1), cm->hooks.ctx);
                }
            }
            return false;
        }
    }
    cm->committed = *fresh_p;
    return true;
}

/* ---------------------------------------------------------------- helpers */

bool cm_encode_ack(const aai16_ApplyAck *ack, uint8_t *buf, size_t cap, size_t *out_len)
{
    pb_ostream_t os = pb_ostream_from_buffer(buf, cap);
    if (!pb_encode(&os, aai16_ApplyAck_fields, ack)) {
        return false;
    }
    *out_len = os.bytes_written;
    return true;
}

bool cm_pin_in_use(const config_manager_t *cm, uint32_t gpio)
{
    for (size_t s = 0; s < CM_BUS_COUNT; s++) {
        if (!cm->committed.present[s]) {
            continue;
        }
        uint32_t pins[MAX_PROFILE_PINS];
        const size_t n = profile_pins(&cm->committed.profile[s], pins);
        for (size_t i = 0; i < n; i++) {
            if (pins[i] == gpio) {
                return true;
            }
        }
    }
    return false;
}

const aai16_SensorProfile *cm_committed_profile(const config_manager_t *cm, aai16_BusType bus)
{
    if (bus < aai16_BusType_BUS_GPIO || bus > aai16_BusType_BUS_SPI) {
        return NULL;
    }
    const size_t s = bus_slot(bus);
    return (cm->committed.present[s] && !cm->committed.sleeping[s]) ? &cm->committed.profile[s] : NULL;
}

const aai16_SensorProfile *cm_dormant_by_id(const config_manager_t *cm, uint32_t sensor_id)
{
    for (size_t s = 0; s < CM_BUS_COUNT; s++) {
        if (cm->committed.present[s] && cm->committed.sleeping[s] && cm->committed.profile[s].sensor_id == sensor_id) {
            return &cm->committed.profile[s];
        }
    }
    return NULL;
}
