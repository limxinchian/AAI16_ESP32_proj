/* Generic bus operation executor and per-bus scheduler: runs a profile's operations on a bus backend, decodes the
 * result and delivers one Reading per cycle. */
#include "sensor_runtime.h"

#include <string.h>

static size_t slot_of(aai16_BusType bus) { return (size_t)bus - 1; }

static uint32_t timeout_of(const aai16_Op *op) { return op->timeout_us ? op->timeout_us : RT_DEFAULT_TIMEOUT_US; }

bus_status_t rt_run_ops(const bus_backend_t *be, const aai16_SensorProfile *p, const aai16_Op *ops,
                        uint32_t n_ops, uint8_t *rx, size_t cap, size_t *rx_len)
{
    size_t used = 0;
    *rx_len = 0;
    for (uint32_t i = 0; i < n_ops; i++) {
        const aai16_Op *op = &ops[i];
        const uint32_t to = timeout_of(op);
        bus_status_t st = BUS_OK;
        switch (op->type) {
        case aai16_OpType_OP_DELAY:
            be->delay_us(be->ctx, op->delay_us);
            break;
        case aai16_OpType_OP_WRITE:
        case aai16_OpType_OP_READ:
        case aai16_OpType_OP_WRITE_READ: {
            const bool w = op->type != aai16_OpType_OP_READ;
            const bool r = op->type != aai16_OpType_OP_WRITE;
            const size_t rn = r ? op->read_len : 0;
            if (used + rn > cap) {
                return BUS_ERROR;
            }
            const uint8_t *tx = w ? op->tx.bytes : NULL;
            const size_t txn = w ? op->tx.size : 0;
            switch (p->bus) {
            case aai16_BusType_BUS_I2C:
                st = be->i2c_xfer(be->ctx, &p->params.i2c, tx, txn, rx + used, rn, to);
                break;
            case aai16_BusType_BUS_UART:
                st = be->uart_xfer(be->ctx, &p->params.uart, tx, txn, rx + used, rn, to);
                break;
            case aai16_BusType_BUS_SPI:
                st = be->spi_xfer(be->ctx, &p->params.spi, tx, txn, rx + used, rn, to);
                break;
            default:
                return BUS_ERROR;
            }
            if (st == BUS_OK) {
                used += rn; /* a failed transfer delivered nothing: never report requested bytes as received */
            }
            break;
        }
        case aai16_OpType_OP_UART_SYNC_READ:
            if (used + op->read_len > cap) {
                return BUS_ERROR;
            }
            st = be->uart_sync_read(be->ctx, &p->params.uart, op->tx.bytes, op->tx.size, op->tail.bytes,
                                    op->tail.size, rx + used, op->read_len, to);
            if (st == BUS_OK) {
                used += op->read_len;
            }
            break;
        case aai16_OpType_OP_GPIO_SET:
            st = be->gpio_set(be->ctx, op->pin, op->level);
            break;
        case aai16_OpType_OP_GPIO_PULSE:
            st = be->gpio_pulse(be->ctx, op->pin, op->level, op->delay_us);
            break;
        case aai16_OpType_OP_GPIO_PULSE_WIDTH: {
            uint32_t w = 0;
            if (used + 4 > cap) {
                return BUS_ERROR;
            }
            st = be->gpio_pulse_width(be->ctx, op->pin, op->level, op->timeout_us, &w);
            if (st == BUS_OK) {
                rx[used++] = (uint8_t)(w >> 24);
                rx[used++] = (uint8_t)(w >> 16);
                rx[used++] = (uint8_t)(w >> 8);
                rx[used++] = (uint8_t)w;
            }
            break;
        }
        case aai16_OpType_OP_GPIO_BIT_FRAME:
            if (used + op->read_len > cap) {
                return BUS_ERROR;
            }
            st = be->gpio_bit_frame(be->ctx, op->pin, op->read_len, op->timeout_us, op->threshold_us, rx + used);
            if (st == BUS_OK) {
                used += op->read_len;
            }
            break;
        default:
            return BUS_ERROR;
        }
        if (st != BUS_OK) {
            *rx_len = used;
            return st;
        }
    }
    *rx_len = used;
    return BUS_OK;
}

void rt_init(sensor_runtime_t *rt, const bus_backend_t *be, rt_reading_fn cb, void *cb_ctx)
{
    memset(rt, 0, sizeof(*rt));
    rt->be = *be;
    rt->on_reading = cb;
    rt->cb_ctx = cb_ctx;
}

void rt_set_generation(sensor_runtime_t *rt, uint32_t generation) { rt->generation = generation; }

bool rt_apply(const aai16_SensorProfile *p, void *ctx)
{
    sensor_runtime_t *rt = ctx;
    if (p->bus < aai16_BusType_BUS_GPIO || p->bus > aai16_BusType_BUS_SPI) {
        return false;
    }
    rt_slot_t *s = &rt->slot[slot_of(p->bus)];
    memset(s, 0, sizeof(*s));
    s->active = true;
    s->profile = *p;
    s->next_due_us = rt->be.now_us(rt->be.ctx); /* first cycle immediately; init runs inside it */
    return true;
}

void rt_stop(aai16_BusType bus, void *ctx)
{
    sensor_runtime_t *rt = ctx;
    if (bus >= aai16_BusType_BUS_GPIO && bus <= aai16_BusType_BUS_SPI) {
        memset(&rt->slot[slot_of(bus)], 0, sizeof(rt->slot[0]));
    }
}

static void deliver(sensor_runtime_t *rt, rt_slot_t *s, uint64_t ts, aai16_ReadError err, const uint8_t *raw, size_t raw_len)
{
    aai16_Reading *r = &rt->scratch;
    r->sensor_id = s->profile.sensor_id;
    r->generation = rt->generation;
    r->timestamp_us = ts;
    r->valid = (err == aai16_ReadError_READ_OK);
    r->error = err;
    r->raw.size = (pb_size_t)(raw_len > sizeof(r->raw.bytes) ? sizeof(r->raw.bytes) : raw_len);
    if (raw && r->raw.size) {
        memcpy(r->raw.bytes, raw, r->raw.size);
    }
    if (rt->on_reading) {
        rt->on_reading(r, rt->cb_ctx);
    }
}

static aai16_ReadError map_bus(bus_status_t st)
{
    switch (st) {
    case BUS_OK:        return aai16_ReadError_READ_OK;
    case BUS_TIMEOUT:   return aai16_ReadError_READ_TIMEOUT;
    case BUS_BAD_FRAME: return aai16_ReadError_READ_BAD_FRAME;
    default:            return aai16_ReadError_READ_BUS_ERROR;
    }
}

static void run_cycle(sensor_runtime_t *rt, rt_slot_t *s)
{
    const bus_backend_t *be = &rt->be;
    const aai16_SensorProfile *p = &s->profile;
    uint8_t raw[CM_MAX_RX_BYTES] = {0}; /* never publish uninitialised stack bytes */
    size_t raw_len = 0;
    const uint64_t ts = be->now_us(be->ctx);

    rt->scratch.values_count = 0;
    s->stats.cycles++;
    if (s->stats.last_ts_us) {
        s->stats.last_period_us = (uint32_t)(ts - s->stats.last_ts_us);
    }
    s->stats.last_ts_us = ts;

    if (!s->init_done) { /* calibration/config ops; retried until they succeed */
        uint8_t init_raw[CM_INIT_RX_MAX] = {0};
        size_t init_len = 0;
        bus_status_t st = rt_run_ops(be, p, p->init_ops, p->init_ops_count, init_raw, sizeof(init_raw), &init_len);
        if (st == BUS_OK && decode_init(p, init_raw, init_len, &s->dec) != DECODE_OK) {
            st = BUS_BAD_FRAME;
        }
        if (st != BUS_OK) {
            s->stats.errors++;
            deliver(rt, s, ts, map_bus(st), NULL, 0);
            s->next_due_us = ts + (p->period_ms * 1000u > RT_INIT_RETRY_US ? p->period_ms * 1000u : RT_INIT_RETRY_US);
            return;
        }
        s->init_done = true;
    }

    bus_status_t st = rt_run_ops(be, p, p->ops, p->ops_count, raw, sizeof(raw), &raw_len);
    aai16_ReadError err = map_bus(st);
    if (st == BUS_OK) {
        float out[CM_OUTPUT_MAX] = {0};
        const decode_status_t d = decode_cycle(p, &s->dec, raw, raw_len, out);
        if (d == DECODE_BAD_FRAME) {
            err = aai16_ReadError_READ_BAD_FRAME;
        } else if (d == DECODE_CALC_ERROR) {
            err = aai16_ReadError_READ_CALC_ERROR;
        } else {
            rt->scratch.values_count = p->output_count;
            memcpy(rt->scratch.values, out, p->output_count * sizeof(float));
        }
    }
    if (err == aai16_ReadError_READ_OK) {
        s->stats.ok++;
    } else {
        s->stats.errors++;
        if (st == BUS_ERROR) {
            s->init_done = false; /* a bus fault may mean the chip reset: re-run init next time */
        }
    }
    deliver(rt, s, ts, err, raw, raw_len);

    /* schedule: keep the phase, never burst to catch up */
    const uint64_t period = (uint64_t)p->period_ms * 1000u;
    const uint64_t now = be->now_us(be->ctx);
    s->next_due_us += period;
    while (s->next_due_us <= now) {
        s->next_due_us += period;
        s->stats.overruns++;
    }
}

uint32_t rt_poll(sensor_runtime_t *rt)
{
    uint32_t ran = 0;
    for (size_t i = 0; i < CM_BUS_COUNT; i++) {
        rt_slot_t *s = &rt->slot[i];
        if (s->active && rt->be.now_us(rt->be.ctx) >= s->next_due_us) {
            run_cycle(rt, s);
            ran++;
        }
    }
    return ran;
}

uint64_t rt_next_due(const sensor_runtime_t *rt)
{
    uint64_t best = 0;
    for (size_t i = 0; i < CM_BUS_COUNT; i++) {
        if (rt->slot[i].active && (best == 0 || rt->slot[i].next_due_us < best)) {
            best = rt->slot[i].next_due_us;
        }
    }
    return best;
}

const rt_stats_t *rt_stats(const sensor_runtime_t *rt, aai16_BusType bus)
{
    return (bus >= aai16_BusType_BUS_GPIO && bus <= aai16_BusType_BUS_SPI) ? &rt->slot[slot_of(bus)].stats : NULL;
}
