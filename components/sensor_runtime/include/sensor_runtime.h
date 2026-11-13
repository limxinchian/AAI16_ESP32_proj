#pragma once
/* Generic operation executor + per-bus scheduler. Runs the bounded ops of a SensorProfile on a bus
 * backend, decodes with sensor_decode, and delivers a Reading per cycle. No sensor-specific code.
 *
 * The backend is the only hardware-facing part: tests supply simulated devices; on the board it is
 * a thin layer over ESP-IDF I2C/UART/SPI/GPIO(+RMT). Single owner task: call rt_* from one task
 * (the acquisition task); ops of different buses are executed one after another, each bus on its own
 * period.
 *
 * SPI semantics: WRITE_READ clocks max(tx.size, read_len) bytes (tx zero-padded) and returns the
 * first read_len received bytes; READ clocks read_len zero bytes; WRITE clocks tx.size bytes and
 * discards the reply.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aai16.pb.h"
#include "config_limits.h"
#include "sensor_decode.h"

typedef enum { BUS_OK = 0, BUS_TIMEOUT, BUS_ERROR, BUS_BAD_FRAME } bus_status_t;

typedef struct {
    /* tx/rx may be empty (n == 0). `timeout_us` is already defaulted by the executor. */
    bus_status_t (*i2c_xfer)(void *c, const aai16_I2cParams *p, const uint8_t *tx, size_t txn, uint8_t *rx,
                             size_t rxn, uint32_t timeout_us);
    bus_status_t (*uart_xfer)(void *c, const aai16_UartParams *p, const uint8_t *tx, size_t txn, uint8_t *rx,
                              size_t rxn, uint32_t timeout_us);
    /* find `pat`, return frame_len bytes starting at it; BUS_BAD_FRAME if `tail` does not match */
    bus_status_t (*uart_sync_read)(void *c, const aai16_UartParams *p, const uint8_t *pat, size_t patn,
                                   const uint8_t *tail, size_t tailn, uint8_t *rx, size_t frame_len,
                                   uint32_t timeout_us);
    bus_status_t (*spi_xfer)(void *c, const aai16_SpiParams *p, const uint8_t *tx, size_t txn, uint8_t *rx,
                             size_t rxn, uint32_t timeout_us);
    bus_status_t (*gpio_set)(void *c, uint32_t pin, bool level);
    bus_status_t (*gpio_pulse)(void *c, uint32_t pin, bool level, uint32_t width_us);
    bus_status_t (*gpio_pulse_width)(void *c, uint32_t pin, bool level, uint32_t timeout_us, uint32_t *width_us);
    bus_status_t (*gpio_bit_frame)(void *c, uint32_t pin, uint32_t nbytes, uint32_t timeout_us,
                                   uint32_t threshold_us, uint8_t *out);
    void (*delay_us)(void *c, uint32_t us);
    uint64_t (*now_us)(void *c);
    void *ctx;
} bus_backend_t;

#define RT_DEFAULT_TIMEOUT_US 100000u
#define RT_INIT_RETRY_US      1000000u

/* Run `ops` in order, appending produced bytes to rx. */
bus_status_t rt_run_ops(const bus_backend_t *be, const aai16_SensorProfile *p, const aai16_Op *ops,
                        uint32_t n_ops, uint8_t *rx, size_t cap, size_t *rx_len);

typedef void (*rt_reading_fn)(const aai16_Reading *r, void *ctx);

typedef struct {
    uint32_t cycles, ok, errors, overruns;
    uint64_t last_ts_us;
    uint32_t last_period_us; /* measured spacing of the last two cycles */
} rt_stats_t;

typedef struct {
    bool active;
    bool init_done;
    aai16_SensorProfile profile;
    decode_state_t dec;
    uint64_t next_due_us;
    rt_stats_t stats;
} rt_slot_t;

typedef struct {
    bus_backend_t be;
    rt_reading_fn on_reading;
    void *cb_ctx;
    uint32_t generation;
    rt_slot_t slot[CM_BUS_COUNT]; /* index = BusType - 1 */
    aai16_Reading scratch;
} sensor_runtime_t;

void rt_init(sensor_runtime_t *rt, const bus_backend_t *be, rt_reading_fn cb, void *cb_ctx);
void rt_set_generation(sensor_runtime_t *rt, uint32_t generation);

/* Hooks matching cm_hooks_t.apply / .stop (ctx = sensor_runtime_t*). */
bool rt_apply(const aai16_SensorProfile *p, void *ctx);
void rt_stop(aai16_BusType bus, void *ctx);

/* Run every due slot once. Returns the number of cycles executed. */
uint32_t rt_poll(sensor_runtime_t *rt);
/* Earliest due time over active slots (0 if none), for sleeping. */
uint64_t rt_next_due(const sensor_runtime_t *rt);
const rt_stats_t *rt_stats(const sensor_runtime_t *rt, aai16_BusType bus);
