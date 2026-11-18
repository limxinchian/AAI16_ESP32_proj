/* Regression test: a failed bus transfer must never publish uninitialised stack bytes or requested-but-not-received
 * bytes in Reading.raw. Simulated I2C bus; the stack is poisoned with 0xAA before every cycle, so any 0xAA in
 * Reading.raw came from uninitialised memory.
 *
 *   A) write accepted, read times out              -> invalid reading, raw is EMPTY (nothing was received)
 *   B) first read OK (2 bytes), second read times out -> invalid reading, raw holds ONLY the first read's bytes
 *   C) every operation succeeds                    -> valid reading, raw holds both reads (behaviour unchanged)
 *   D) UART sync read times out                    -> invalid reading, raw is EMPTY
 *   E) GPIO bit frame (DHT22 style) times out      -> invalid reading, raw is EMPTY
 *
 * Found by an outside review of commit 2d64ff1 (components/sensor_runtime/sensor_runtime.c: run_cycle / rt_run_ops).
 * Run by tests/runtime_host/run.py. */
#include <stdio.h>
#include <string.h>

#include "sensor_runtime.h"

static int g_mode, g_reads;

/* mode 0: every read times out; mode 1: first read ok, later reads time out; mode 2: everything succeeds */
static bus_status_t i2c_sim(void *c, const aai16_I2cParams *p, const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn, uint32_t to)
{
    (void)c; (void)p; (void)tx; (void)txn; (void)to;
    if (rxn == 0) return BUS_OK;                                     /* writes are always accepted */
    const int n = g_reads++;
    if (g_mode == 0 || (g_mode == 1 && n >= 1)) return BUS_TIMEOUT;  /* rx deliberately left untouched */
    for (size_t i = 0; i < rxn; i++) rx[i] = (uint8_t)(0x11 * (n + 1) + (int)i * 0x11);
    return BUS_OK;
}
static bus_status_t uart_xfer(void *c, const aai16_UartParams *p, const uint8_t *tx, size_t a, uint8_t *rx, size_t b, uint32_t t)
{
    (void)c; (void)p; (void)tx; (void)a; (void)rx; (void)b; (void)t;
    return BUS_ERROR;
}
static bus_status_t uart_sync_timeout(void *c, const aai16_UartParams *p, const uint8_t *pt, size_t a, const uint8_t *tl, size_t b,
                                      uint8_t *rx, size_t l, uint32_t t)
{
    (void)c; (void)p; (void)pt; (void)a; (void)tl; (void)b; (void)rx; (void)l; (void)t;
    return BUS_TIMEOUT;
}
static bus_status_t spi_xfer(void *c, const aai16_SpiParams *p, const uint8_t *tx, size_t a, uint8_t *rx, size_t b, uint32_t t)
{
    (void)c; (void)p; (void)tx; (void)a; (void)rx; (void)b; (void)t;
    return BUS_ERROR;
}
static bus_status_t gpio_bit_timeout(void *c, uint32_t pin, uint32_t n, uint32_t to, uint32_t th, uint8_t *out)
{
    (void)c; (void)pin; (void)n; (void)to; (void)th; (void)out;
    return BUS_TIMEOUT;
}
static void delay_us(void *c, uint32_t us);
static uint64_t g_now = 1000;
static uint64_t now_us(void *c) { (void)c; return g_now; }
static void delay_us(void *c, uint32_t us) { (void)c; g_now += us; }

static int g_size, g_aa, g_calls;
static bool g_valid;
static unsigned char g_bytes[64];
static void on_reading(const aai16_Reading *r, void *ctx)
{
    (void)ctx;
    g_calls++;
    g_valid = r->valid;
    g_size = r->raw.size;
    g_aa = 0;
    for (int i = 0; i < r->raw.size && i < 64; i++) {
        g_bytes[i] = r->raw.bytes[i];
        g_aa += r->raw.bytes[i] == 0xAA;
    }
}

static void __attribute__((noinline)) poison_stack(void)
{
    volatile unsigned char junk[4096];
    for (int i = 0; i < 4096; i++) junk[i] = 0xAA;
    (void)junk[100];
}

static int g_fail;
#define EXPECT(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL " __VA_ARGS__); printf("\n"); } } while (0)

static void run_profile(const char *name, aai16_SensorProfile *p, bool want_valid, int want_size, const unsigned char *want)
{
    static sensor_runtime_t rt;
    bus_backend_t be = {0};
    be.i2c_xfer = i2c_sim; be.uart_xfer = uart_xfer; be.uart_sync_read = uart_sync_timeout; be.spi_xfer = spi_xfer;
    be.gpio_bit_frame = gpio_bit_timeout; be.delay_us = delay_us; be.now_us = now_us;
    rt_init(&rt, &be, on_reading, NULL);
    g_reads = 0; g_size = -1; g_aa = -1; g_valid = !want_valid; g_calls = 0;
    (void)rt_apply(p, &rt);
    poison_stack();
    rt_poll(&rt);
    const int before = g_fail;
    EXPECT(g_calls == 1, "%s: expected exactly one reading, got %d", name, g_calls);
    EXPECT(g_valid == want_valid, "%s: valid=%d, expected %d", name, g_valid, want_valid);
    EXPECT(g_size == want_size, "%s: raw.size=%d, expected %d", name, g_size, want_size);
    EXPECT(g_aa == 0, "%s: %d byte(s) of Reading.raw equal the stack poison 0xAA", name, g_aa);
    for (int i = 0; i < want_size && i < g_size; i++) EXPECT(g_bytes[i] == want[i], "%s: raw[%d]=0x%02x, expected 0x%02x", name, i, g_bytes[i], want[i]);
    printf("  %s %s: valid=%d raw.size=%d\n", g_fail == before ? "ok  " : "FAIL", name, g_valid, g_size);
}

static void i2c_profile(aai16_SensorProfile *p, int nreads)
{
    memset(p, 0, sizeof(*p));
    p->sensor_id = 1; p->bus = aai16_BusType_BUS_I2C; p->enabled = true; p->period_ms = 100;
    p->which_params = aai16_SensorProfile_i2c_tag; p->params.i2c.addr = 0x19;
    p->ops_count = 1 + (uint32_t)nreads;
    p->ops[0].type = aai16_OpType_OP_WRITE; p->ops[0].tx.size = 1; p->ops[0].tx.bytes[0] = 0xA8;
    for (int i = 0; i < nreads; i++) { p->ops[1 + i].type = aai16_OpType_OP_READ; p->ops[1 + i].read_len = 2; }
}

int main(void)
{
    static aai16_SensorProfile p;
    const unsigned char b[] = {0x11, 0x22};
    const unsigned char c[] = {0x11, 0x22, 0x22, 0x33};

    puts("failed transfers must not publish uninitialised or requested-but-missing bytes");
    i2c_profile(&p, 1); g_mode = 0; run_profile("A  read times out, nothing received      ", &p, false, 0, NULL);
    i2c_profile(&p, 2); g_mode = 1; run_profile("B  first read ok, second read times out  ", &p, false, 2, b);
    i2c_profile(&p, 2); g_mode = 2; run_profile("C  every operation succeeds (unchanged)  ", &p, true, 4, c);

    memset(&p, 0, sizeof(p));                                  /* D: UART sync read (LD2450 style) times out */
    p.sensor_id = 3; p.bus = aai16_BusType_BUS_UART; p.enabled = true; p.period_ms = 100;
    p.which_params = aai16_SensorProfile_uart_tag; p.params.uart.port = 1; p.params.uart.baud = 256000;
    p.ops_count = 1; p.ops[0].type = aai16_OpType_OP_UART_SYNC_READ; p.ops[0].read_len = 30;
    p.ops[0].tx.size = 2; p.ops[0].tx.bytes[0] = 0xAA; p.ops[0].tx.bytes[1] = 0xFF;
    run_profile("D  UART sync read times out            ", &p, false, 0, NULL);

    memset(&p, 0, sizeof(p));                                  /* E: GPIO bit frame (DHT22 style) times out */
    p.sensor_id = 2; p.bus = aai16_BusType_BUS_GPIO; p.enabled = true; p.period_ms = 2000;
    p.which_params = aai16_SensorProfile_gpio_tag; p.params.gpio.data_pin = 6;
    p.ops_count = 1; p.ops[0].type = aai16_OpType_OP_GPIO_BIT_FRAME; p.ops[0].pin = 6; p.ops[0].read_len = 5; p.ops[0].timeout_us = 1000;
    run_profile("E  GPIO bit frame times out            ", &p, false, 0, NULL);

    printf("%d failure(s)\n", g_fail);
    return g_fail ? 1 : 0;
}
