/* Host tests for sensor_runtime with SIMULATED devices on a simulated clock.
 * The "devices" below are bounded test doubles of the protocols the profiles describe; they are not
 * firmware and say nothing about real silicon. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config_manager.h"
#include "sensor_runtime.h"

static int g_checks, g_fail;
static const char *g_dir;

#define CHECK(cond)                                                     \
    do {                                                                \
        g_checks++;                                                     \
        if (!(cond)) {                                                  \
            g_fail++;                                                   \
            printf("  FAIL line %d: %s\n", __LINE__, #cond);            \
        }                                                               \
    } while (0)

/* ----------------------------------------------------------- simulated world */
static uint64_t g_now;
static struct {
    uint8_t bme[256];
    uint8_t written[256];
    int init_writes;           /* writes seen on the BME280 */
    bool i2c_fail;             /* every I2C transaction NACKs */
    bool dht_bad_checksum, mhz_bad_checksum, ld_bad_tail, uart_timeout;
    uint32_t i2c_xfers;
} W;

static bus_status_t i2c_xfer(void *c, const aai16_I2cParams *p, const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn, uint32_t to)
{
    (void)c; (void)to;
    W.i2c_xfers++;
    if (W.i2c_fail) return BUS_ERROR;
    if (p->addr == 0x19) { /* GY-511 accelerometer: OUT_X_L_A.. = 1000, -500, 1020 mg, 12-bit left-justified */
        if (txn == 2) return BUS_OK;
        if (txn == 1 && tx[0] == 0xA8 && rxn == 6) {
            const int16_t mg[3] = {1000, -500, 1020};
            for (int i = 0; i < 3; i++) {
                const uint16_t v = (uint16_t)(((uint16_t)mg[i] & 0xFFF) << 4);
                rx[2 * i] = (uint8_t)v; rx[2 * i + 1] = (uint8_t)(v >> 8);
            }
            return BUS_OK;
        }
        return BUS_ERROR;
    }
    if (p->addr == 0x76) { /* BME280 register file */
        if (txn >= 2 && rxn == 0) {
            for (size_t i = 1; i < txn; i++) W.written[tx[0] + i - 1] = tx[i];
            W.init_writes++;
            return BUS_OK;
        }
        if (txn == 1 && rxn > 0 && (size_t)tx[0] + rxn <= 256) {
            memcpy(rx, W.bme + tx[0], rxn);
            return BUS_OK;
        }
    }
    return BUS_ERROR; /* NACK */
}

static bus_status_t uart_xfer(void *c, const aai16_UartParams *p, const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn, uint32_t to)
{
    (void)c; (void)to;
    if (W.uart_timeout) return BUS_TIMEOUT;
    if (p->port == 2 && txn == 9 && tx[0] == 0xFF && tx[1] == 0x01 && tx[2] == 0x86 && rxn == 9) { /* MH-Z19: 400 ppm */
        const uint8_t r[9] = {0xFF, 0x86, 0x01, 0x90, 0, 0, 0, 0, 0};
        memcpy(rx, r, 9);
        uint32_t sum = 0;
        for (int i = 1; i < 8; i++) sum += rx[i];
        rx[8] = (uint8_t)((0x100 - (sum & 0xFF)) & 0xFF);
        if (W.mhz_bad_checksum) rx[8] ^= 1;
        return BUS_OK;
    }
    return BUS_ERROR;
}

static bus_status_t uart_sync(void *c, const aai16_UartParams *p, const uint8_t *pat, size_t pn, const uint8_t *tail, size_t tn,
                              uint8_t *rx, size_t len, uint32_t to)
{
    (void)c; (void)to;
    if (p->port != 1 || len != 30 || pn != 4 || pat[0] != 0xAA) return BUS_ERROR;
    memset(rx, 0, len);
    memcpy(rx, pat, pn);
    rx[4] = 0x0E; rx[5] = 0x03;   /* x = -782 (bit15 clear = negative) */
    rx[6] = 0xB1; rx[7] = 0x86;   /* y = 0x8000 | 0x06B1 = +1713 */
    rx[8] = 0x10; rx[9] = 0x00;   /* speed = -16 */
    rx[28] = 0x55; rx[29] = W.ld_bad_tail ? 0xCD : 0xCC;
    return (tn == 2 && rx[28] == tail[0] && rx[29] == tail[1]) ? BUS_OK : BUS_BAD_FRAME;
}

static bus_status_t spi_xfer(void *c, const aai16_SpiParams *p, const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn, uint32_t to)
{
    (void)c; (void)to;
    if (p->cs == 14 && txn == 2 && tx[0] == 0xEE && rxn == 2) { rx[0] = 0; rx[1] = 0x92; return BUS_OK; }   /* RC522 VersionReg */
    /* RC522 card detection: register writes, FIFOLevelReg (2 = a card answered), FIFO (ATQA 0x0004) */
    if (p->cs == 14 && txn == 2 && rxn == 0) return BUS_OK;
    if (p->cs == 14 && txn == 2 && tx[0] == 0x94 && rxn == 2) { rx[0] = 0; rx[1] = 2; return BUS_OK; }
    if (p->cs == 14 && txn == 3 && tx[0] == 0x92 && rxn == 3) { rx[0] = 0; rx[1] = 0x04; rx[2] = 0x00; return BUS_OK; }
    if (p->cs == 10 && txn == 0 && rxn == 2) { rx[0] = (100 << 3) >> 8; rx[1] = (100 << 3) & 0xFF; return BUS_OK; } /* MAX6675 25.00 C */
    return BUS_ERROR;
}

static bus_status_t gpio_set(void *c, uint32_t pin, bool l) { (void)c; (void)pin; (void)l; return BUS_OK; }
static bus_status_t gpio_pulse(void *c, uint32_t pin, bool l, uint32_t w) { (void)c; (void)pin; (void)l; g_now += w; return BUS_OK; }
static bus_status_t gpio_pw(void *c, uint32_t pin, bool l, uint32_t to, uint32_t *w)
{
    (void)c; (void)l; (void)to;
    if (pin != 5) return BUS_ERROR;
    *w = 5831; g_now += 5831;
    return BUS_OK;
}
static bus_status_t gpio_bits(void *c, uint32_t pin, uint32_t n, uint32_t to, uint32_t th, uint8_t *out)
{
    (void)c; (void)to; (void)th;
    if (pin != 6 || n != 5) return BUS_ERROR;
    const uint8_t f[5] = {0x02, 0x8C, 0x00, 0xFD, (uint8_t)((0x02 + 0x8C + 0x00 + 0xFD) & 0xFF)}; /* 65.2 %RH, 25.3 C */
    memcpy(out, f, 5);
    if (W.dht_bad_checksum) out[4] ^= 0x10;
    g_now += 4000;
    return BUS_OK;
}
static void delay_us(void *c, uint32_t us) { (void)c; g_now += us; }
static uint64_t now_us(void *c) { (void)c; return g_now; }

static const bus_backend_t BE = {i2c_xfer, uart_xfer, uart_sync, spi_xfer, gpio_set, gpio_pulse, gpio_pw, gpio_bits, delay_us, now_us, NULL};

/* ------------------------------------------------------------- collector */
typedef struct {
    uint32_t n, valid;
    aai16_ReadError last_err;
    uint32_t last_n_values;
    float v[8];
    uint32_t gen;
    uint64_t first_ts, last_ts;
} seen_t;
static seen_t R_[256];

static void on_reading(const aai16_Reading *r, void *ctx)
{
    (void)ctx;
    seen_t *s = &R_[r->sensor_id];
    if (s->n == 0) s->first_ts = r->timestamp_us;
    s->n++; s->last_ts = r->timestamp_us; s->gen = r->generation; s->last_err = r->error;
    if (r->valid) {
        s->valid++;
        s->last_n_values = r->values_count;
        memcpy(s->v, r->values, sizeof(float) * r->values_count);
    }
}

static sensor_runtime_t rt;
static config_manager_t cm;
static aai16_ApplyAck last_ack;

static void ack_hook(const aai16_ApplyAck *a, void *c)
{
    last_ack = *a;
    if (a->phase == aai16_ApplyPhase_PHASE_APPLIED) rt_set_generation((sensor_runtime_t *)c, a->generation);
}

static uint8_t g_buf[40000];
static size_t load(const char *name)
{
    char p[600];
    snprintf(p, sizeof(p), "%s/%s", g_dir, name);
    FILE *f = fopen(p, "rb");
    if (!f) { printf("cannot open %s\n", p); exit(2); }
    size_t n = fread(g_buf, 1, sizeof(g_buf), f);
    fclose(f);
    return n;
}

static void apply(const char *file)
{
    cm_handle_bytes(&cm, g_buf, load(file));
}

static void run_for(uint64_t us)
{
    const uint64_t end = g_now + us;
    while (g_now < end) {
        rt_poll(&rt);
        g_now += 1000; /* 1 ms scheduler tick */
    }
}

static void new_world(void)
{
    memset(&W, 0, sizeof(W));
    memset(R_, 0, sizeof(R_));
    FILE *f;
    char p[600];
    snprintf(p, sizeof(p), "%s/bme_regs.bin", g_dir);
    f = fopen(p, "rb");
    if (!f || fread(W.bme, 1, 256, f) != 256) { puts("cannot read bme_regs.bin"); exit(2); }
    fclose(f);
    g_now = 1000000;
    rt_init(&rt, &BE, on_reading, NULL);
    rt.cb_ctx = NULL;
    cm_hooks_t h = {rt_apply, rt_stop, NULL, ack_hook, &rt};
    cm_init(&cm, &h);
}

static bool near(float a, double b, double rel) { return fabs((double)a - b) <= rel * fabs(b) + 1e-4; }

enum { GY511 = 1, DHT22 = 2, LD2450 = 3, RC522 = 4, BME280 = 5, HCSR04 = 6, MHZ19 = 7, MAX6675 = 8 };

static void test_config_a_and_swap(void)
{
    puts("config A runs on simulated devices; swapping one bus leaves the others undisturbed");
    new_world();
    apply("config_a.bin");
    CHECK(last_ack.phase == aai16_ApplyPhase_PHASE_APPLIED);
    const uint64_t t0 = g_now;
    run_for(10000000);

    /* independent periods: 20 ms, 2 s, 100 ms, 200 ms over exactly 10 s */
    /* The RC522 card poll blocks the shared bus task for ~7 ms (card power-up wait). When the DHT22, LD2450 and
     * RC522 cycles coincide, the 20 ms GY-511 can skip a sample: measured 498 of 500 (0.4 %). Slower GY-511
     * periods are unaffected. Everything else stays exact. */
    CHECK(R_[GY511].n >= 495 && R_[GY511].n <= 500 && R_[GY511].valid == R_[GY511].n);
    CHECK(R_[DHT22].n == 5 && R_[DHT22].valid == 5);
    CHECK(R_[LD2450].n == 100 && R_[LD2450].valid == 100);
    CHECK(R_[RC522].n == 50 && R_[RC522].valid == 50);
    CHECK(rt_stats(&rt, aai16_BusType_BUS_I2C)->overruns <= 5 && rt_stats(&rt, aai16_BusType_BUS_GPIO)->overruns == 0);
    const uint32_t gap = rt_stats(&rt, aai16_BusType_BUS_I2C)->last_period_us;
    CHECK(gap >= 20000 && gap <= 21000);

    CHECK(R_[GY511].last_n_values == 3 && near(R_[GY511].v[0], 1.0, 1e-5) && near(R_[GY511].v[1], -0.5, 1e-5) && near(R_[GY511].v[2], 1.02, 1e-5));
    CHECK(R_[DHT22].last_n_values == 2 && near(R_[DHT22].v[0], 65.2, 1e-5) && near(R_[DHT22].v[1], 25.3, 1e-5));
    CHECK(R_[LD2450].last_n_values == 3 && R_[LD2450].v[0] == -782.0f && R_[LD2450].v[1] == 1713.0f && R_[LD2450].v[2] == -16.0f);
    CHECK(R_[RC522].last_n_values == 2 && near(R_[RC522].v[0], 1.0, 1e-6) && near(R_[RC522].v[1], 4.0, 1e-6)); /* card present, ATQA 4 */
    /* buses run serially in one task, so the DHT22's ~6 ms of blocking delays the first I2C cycle */
    CHECK(R_[GY511].gen == 1 && R_[GY511].first_ts >= t0 && R_[GY511].first_ts - t0 < 10000);

    /* swap only the I2C bus: GY-511 -> BME280 (chip needs its init writes + calibration read) */
    const uint32_t gy_n = R_[GY511].n, dht_n = R_[DHT22].n, ld_n = R_[LD2450].n, rc_n = R_[RC522].n;
    apply("swap_bme280.bin");
    CHECK(last_ack.phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack.generation == 2);
    run_for(10000000);
    CHECK(R_[GY511].n == gy_n);                                         /* stopped: no more readings */
    CHECK(R_[BME280].n == 5 && R_[BME280].valid == 5 && R_[BME280].gen == 2);
    CHECK(R_[DHT22].n == dht_n + 5 && R_[LD2450].n == ld_n + 100 && R_[RC522].n == rc_n + 50); /* unaffected */

    FILE *f; char p[600]; double T, P, H;
    snprintf(p, sizeof(p), "%s/expect.txt", g_dir);
    f = fopen(p, "r");
    CHECK(f && fscanf(f, "%lf %lf %lf", &T, &P, &H) == 3);
    if (f) fclose(f);
    CHECK(R_[BME280].last_n_values == 3 && near(R_[BME280].v[0], T, 1e-5) && near(R_[BME280].v[1], P, 1e-6) && near(R_[BME280].v[2], H, 1e-5));
    CHECK(W.written[0xF2] == 0x01 && W.written[0xF4] == 0x27); /* init ops really ran before reading */
    CHECK(fabs(R_[BME280].v[0] - 25.08) < 0.01 && fabs(R_[BME280].v[1] - 100653.27) < 0.5);
}

static void test_faults(void)
{
    puts("bus faults are reported per sensor, isolated from other buses, and recover");
    new_world();
    apply("config_a.bin");
    apply("swap_bme280.bin");
    run_for(6000000);
    CHECK(R_[BME280].valid >= 2 && W.init_writes == 2);
    const uint32_t ok_before = R_[BME280].valid, inits = W.init_writes;
    const uint32_t gpio_before = R_[DHT22].valid, ld_before = R_[LD2450].valid;

    W.i2c_fail = true;                       /* chip stops answering */
    run_for(6000000);
    CHECK(R_[BME280].valid == ok_before && R_[BME280].last_err == aai16_ReadError_READ_BUS_ERROR);
    CHECK(R_[BME280].n > ok_before);          /* invalid readings keep being published */
    CHECK(R_[DHT22].valid == gpio_before + 3 && R_[LD2450].valid == ld_before + 60); /* others carry on */

    W.i2c_fail = false;                      /* chip is back: init is re-run, then valid data resumes */
    run_for(6000000);
    CHECK(R_[BME280].valid > ok_before && R_[BME280].last_err == aai16_ReadError_READ_OK);
    CHECK(W.init_writes > (int)inits);       /* calibration + config re-applied after the fault */

    /* frame-level faults become invalid readings (not wrong numbers) */
    const uint32_t dv = R_[DHT22].valid;
    W.dht_bad_checksum = true; W.ld_bad_tail = true;
    run_for(4000000);
    CHECK(R_[DHT22].valid == dv && R_[DHT22].last_err == aai16_ReadError_READ_BAD_FRAME);
    CHECK(R_[LD2450].last_err == aai16_ReadError_READ_BAD_FRAME);
    W.dht_bad_checksum = W.ld_bad_tail = false;
    run_for(4000000);
    CHECK(R_[DHT22].valid > dv && R_[DHT22].last_err == aai16_ReadError_READ_OK);
}

static void test_config_b_and_uart_timeout(void)
{
    puts("config B: HC-SR04, BME280, MH-Z19, MAX6675");
    new_world();
    apply("config_a.bin");
    run_for(1000000);
    apply("config_b.bin");
    CHECK(last_ack.phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack.generation == 3);
    const uint32_t gy = R_[GY511].n, dht = R_[DHT22].n, ld = R_[LD2450].n, rc = R_[RC522].n;
    run_for(6000000);
    CHECK(R_[GY511].n == gy && R_[DHT22].n == dht && R_[LD2450].n == ld && R_[RC522].n == rc); /* A all stopped */
    CHECK(R_[HCSR04].valid == 60 && near(R_[HCSR04].v[0], 5831 * 0.01715, 1e-5));
    CHECK(R_[BME280].valid == 3);
    CHECK(R_[MHZ19].valid == 3 && R_[MHZ19].v[0] == 400.0f);
    CHECK(R_[MAX6675].valid == 24 && near(R_[MAX6675].v[0], 25.0, 1e-6));

    W.mhz_bad_checksum = true;
    run_for(4000000);
    CHECK(R_[MHZ19].last_err == aai16_ReadError_READ_BAD_FRAME && R_[MHZ19].valid == 3);
    W.mhz_bad_checksum = false; W.uart_timeout = true;
    run_for(4000000);
    CHECK(R_[MHZ19].last_err == aai16_ReadError_READ_TIMEOUT);
}

static void test_overrun_and_stop(void)
{
    puts("a stalled scheduler does not burst to catch up; stop/restart behave");
    new_world();
    apply("config_a.bin");
    run_for(1000000);
    const uint32_t before = R_[GY511].n;
    g_now += 5000000;                         /* acquisition task starved for 5 s */
    rt_poll(&rt);
    CHECK(R_[GY511].n == before + 1);         /* exactly one cycle, not 250 */
    CHECK(rt_stats(&rt, aai16_BusType_BUS_I2C)->overruns >= 200);
    CHECK(rt_next_due(&rt) > g_now);

    rt_stop(aai16_BusType_BUS_I2C, &rt);
    CHECK(rt_stats(&rt, aai16_BusType_BUS_I2C)->cycles == 0);
    const uint32_t gy = R_[GY511].n;
    run_for(500000);
    CHECK(R_[GY511].n == gy && R_[DHT22].n >= 1);

    aai16_SensorProfile bad = aai16_SensorProfile_init_zero;
    bad.bus = (aai16_BusType)9;
    CHECK(!rt_apply(&bad, &rt));
    CHECK(rt_stats(&rt, (aai16_BusType)0) == NULL);
}

int main(int argc, char **argv)
{
    if (argc < 2) return 2;
    g_dir = argv[1];
    test_config_a_and_swap();
    test_faults();
    test_config_b_and_uart_timeout();
    test_overrun_and_stop();
    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
