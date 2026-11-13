/* Host tests for config_manager. Built and run by tests/config_host/run.py.
 * argv[1] = directory with fixture .bin files written by the Python encoder.
 * Hooks below are bounded test doubles for the bus engines / NVS, not firmware. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config_manager.h"
#include "pb_decode.h"
#include "pb_encode.h"

static int g_checks, g_failures;
static const char *g_dir;

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                      \
    } while (0)

/* ------------------------------------------------------------ mock hooks */

#define MAX_ACKS 16
static struct {
    aai16_ApplyAck acks[MAX_ACKS];
    int n_acks;
    int apply_calls, stop_calls, persist_calls;
    uint32_t applied_ids[32];
    uint32_t fail_apply_sensor; /* apply() returns false for this sensor id */
    bool fail_persist;
    uint8_t blob[8192];
    size_t blob_len;
} M;

static bool hook_apply(const aai16_SensorProfile *p, void *ctx)
{
    (void)ctx;
    if (M.apply_calls < 32) {
        M.applied_ids[M.apply_calls] = p->sensor_id;
    }
    M.apply_calls++;
    return p->sensor_id != M.fail_apply_sensor;
}

static void hook_stop(aai16_BusType bus, void *ctx)
{
    (void)bus;
    (void)ctx;
    M.stop_calls++;
}

static bool hook_persist(const aai16_CommittedConfig *cfg, void *ctx)
{
    (void)ctx;
    M.persist_calls++;
    if (M.fail_persist) {
        return false;
    }
    pb_ostream_t os = pb_ostream_from_buffer(M.blob, sizeof(M.blob));
    if (!pb_encode(&os, aai16_CommittedConfig_fields, cfg)) {
        return false;
    }
    M.blob_len = os.bytes_written;
    return true;
}

static void hook_ack(const aai16_ApplyAck *a, void *ctx)
{
    (void)ctx;
    if (M.n_acks < MAX_ACKS) {
        M.acks[M.n_acks] = *a;
    }
    M.n_acks++;
}

static const aai16_ApplyAck *last_ack(void) { return &M.acks[M.n_acks - 1]; }

static const cm_hooks_t HOOKS = {hook_apply, hook_stop, hook_persist, hook_ack, NULL};

/* config_manager_t is ~20 KB; keep instances static. */
static config_manager_t g_cm, g_cm2;
static aai16_ConfigApply g_req;

static void reset(config_manager_t *cm)
{
    memset(&M, 0, sizeof(M));
    cm_init(cm, &HOOKS);
}

/* ------------------------------------------------------------- fixtures */

static uint8_t g_buf[8192];

static size_t load(const char *name)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", g_dir, name);
    FILE *f = fopen(path, "rb");
    if (!f) {
        printf("  cannot open %s\n", path);
        exit(2);
    }
    size_t n = fread(g_buf, 1, sizeof(g_buf), f);
    fclose(f);
    return n;
}

static void save(const char *name, const aai16_ApplyAck *ack)
{
    uint8_t b[64];
    size_t n = 0;
    CHECK(cm_encode_ack(ack, b, sizeof(b), &n));
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", g_dir, name);
    FILE *f = fopen(path, "wb");
    CHECK(f != NULL);
    if (f) {
        fwrite(b, 1, n, f);
        fclose(f);
    }
}

/* Small valid profiles used to build requests directly in C. */
static aai16_SensorProfile prof_i2c(uint32_t id)
{
    aai16_SensorProfile p = aai16_SensorProfile_init_zero;
    p.sensor_id = id;
    p.bus = aai16_BusType_BUS_I2C;
    p.enabled = true;
    p.period_ms = 100;
    p.which_params = aai16_SensorProfile_i2c_tag;
    p.params.i2c.sda = 8;
    p.params.i2c.scl = 9;
    p.params.i2c.addr = 0x19;
    p.params.i2c.clock_hz = 400000;
    p.ops_count = 1;
    p.ops[0].type = aai16_OpType_OP_WRITE_READ;
    p.ops[0].tx.size = 1;
    p.ops[0].tx.bytes[0] = 0xA8;
    p.ops[0].read_len = 6;
    p.decode_count = 1;
    p.decode[0].width = 2;
    p.decode[0].scale = 1.0f;
    return p;
}

static aai16_SensorProfile prof_gpio(uint32_t id)
{
    aai16_SensorProfile p = aai16_SensorProfile_init_zero;
    p.sensor_id = id;
    p.bus = aai16_BusType_BUS_GPIO;
    p.enabled = true;
    p.period_ms = 100;
    p.which_params = aai16_SensorProfile_gpio_tag;
    p.params.gpio.data_pin = 6;
    p.ops_count = 1;
    p.ops[0].type = aai16_OpType_OP_GPIO_PULSE_WIDTH;
    p.ops[0].pin = 6;
    p.ops[0].level = true;
    p.ops[0].timeout_us = 1000;
    return p;
}

static aai16_SensorProfile prof_spi(uint32_t id)
{
    aai16_SensorProfile p = aai16_SensorProfile_init_zero;
    p.sensor_id = id;
    p.bus = aai16_BusType_BUS_SPI;
    p.enabled = true;
    p.period_ms = 100;
    p.which_params = aai16_SensorProfile_spi_tag;
    p.params.spi.sclk = 12;
    p.params.spi.mosi = 11;
    p.params.spi.miso = 13;
    p.params.spi.cs = 14;
    p.params.spi.clock_hz = 1000000;
    p.ops_count = 1;
    p.ops[0].type = aai16_OpType_OP_READ;
    p.ops[0].read_len = 2;
    return p;
}

static aai16_SensorProfile prof_uart(uint32_t id)
{
    aai16_SensorProfile p = aai16_SensorProfile_init_zero;
    p.sensor_id = id;
    p.bus = aai16_BusType_BUS_UART;
    p.enabled = true;
    p.period_ms = 100;
    p.which_params = aai16_SensorProfile_uart_tag;
    p.params.uart.port = 1;
    p.params.uart.tx = 17;
    p.params.uart.rx = 18;
    p.params.uart.baud = 9600;
    p.ops_count = 1;
    p.ops[0].type = aai16_OpType_OP_READ;
    p.ops[0].read_len = 9;
    return p;
}

static void req1(uint32_t gen, const aai16_SensorProfile *p)
{
    memset(&g_req, 0, sizeof(g_req));
    g_req.generation = gen;
    g_req.profiles_count = 1;
    g_req.profiles[0] = *p;
}

/* ------------------------------------------------------------------ tests */

static void test_config_a_from_python_encoder(void)
{
    puts("config A (Python-encoded) applies; C re-encode is byte-identical");
    reset(&g_cm);
    size_t n = load("config_a.bin");

    /* decode with nanopb, re-encode, compare with the Python bytes */
    aai16_ConfigApply dec = aai16_ConfigApply_init_zero;
    pb_istream_t is = pb_istream_from_buffer(g_buf, n);
    CHECK(pb_decode(&is, aai16_ConfigApply_fields, &dec));
    static uint8_t re[8192];
    pb_ostream_t os = pb_ostream_from_buffer(re, sizeof(re));
    CHECK(pb_encode(&os, aai16_ConfigApply_fields, &dec));
    CHECK(os.bytes_written == n && memcmp(re, g_buf, n) == 0);

    cm_handle_bytes(&g_cm, g_buf, n);
    CHECK(M.n_acks == 2);
    CHECK(M.acks[0].phase == aai16_ApplyPhase_PHASE_ADMITTED && M.acks[0].generation == 1);
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack()->reason == aai16_Reason_REASON_NONE);
    CHECK(g_cm.committed.generation == 1);
    CHECK(M.apply_calls == 4 && M.persist_calls == 1);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_I2C)->sensor_id == 1);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_GPIO)->sensor_id == 2);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_UART)->sensor_id == 3);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_SPI)->sensor_id == 4);
    save("ack_applied.bin", last_ack());
}

static void test_single_bus_swap_leaves_others(void)
{
    puts("swapping one bus (GY-511 -> BME280 fixture) leaves the other three untouched");
    reset(&g_cm);
    cm_handle_bytes(&g_cm, g_buf, load("config_a.bin"));
    int applies_before = M.apply_calls;
    cm_handle_bytes(&g_cm, g_buf, load("swap_i2c_bme280.bin"));
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack()->generation == 2);
    CHECK(M.apply_calls == applies_before + 1);
    CHECK(M.applied_ids[applies_before] == 5);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_I2C)->sensor_id == 5);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_GPIO)->sensor_id == 2);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_UART)->sensor_id == 3);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_SPI)->sensor_id == 4);
}

static void test_config_b_replaces_all(void)
{
    puts("config B replaces all four buses");
    reset(&g_cm);
    cm_handle_bytes(&g_cm, g_buf, load("config_a.bin"));
    cm_handle_bytes(&g_cm, g_buf, load("config_b.bin"));
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack()->generation == 3);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_GPIO)->sensor_id == 6);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_I2C)->sensor_id == 5);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_UART)->sensor_id == 7);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_SPI)->sensor_id == 8);
}

static void test_generation_rules(void)
{
    puts("stale / duplicate / zero generation rejected, state preserved");
    reset(&g_cm);
    aai16_SensorProfile p = prof_i2c(1);
    req1(5, &p);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_APPLIED);

    int calls = M.apply_calls;
    req1(5, &p);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_REJECTED && last_ack()->reason == aai16_Reason_REASON_DUPLICATE_GENERATION);
    req1(4, &p);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_STALE_GENERATION);
    req1(0, &p);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_STALE_GENERATION);
    CHECK(M.apply_calls == calls && g_cm.committed.generation == 5);
}

/* ---- table of single-profile mutations that must be rejected ---- */
typedef struct {
    const char *name;
    aai16_Reason reason;
    void (*mutate)(aai16_SensorProfile *);
} neg_case_t;

static void m_pin26(aai16_SensorProfile *p) { p->params.i2c.sda = 26; }
static void m_pin0(aai16_SensorProfile *p) { p->params.i2c.scl = 0; }
static void m_pin_same(aai16_SensorProfile *p) { p->params.i2c.scl = p->params.i2c.sda; }
static void m_addr(aai16_SensorProfile *p) { p->params.i2c.addr = 0x78; }
static void m_i2c_hz(aai16_SensorProfile *p) { p->params.i2c.clock_hz = 5000000; }
static void m_period(aai16_SensorProfile *p) { p->period_ms = 5; }
static void m_period_big(aai16_SensorProfile *p) { p->period_ms = 4000000; }
static void m_no_ops(aai16_SensorProfile *p) { p->ops_count = 0; }
static void m_read0(aai16_SensorProfile *p) { p->ops[0].read_len = 0; }
static void m_read65(aai16_SensorProfile *p) { p->ops[0].read_len = 65; }
static void m_wr_no_tx(aai16_SensorProfile *p) { p->ops[0].tx.size = 0; }
static void m_unknown_op(aai16_SensorProfile *p) { p->ops[0].type = (aai16_OpType)99; }
static void m_unspec_op(aai16_SensorProfile *p) { p->ops[0].type = aai16_OpType_OP_UNSPECIFIED; }
static void m_gpio_on_i2c(aai16_SensorProfile *p) { p->ops[0].type = aai16_OpType_OP_GPIO_SET; }
static void m_unknown_bus(aai16_SensorProfile *p) { p->bus = (aai16_BusType)9; }
static void m_bus_unspec(aai16_SensorProfile *p) { p->bus = aai16_BusType_BUS_UNSPECIFIED; }
static void m_mismatch(aai16_SensorProfile *p) { p->bus = aai16_BusType_BUS_SPI; }
static void m_no_params(aai16_SensorProfile *p) { p->which_params = 0; }
static void m_id0(aai16_SensorProfile *p) { p->sensor_id = 0; }
static void m_id256(aai16_SensorProfile *p) { p->sensor_id = 256; }
static void m_dec_range(aai16_SensorProfile *p) { p->decode[0].src_offset = 5; } /* 5+2 > 6 */
static void m_dec_width(aai16_SensorProfile *p) { p->decode[0].width = 5; }
static void m_dec_slot(aai16_SensorProfile *p) { p->decode[0].dest_reg = 32; }
static void m_dec_nan(aai16_SensorProfile *p) { p->decode[0].scale = NAN; }
static void m_dec_dup(aai16_SensorProfile *p)
{
    p->decode_count = 2;
    p->decode[1] = p->decode[0];
}
static void m_rx_total(aai16_SensorProfile *p)
{
    p->ops_count = 2; /* 6 + 60 = 66 > 64 */
    p->ops[1] = p->ops[0];
    p->ops[1].read_len = 60;
    p->decode_count = 0;
}
static void m_timeout(aai16_SensorProfile *p) { p->ops[0].timeout_us = 2000000; }
static void m_delay0(aai16_SensorProfile *p)
{
    p->ops[0].type = aai16_OpType_OP_DELAY;
    p->ops[0].delay_us = 0;
    p->decode_count = 0;
}

static const neg_case_t NEG_I2C[] = {
    {"pin 26 (flash)", aai16_Reason_REASON_FORBIDDEN_PIN, m_pin26},
    {"pin 0 (strapping)", aai16_Reason_REASON_FORBIDDEN_PIN, m_pin0},
    {"sda == scl", aai16_Reason_REASON_PIN_CONFLICT, m_pin_same},
    {"i2c addr 0x78", aai16_Reason_REASON_BAD_ADDRESS, m_addr},
    {"i2c 5 MHz", aai16_Reason_REASON_BAD_CLOCK, m_i2c_hz},
    {"period 5 ms", aai16_Reason_REASON_BAD_PERIOD, m_period},
    {"period 4,000,000 ms", aai16_Reason_REASON_BAD_PERIOD, m_period_big},
    {"no ops", aai16_Reason_REASON_NO_OPS, m_no_ops},
    {"read_len 0", aai16_Reason_REASON_BAD_LENGTH, m_read0},
    {"read_len 65", aai16_Reason_REASON_BAD_LENGTH, m_read65},
    {"write_read without tx", aai16_Reason_REASON_BAD_LENGTH, m_wr_no_tx},
    {"op type 99", aai16_Reason_REASON_UNKNOWN_OP, m_unknown_op},
    {"op type unspecified", aai16_Reason_REASON_UNKNOWN_OP, m_unspec_op},
    {"GPIO op on I2C", aai16_Reason_REASON_OP_NOT_ALLOWED_ON_BUS, m_gpio_on_i2c},
    {"bus 9", aai16_Reason_REASON_UNKNOWN_BUS, m_unknown_bus},
    {"bus unspecified", aai16_Reason_REASON_UNKNOWN_BUS, m_bus_unspec},
    {"params for wrong bus", aai16_Reason_REASON_PARAMS_MISMATCH, m_mismatch},
    {"params missing", aai16_Reason_REASON_PARAMS_MISMATCH, m_no_params},
    {"sensor id 0", aai16_Reason_REASON_BAD_SENSOR_ID, m_id0},
    {"sensor id 256", aai16_Reason_REASON_BAD_SENSOR_ID, m_id256},
    {"decode beyond raw buffer", aai16_Reason_REASON_BAD_DECODE, m_dec_range},
    {"decode width 5", aai16_Reason_REASON_BAD_DECODE, m_dec_width},
    {"decode register 32", aai16_Reason_REASON_BAD_DECODE, m_dec_slot},
    {"decode scale NaN", aai16_Reason_REASON_BAD_DECODE, m_dec_nan},
    {"decode register written twice", aai16_Reason_REASON_BAD_DECODE, m_dec_dup},
    {"total rx 66 > 64", aai16_Reason_REASON_BAD_LENGTH, m_rx_total},
    {"timeout 2 s", aai16_Reason_REASON_BAD_TIMEOUT, m_timeout},
    {"delay 0 us", aai16_Reason_REASON_BAD_DELAY, m_delay0},
};

static void test_negative_table(void)
{
    puts("invalid single-profile requests are rejected with the right reason");
    for (size_t i = 0; i < sizeof(NEG_I2C) / sizeof(NEG_I2C[0]); i++) {
        reset(&g_cm);
        aai16_SensorProfile good = prof_i2c(1);
        req1(1, &good);
        cm_handle_apply(&g_cm, &g_req); /* committed baseline at generation 1 */
        int calls = M.apply_calls, persists = M.persist_calls;

        aai16_SensorProfile bad = prof_i2c(1);
        NEG_I2C[i].mutate(&bad);
        req1(2, &bad);
        cm_handle_apply(&g_cm, &g_req);

        const aai16_ApplyAck *a = last_ack();
        bool ok = a->phase == aai16_ApplyPhase_PHASE_REJECTED && a->reason == NEG_I2C[i].reason &&
                  a->generation == 2 && g_cm.committed.generation == 1 &&
                  M.apply_calls == calls && M.persist_calls == persists;
        g_checks++;
        if (!ok) {
            g_failures++;
            printf("  FAIL [%s]: phase=%d reason=%d (want %d)\n", NEG_I2C[i].name, a->phase, a->reason,
                   NEG_I2C[i].reason);
        }
    }
}

static void test_other_bus_validation(void)
{
    puts("UART / SPI / GPIO specific checks");
    struct {
        const char *name;
        aai16_Reason reason;
        aai16_SensorProfile p;
    } cases[8];
    size_t n = 0;

    cases[n].name = "uart baud 100";
    cases[n].reason = aai16_Reason_REASON_BAD_BAUD;
    cases[n].p = prof_uart(1);
    cases[n++].p.params.uart.baud = 100;
    cases[n].name = "uart port 0 (console)";
    cases[n].reason = aai16_Reason_REASON_BAD_PORT;
    cases[n].p = prof_uart(1);
    cases[n++].p.params.uart.port = 0;
    cases[n].name = "spi mode 4";
    cases[n].reason = aai16_Reason_REASON_BAD_SPI_MODE;
    cases[n].p = prof_spi(1);
    cases[n++].p.params.spi.mode = 4;
    cases[n].name = "spi 80 MHz";
    cases[n].reason = aai16_Reason_REASON_BAD_CLOCK;
    cases[n].p = prof_spi(1);
    cases[n++].p.params.spi.clock_hz = 80000000;
    cases[n].name = "spi cs on pin 33 (PSRAM)";
    cases[n].reason = aai16_Reason_REASON_FORBIDDEN_PIN;
    cases[n].p = prof_spi(1);
    cases[n++].p.params.spi.cs = 33;
    cases[n].name = "gpio op on unconfigured pin";
    cases[n].reason = aai16_Reason_REASON_PIN_NOT_CONFIGURED;
    cases[n].p = prof_gpio(1);
    cases[n++].p.ops[0].pin = 7;
    cases[n].name = "pulse-width timeout 0";
    cases[n].reason = aai16_Reason_REASON_BAD_TIMEOUT;
    cases[n].p = prof_gpio(1);
    cases[n++].p.ops[0].timeout_us = 0;
    cases[n].name = "i2c op on gpio bus";
    cases[n].reason = aai16_Reason_REASON_OP_NOT_ALLOWED_ON_BUS;
    cases[n].p = prof_gpio(1);
    cases[n++].p.ops[0].type = aai16_OpType_OP_READ;

    for (size_t i = 0; i < n; i++) {
        reset(&g_cm);
        req1(1, &cases[i].p);
        cm_handle_apply(&g_cm, &g_req);
        bool ok = last_ack()->phase == aai16_ApplyPhase_PHASE_REJECTED && last_ack()->reason == cases[i].reason &&
                  g_cm.committed.generation == 0 && M.apply_calls == 0;
        g_checks++;
        if (!ok) {
            g_failures++;
            printf("  FAIL [%s]: reason=%d (want %d)\n", cases[i].name, last_ack()->reason, cases[i].reason);
        }
    }
}

static void test_cross_profile_rules(void)
{
    puts("duplicate id / duplicate bus / pin collisions across profiles");
    reset(&g_cm);

    /* duplicate sensor id inside one request */
    memset(&g_req, 0, sizeof(g_req));
    g_req.generation = 1;
    g_req.profiles_count = 2;
    g_req.profiles[0] = prof_i2c(1);
    g_req.profiles[1] = prof_gpio(1);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_DUPLICATE_SENSOR_ID && last_ack()->profile_index == 1);

    /* two profiles for the same bus in one request */
    g_req.profiles[1] = prof_i2c(2);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_DUPLICATE_BUS);

    /* GPIO profile that reuses an I2C pin */
    g_req.profiles[0] = prof_i2c(1);
    g_req.profiles[1] = prof_gpio(2);
    g_req.profiles[1].params.gpio.data_pin = 8;
    g_req.profiles[1].ops[0].pin = 8;
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_PIN_CONFLICT && last_ack()->detail == 8);

    /* new request collides with a profile that is ALREADY committed on another bus */
    g_req.profiles_count = 2;
    g_req.profiles[0] = prof_i2c(1);
    g_req.profiles[1] = prof_gpio(2);
    cm_handle_apply(&g_cm, &g_req); /* valid: committed at gen 1 */
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_APPLIED);
    aai16_SensorProfile clash = prof_spi(1); /* id 1 is the committed I2C sensor */
    req1(2, &clash);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_DUPLICATE_SENSOR_ID);
    aai16_SensorProfile pinclash = prof_spi(3);
    pinclash.params.spi.mosi = 6; /* GPIO profile (committed) owns pin 6 */
    req1(2, &pinclash);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_PIN_CONFLICT);
    CHECK(g_cm.committed.generation == 1);

    /* empty request */
    memset(&g_req, 0, sizeof(g_req));
    g_req.generation = 2;
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_EMPTY_REQUEST);
}

static void test_disable_bus(void)
{
    puts("enabled=false stops one bus and clears it from committed state");
    reset(&g_cm);
    memset(&g_req, 0, sizeof(g_req));
    g_req.generation = 1;
    g_req.profiles_count = 2;
    g_req.profiles[0] = prof_i2c(1);
    g_req.profiles[1] = prof_gpio(2);
    cm_handle_apply(&g_cm, &g_req);

    aai16_SensorProfile off = {0};
    off.sensor_id = 2;
    off.bus = aai16_BusType_BUS_GPIO;
    off.enabled = false;
    req1(2, &off);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_APPLIED);
    CHECK(M.stop_calls == 1);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_GPIO) == NULL);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_I2C) != NULL);
}

static void test_apply_failure_rolls_back(void)
{
    puts("engine failure on 2nd profile -> rejected, 1st bus restored, state unchanged");
    reset(&g_cm);
    aai16_SensorProfile base = prof_i2c(1);
    req1(1, &base);
    cm_handle_apply(&g_cm, &g_req);

    memset(&g_req, 0, sizeof(g_req));
    g_req.generation = 2;
    g_req.profiles_count = 2;
    g_req.profiles[0] = prof_i2c(5);
    g_req.profiles[1] = prof_gpio(2);
    M.fail_apply_sensor = 2;
    int persists = M.persist_calls;
    int calls = M.apply_calls;
    cm_handle_apply(&g_cm, &g_req);

    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_REJECTED && last_ack()->reason == aai16_Reason_REASON_APPLY_FAILED);
    CHECK(last_ack()->sensor_id == 2 && last_ack()->profile_index == 1);
    CHECK(g_cm.committed.generation == 1);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_I2C)->sensor_id == 1);
    CHECK(cm_committed_profile(&g_cm, aai16_BusType_BUS_GPIO) == NULL);
    CHECK(M.persist_calls == persists);
    /* applied 5, failed 2, then restored the old I2C profile (id 1) and stopped GPIO */
    CHECK(M.apply_calls == calls + 3 && M.applied_ids[calls + 2] == 1);
    CHECK(M.stop_calls == 1);
}

static void test_persist_failure_rolls_back(void)
{
    puts("persist failure -> rejected, hardware restored, generation not advanced");
    reset(&g_cm);
    aai16_SensorProfile base = prof_i2c(1);
    req1(1, &base);
    cm_handle_apply(&g_cm, &g_req);
    M.fail_persist = true;
    aai16_SensorProfile next = prof_i2c(5);
    req1(2, &next);
    int calls = M.apply_calls;
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_PERSIST_FAILED);
    CHECK(g_cm.committed.generation == 1 && cm_committed_profile(&g_cm, aai16_BusType_BUS_I2C)->sensor_id == 1);
    CHECK(M.apply_calls == calls + 2 && M.applied_ids[calls + 1] == 1);
    /* retry of the same generation succeeds once storage works again */
    M.fail_persist = false;
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_APPLIED && g_cm.committed.generation == 2);
}

static void test_malformed_and_oversize(void)
{
    puts("malformed / oversized wire input rejected, nothing applied");
    const char *names[] = {"truncated.bin", "garbage.bin", "too_many.bin", "tx_too_long.bin"};
    for (size_t i = 0; i < 4; i++) {
        reset(&g_cm);
        cm_handle_bytes(&g_cm, g_buf, load(names[i]));
        bool ok = M.n_acks == 1 && last_ack()->phase == aai16_ApplyPhase_PHASE_REJECTED &&
                  last_ack()->reason == aai16_Reason_REASON_MALFORMED && M.apply_calls == 0 &&
                  g_cm.committed.generation == 0;
        g_checks++;
        if (!ok) {
            g_failures++;
            printf("  FAIL [%s]: acks=%d reason=%d\n", names[i], M.n_acks, last_ack()->reason);
        }
    }
    reset(&g_cm);
    cm_handle_bytes(&g_cm, g_buf, 0); /* empty payload */
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_REJECTED && M.apply_calls == 0);

    reset(&g_cm);
    cm_handle_bytes(&g_cm, g_buf, load("bad_pin.bin"));
    CHECK(last_ack()->reason == aai16_Reason_REASON_FORBIDDEN_PIN);
    save("ack_rejected.bin", last_ack());
}

static void test_persist_restore(void)
{
    puts("committed config survives encode -> restore; corrupt/tampered blobs are refused");
    reset(&g_cm);
    cm_handle_bytes(&g_cm, g_buf, load("config_a.bin"));
    static uint8_t blob[8192];
    size_t n = 0;
    CHECK(cm_encode_committed(&g_cm, blob, sizeof(blob), &n) && n > 0);
    CHECK(n == M.blob_len && memcmp(blob, M.blob, n) == 0); /* same bytes the persist hook saw */

    reset(&g_cm2);
    CHECK(cm_restore(&g_cm2, blob, n));
    CHECK(g_cm2.committed.generation == 1 && M.apply_calls == 4);
    CHECK(cm_committed_profile(&g_cm2, aai16_BusType_BUS_SPI)->sensor_id == 4);

    /* the restored manager continues the generation sequence */
    aai16_SensorProfile p = prof_i2c(9);
    req1(1, &p);
    cm_handle_apply(&g_cm2, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_DUPLICATE_GENERATION);

    /* truncated blob */
    reset(&g_cm2);
    CHECK(!cm_restore(&g_cm2, blob, n / 2));
    CHECK(g_cm2.committed.generation == 0 && M.apply_calls == 0);
    /* empty / zero generation */
    CHECK(!cm_restore(&g_cm2, blob, 0));

    /* tamper: decode, move a pin onto a forbidden GPIO, re-encode -> refused */
    aai16_CommittedConfig cc = aai16_CommittedConfig_init_zero;
    pb_istream_t is = pb_istream_from_buffer(blob, n);
    CHECK(pb_decode(&is, aai16_CommittedConfig_fields, &cc));
    cc.profiles[0].which_params = aai16_SensorProfile_gpio_tag;
    cc.profiles[0].params.gpio.data_pin = 27;
    static uint8_t bad[8192];
    pb_ostream_t os = pb_ostream_from_buffer(bad, sizeof(bad));
    CHECK(pb_encode(&os, aai16_CommittedConfig_fields, &cc));
    reset(&g_cm2);
    CHECK(!cm_restore(&g_cm2, bad, os.bytes_written));
    CHECK(M.apply_calls == 0 && g_cm2.committed.generation == 0);

    /* wrong schema version */
    cc.schema_version = 99;
    cc.profiles[0].params.gpio.data_pin = 6;
    os = pb_ostream_from_buffer(bad, sizeof(bad));
    CHECK(pb_encode(&os, aai16_CommittedConfig_fields, &cc));
    CHECK(!cm_restore(&g_cm2, bad, os.bytes_written));

    /* a bus engine that refuses to start during restore: nothing is loaded, started buses stopped */
    reset(&g_cm2);
    M.fail_apply_sensor = 3;
    CHECK(!cm_restore(&g_cm2, blob, n));
    CHECK(g_cm2.committed.generation == 0 && M.stop_calls > 0);
}

static void test_restore_empty(void)
{
    puts("an empty committed configuration (after a clear) restores with its generation");
    reset(&g_cm);
    aai16_SensorProfile p = prof_i2c(1);
    req1(5, &p);
    cm_handle_apply(&g_cm, &g_req);
    aai16_SensorProfile off = {0};
    off.sensor_id = 1; off.bus = aai16_BusType_BUS_I2C; off.enabled = false;
    req1(6, &off);
    cm_handle_apply(&g_cm, &g_req);
    CHECK(last_ack()->phase == aai16_ApplyPhase_PHASE_APPLIED && g_cm.committed.generation == 6);
    static uint8_t blob[512];
    size_t n = 0;
    CHECK(cm_encode_committed(&g_cm, blob, sizeof(blob), &n));
    reset(&g_cm2);
    CHECK(cm_restore(&g_cm2, blob, n) && g_cm2.committed.generation == 6 && M.apply_calls == 0);
    req1(6, &p);
    cm_handle_apply(&g_cm2, &g_req);
    CHECK(last_ack()->reason == aai16_Reason_REASON_DUPLICATE_GENERATION); /* the generation survived the reboot */
}

static void test_ack_size(void)
{
    puts("ACK stays small");
    aai16_ApplyAck a = aai16_ApplyAck_init_zero;
    a.generation = 0xFFFFFFFFu;
    a.phase = aai16_ApplyPhase_PHASE_REJECTED;
    a.reason = aai16_Reason_REASON_MALFORMED;
    a.sensor_id = 255;
    a.profile_index = 3;
    a.detail = 0xFFFFFFFFu;
    uint8_t b[64];
    size_t n = 0;
    CHECK(cm_encode_ack(&a, b, sizeof(b), &n) && n <= aai16_ApplyAck_size);
    CHECK(!cm_encode_ack(&a, b, 4, &n)); /* too-small buffer is an error, not an overrun */
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        puts("usage: test_config <fixture-dir>");
        return 2;
    }
    g_dir = argv[1];

    test_config_a_from_python_encoder();
    test_single_bus_swap_leaves_others();
    test_config_b_replaces_all();
    test_generation_rules();
    test_negative_table();
    test_other_bus_validation();
    test_cross_profile_rules();
    test_disable_bus();
    test_apply_failure_rolls_back();
    test_persist_failure_rolls_back();
    test_malformed_and_oversize();
    test_persist_restore();
    test_restore_empty();
    test_ack_size();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
