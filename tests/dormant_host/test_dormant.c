/* Dormant sensors + RACT_SENSOR_START / RACT_SENSOR_STOP on a full node with simulated devices.
 * Doubles: GY-511 (I2C 0x19), DHT22 (GPIO pin 6), the NVS blobs, the MQTT sink. Everything else is the real node. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "node_core.h"
#include "pb_decode.h"
#include "pb_encode.h"

static int g_checks, g_fail;
static const char *g_dir;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fail++; printf("  FAIL line %d: %s\n", __LINE__, #cond); } } while (0)

static uint8_t g_buf[40000];
static size_t load(const char *name)
{
    char p[600];
    snprintf(p, sizeof(p), "%s/%s", g_dir, name);
    FILE *f = fopen(p, "rb");
    if (!f) { printf("cannot open %s\n", p); exit(2); }
    const size_t n = fread(g_buf, 1, sizeof(g_buf), f);
    fclose(f);
    return n;
}

static uint64_t g_now;
static bus_status_t i2c_xfer(void *c, const aai16_I2cParams *p, const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn, uint32_t to)
{
    (void)c; (void)to;
    if (p->addr == 0x19) {
        if (txn == 2) return BUS_OK;
        if (txn == 1 && tx[0] == 0xA8 && rxn == 6) { memset(rx, 0, 6); return BUS_OK; }
    }
    return BUS_ERROR;
}
static bus_status_t uart_xfer(void *c, const aai16_UartParams *p, const uint8_t *tx, size_t a, uint8_t *rx, size_t b, uint32_t t) { (void)c; (void)p; (void)tx; (void)a; (void)rx; (void)b; (void)t; return BUS_ERROR; }
static bus_status_t uart_sync(void *c, const aai16_UartParams *p, const uint8_t *pt, size_t a, const uint8_t *tl, size_t b, uint8_t *rx, size_t l, uint32_t t) { (void)c; (void)p; (void)pt; (void)a; (void)tl; (void)b; (void)rx; (void)l; (void)t; return BUS_ERROR; }
static bus_status_t spi_xfer(void *c, const aai16_SpiParams *p, const uint8_t *tx, size_t a, uint8_t *rx, size_t b, uint32_t t) { (void)c; (void)p; (void)tx; (void)a; (void)rx; (void)b; (void)t; return BUS_ERROR; }
static bus_status_t gpio_set(void *c, uint32_t pin, bool l) { (void)c; (void)pin; (void)l; return BUS_OK; }
static bus_status_t gpio_pulse(void *c, uint32_t pin, bool l, uint32_t w) { (void)c; (void)pin; (void)l; g_now += w; return BUS_OK; }
static bus_status_t gpio_pw(void *c, uint32_t pin, bool l, uint32_t to, uint32_t *w) { (void)c; (void)pin; (void)l; (void)to; (void)w; return BUS_ERROR; }
static bus_status_t gpio_bits(void *c, uint32_t pin, uint32_t n, uint32_t to, uint32_t th, uint8_t *out)
{
    (void)c; (void)to; (void)th;
    if (pin != 6 || n != 5) return BUS_ERROR;
    out[0] = 0x01; out[1] = 0xF4; out[2] = 0x00; out[3] = 0xFA; out[4] = 0xEF; /* 50.0 %, 25.0 C */
    g_now += 4000;
    return BUS_OK;
}
static void delay_us(void *c, uint32_t us) { (void)c; g_now += us; }
static uint64_t now_us(void *c) { (void)c; return g_now; }

typedef struct { node_out_t kind; uint8_t data[NODE_OUT_BUF]; size_t len; } msg_t;
static msg_t LOG_[30000];
static int NLOG;
static bool publish(void *c, node_out_t kind, const char *target, const uint8_t *buf, size_t len)
{
    (void)c; (void)target;
    if (NLOG >= 30000) return false;
    msg_t *m = &LOG_[NLOG++];
    m->kind = kind; m->len = len; memcpy(m->data, buf, len);
    return true;
}
static void reset_log(void) { NLOG = 0; }
static const msg_t *last(node_out_t k) { for (int i = NLOG - 1; i >= 0; i--) if (LOG_[i].kind == k) return &LOG_[i]; return NULL; }
#define DECODE(m, T, out) do { memset(&(out), 0, sizeof(out)); pb_istream_t _s = pb_istream_from_buffer((m)->data, (m)->len); CHECK(pb_decode(&_s, T##_fields, &(out))); } while (0)

static aai16_ApplyAck last_ack(node_out_t k)
{
    aai16_ApplyAck a = aai16_ApplyAck_init_zero;
    const msg_t *m = last(k);
    CHECK(m != NULL);
    if (m) DECODE(m, aai16_ApplyAck, a);
    return a;
}
static int readings_of(uint32_t id)
{
    int c = 0;
    for (int i = 0; i < NLOG; i++) {
        if (LOG_[i].kind != OUT_READING) continue;
        aai16_Reading r; DECODE(&LOG_[i], aai16_Reading, r);
        c += r.sensor_id == id;
    }
    return c;
}
static bool saw_note(const char *sub)
{
    for (int i = 0; i < NLOG; i++) {
        if (LOG_[i].kind != OUT_NOTIFICATION) continue;
        aai16_Notification n; DECODE(&LOG_[i], aai16_Notification, n);
        if (strstr(n.text, sub)) return true;
    }
    return false;
}

static struct { uint8_t cfg[NODE_BLOB_MAX]; size_t cfg_len; uint8_t state[NODE_BLOB_MAX]; size_t state_len; } P;
static bool cfg_save(void *c, const uint8_t *b, size_t n) { (void)c; if (n > sizeof(P.cfg)) return false; memcpy(P.cfg, b, n); P.cfg_len = n; return true; }
static bool cfg_load(void *c, uint8_t *b, size_t cap, size_t *n) { (void)c; if (!P.cfg_len || P.cfg_len > cap) return false; memcpy(b, P.cfg, P.cfg_len); *n = P.cfg_len; return true; }
static bool state_save(void *c, const uint8_t *b, size_t n) { (void)c; if (n > sizeof(P.state)) return false; memcpy(P.state, b, n); P.state_len = n; return true; }
static bool state_load(void *c, uint8_t *b, size_t cap, size_t *n) { (void)c; if (!P.state_len || P.state_len > cap) return false; memcpy(b, P.state, P.state_len); *n = P.state_len; return true; }
static aai16_ModelReason ml_load(void *c, const ml_package_t *pkg) { (void)c; (void)pkg; return aai16_ModelReason_MODEL_REASON_NONE; }
static bool ml_invoke(void *c, const int8_t *in, uint32_t a, int8_t *out, uint32_t b, uint32_t *us) { (void)c; (void)in; (void)a; (void)out; (void)b; (void)us; return false; }
static uint32_t heap(void *c) { (void)c; return 100000; }

/* model store doubles (no model is used here, but the node wires the store up at init) */
#define BANK_SZ 65536
static uint8_t g_bank[2][BANK_SZ];
static bool f_erase(void *c, uint8_t b, size_t size) { (void)c; (void)size; memset(g_bank[b], 0xFF, BANK_SZ); return true; }
static bool f_write(void *c, uint8_t b, uint32_t off, const uint8_t *d, size_t n) { (void)c; if (off + n > BANK_SZ) return false; memcpy(g_bank[b] + off, d, n); return true; }
static const uint8_t *f_map(void *c, uint8_t b, size_t len) { (void)c; return len <= BANK_SZ ? g_bank[b] : NULL; }
static bool f_load(void *c, model_meta_t *m) { (void)c; (void)m; return false; }
static bool f_save(void *c, const model_meta_t *m) { (void)c; (void)m; return true; }

static node_platform_t make_pf(void)
{
    node_platform_t pf = {0};
    pf.node_id = "nodeW";
    pf.store = (model_store_ops_t){f_erase, f_write, f_map, f_load, f_save, NULL, BANK_SZ};
    pf.bus = (bus_backend_t){i2c_xfer, uart_xfer, uart_sync, spi_xfer, gpio_set, gpio_pulse, gpio_pw, gpio_bits, delay_us, now_us, NULL};
    pf.publish = publish; pf.persist_config = cfg_save; pf.load_config = cfg_load;
    pf.persist_state = state_save; pf.load_state = state_load;
    pf.ml_load = ml_load; pf.ml_invoke = ml_invoke; pf.now_us = now_us; pf.free_heap = heap;
    return pf;
}

static node_t N;
static void run_for(uint64_t us)
{
    const uint64_t end = g_now + us;
    while (g_now < end) { node_tick(&N); g_now += 1000; }
}
static void feed(uint32_t sid, float v)
{
    aai16_Reading r = aai16_Reading_init_zero;
    r.sensor_id = sid; r.valid = true; r.values_count = 1; r.values[0] = v; r.timestamp_us = g_now;
    node_feed_reading(&N, &r);
}
static bool shows(const char *sub)
{
    char out[400];
    node_repl_line(&N, "SHOW", out, sizeof(out));
    return strstr(out, sub) != NULL;
}
static void send(node_in_t kind, const char *file) { node_on_message(&N, kind, NULL, g_buf, load(file)); }
static void tap(void) { feed(4, 0.0f); run_for(1100000); feed(4, 1.0f); } /* release, wait out the 1 s cooldown, touch */

#define BAD(r) aai16_Reason_REASON_##r

static void scenario(void)
{
    node_platform_t pf = make_pf();
    memset(&P, 0, sizeof(P));
    g_now = 1000000;
    node_init(&N, &pf);
    node_boot(&N);

    puts("1. config with a dormant GY-511: stored, persisted, NOT running");
    send(IN_CONFIG, "cfg_dormant.bin");
    CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack(OUT_APPLY_ACK).generation == 1);
    CHECK(cm_dormant_by_id(&N.cm, 1) != NULL && cm_committed_profile(&N.cm, aai16_BusType_BUS_I2C) == NULL);
    CHECK(cm_dormant_by_id(&N.cm, 2) == NULL);                                   /* the DHT22 is a running sensor */
    CHECK(P.cfg_len > 0 && P.cfg_len < NODE_BLOB_MAX);
    reset_log();
    run_for(2100000);
    CHECK(readings_of(1) == 0 && readings_of(2) >= 1);
    CHECK(shows("sensors=[2]"));

    puts("2. invalid dormant configurations are rejected and change nothing");
    send(IN_CONFIG, "cfg_both.bin");
    CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_REJECTED && last_ack(OUT_APPLY_ACK).reason == BAD(BAD_DORMANT));
    send(IN_CONFIG, "cfg_pinclash.bin");
    CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_REJECTED && last_ack(OUT_APPLY_ACK).reason == BAD(PIN_CONFLICT));
    CHECK(N.cm.committed.generation == 1 && cm_dormant_by_id(&N.cm, 1) != NULL && shows("sensors=[2]"));

    puts("3. rules: valid set accepted; bad sensor ids / durations rejected");
    send(IN_RULES, "rules_ok.bin");
    CHECK(last_ack(OUT_RULE_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED && N.re.count == 5);
    const char *bad[4] = {"rules_bad_zero.bin", "rules_bad_virtual.bin", "rules_bad_long.bin", "rules_bad_stopdur.bin"};
    for (int i = 0; i < 4; i++) {
        send(IN_RULES, bad[i]);
        CHECK(last_ack(OUT_RULE_ACK).phase == aai16_ApplyPhase_PHASE_REJECTED && last_ack(OUT_RULE_ACK).reason == BAD(BAD_ACTION));
    }
    CHECK(N.re.count == 5 && N.re.generation == 1);                              /* the good set is still in force */

    puts("4. a tap wakes the sensor for 5 s, then it goes back to sleep");
    reset_log();
    feed(4, 1.0f);
    run_for(1000000);
    CHECK(readings_of(1) >= 45 && readings_of(1) <= 51);
    CHECK(saw_note("sensor 1 started (timed)") && shows("sensors=[2,1]"));
    run_for(4200000);
    CHECK(saw_note("sensor 1 stopped (timer)") && shows("sensors=[2]"));
    const int total = readings_of(1);
    CHECK(total >= 240 && total <= 256);
    run_for(1000000);
    CHECK(readings_of(1) == total);                                              /* asleep again */
    CHECK(cm_dormant_by_id(&N.cm, 1) != NULL);                                   /* ... and still stored */

    puts("5. a second tap while awake re-arms the timer");
    reset_log();
    tap();
    run_for(3000000);
    tap();                                                                       /* 4.1 s after the first: the first timer would end at 5 s */
    run_for(4500000);
    CHECK(shows("sensors=[2,1]"));                                               /* 8.6 s after the first tap: only the re-arm keeps it awake */
    run_for(1000000);
    CHECK(shows("sensors=[2]"));

    puts("6. a rule can stop a woken sensor early");
    tap();
    run_for(1000000);
    CHECK(shows("sensors=[2,1]"));
    feed(5, 1.0f);
    CHECK(shows("sensors=[2]") && saw_note("sensor 1 stopped"));

    puts("7. refusals: unknown sensor, stopping a configured sensor, starting a running one");
    const uint32_t e0 = N.re.errors;
    feed(6, 1.0f);
    CHECK(N.re.errors == e0 + 1);
    feed(7, 1.0f);
    CHECK(N.re.errors == e0 + 2 && shows("sensors=[2]"));                        /* the DHT22 keeps running */
    feed(8, 1.0f);
    CHECK(N.re.errors == e0 + 3 && shows("sensors=[2]"));

    puts("8. reboot while awake: the sensor stays asleep, the stored profile and the rules come back");
    tap();
    run_for(500000);
    CHECK(shows("sensors=[2,1]"));
    memset(&N, 0, sizeof(N));
    node_init(&N, &pf);
    node_boot(&N);
    reset_log();
    run_for(1100000);
    CHECK(readings_of(1) == 0 && shows("sensors=[2]"));
    CHECK(cm_dormant_by_id(&N.cm, 1) != NULL && N.re.count == 5);
    feed(4, 1.0f);
    run_for(1000000);
    CHECK(readings_of(1) >= 45 && shows("sensors=[2,1]"));
    run_for(5500000);
    CHECK(shows("sensors=[2]"));

    puts("9. a new configuration for the bus replaces the dormant profile (and the sensor then runs for good)");
    send(IN_CONFIG, "cfg_enable_gy.bin");
    CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack(OUT_APPLY_ACK).generation == 10);
    CHECK(cm_dormant_by_id(&N.cm, 1) == NULL && cm_committed_profile(&N.cm, aai16_BusType_BUS_I2C) != NULL);
    reset_log();
    run_for(6000000);
    CHECK(readings_of(1) > 280 && shows("sensors=[2,1]"));                       /* no timer for a configured sensor */
    const uint32_t e1 = N.re.errors;
    feed(4, 0.0f); run_for(1100000); feed(4, 1.0f);
    CHECK(N.re.errors == e1 + 1);                                                /* START refused: it is not dormant */
    feed(5, 1.0f);
    CHECK(N.re.errors == e1 + 2 && shows("sensors=[2,1]"));                      /* STOP refused: a configuration started it */

    puts("10. disabling the bus removes the sensor completely; dormant -> CLEAR wipes everything");
    send(IN_CONFIG, "cfg_off_i2c.bin");
    CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED && shows("sensors=[2]"));
    CHECK(cm_dormant_by_id(&N.cm, 1) == NULL);
    feed(4, 0.0f); run_for(1100000);
    const uint32_t e2 = N.re.errors;
    feed(4, 1.0f);
    CHECK(N.re.errors == e2 + 1 && shows("sensors=[2]"));                        /* nothing to start any more */
    send(IN_CONFIG, "cfg_dormant_again.bin");
    CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED && cm_dormant_by_id(&N.cm, 1) != NULL);
    {
        aai16_NodeCommand cmd = aai16_NodeCommand_init_zero;
        cmd.cmd = aai16_RemoteCmd_RCMD_CLEAR;
        uint8_t b[16];
        pb_ostream_t os = pb_ostream_from_buffer(b, sizeof(b));
        CHECK(pb_encode(&os, aai16_NodeCommand_fields, &cmd));
        node_on_message(&N, IN_COMMAND, NULL, b, os.bytes_written);
    }
    CHECK(cm_dormant_by_id(&N.cm, 1) == NULL && shows("sensors=[]") && N.re.count == 0);
    memset(&N, 0, sizeof(N));                                                    /* the cleared state is what comes back */
    node_init(&N, &pf);
    node_boot(&N);
    CHECK(cm_dormant_by_id(&N.cm, 1) == NULL && shows("sensors=[]"));

    puts("11. boundary rules accepted (longest timer, highest physical id)");
    send(IN_RULES, "rules_edge_ok.bin");
    CHECK(last_ack(OUT_RULE_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED && N.re.count == 2);
}

int main(int argc, char **argv)
{
    if (argc < 2) return 2;
    g_dir = argv[1];
    setvbuf(stdout, NULL, _IONBF, 0);
    scenario();
    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
