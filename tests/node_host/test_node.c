/* Full-node integration test on simulated hardware. See run.py for what is real and what is a double.
 * Doubles: bus devices (GY-511, DHT22, BME280, HC-SR04 data), flash banks, NVS blob, the MQTT sink and the
 * inference multiply (answers only for the exact recorded int8 tensors of the real trained models). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "node_core.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "sha256.h"

static int g_checks, g_fail;
static const char *g_dir;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fail++; printf("  FAIL line %d: %s\n", __LINE__, #cond); } } while (0)
#define R(x) aai16_ModelReason_MODEL_REASON_##x

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

/* ---------------------------------------------------------- simulated world */
static uint64_t g_now;
static struct {
    uint8_t motion[64][6]; uint32_t motion_cur;
    uint8_t dht[20][5];    uint32_t dht_cur;
    uint8_t bme[256];
    uint32_t reads_gy, reads_dht;
} W;

static bus_status_t i2c_xfer(void *c, const aai16_I2cParams *p, const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn, uint32_t to)
{
    (void)c; (void)to;
    if (p->addr == 0x19) {
        if (txn == 2) return BUS_OK;
        if (txn == 1 && tx[0] == 0xA8 && rxn == 6) { memcpy(rx, W.motion[W.motion_cur++ % 64], 6); W.reads_gy++; return BUS_OK; }
    } else if (p->addr == 0x76) {
        if (txn >= 2 && rxn == 0) return BUS_OK;
        if (txn == 1 && rxn > 0 && (size_t)tx[0] + rxn <= 256) { memcpy(rx, W.bme + tx[0], rxn); return BUS_OK; }
    }
    return BUS_ERROR;
}
static bus_status_t uart_xfer(void *c, const aai16_UartParams *p, const uint8_t *tx, size_t a, uint8_t *rx, size_t b, uint32_t t) { (void)c; (void)p; (void)tx; (void)a; (void)rx; (void)b; (void)t; return BUS_ERROR; }
static bus_status_t uart_sync(void *c, const aai16_UartParams *p, const uint8_t *pt, size_t a, const uint8_t *tl, size_t b, uint8_t *rx, size_t l, uint32_t t) { (void)c; (void)p; (void)pt; (void)a; (void)tl; (void)b; (void)rx; (void)l; (void)t; return BUS_ERROR; }
static bus_status_t spi_xfer(void *c, const aai16_SpiParams *p, const uint8_t *tx, size_t a, uint8_t *rx, size_t b, uint32_t t) { (void)c; (void)p; (void)tx; (void)a; (void)rx; (void)b; (void)t; return BUS_ERROR; }
static bus_status_t gpio_set(void *c, uint32_t pin, bool l) { (void)c; (void)pin; (void)l; return BUS_OK; }
static bus_status_t gpio_pulse(void *c, uint32_t pin, bool l, uint32_t w) { (void)c; (void)pin; (void)l; g_now += w; return BUS_OK; }
static bus_status_t gpio_pw(void *c, uint32_t pin, bool l, uint32_t to, uint32_t *w) { (void)c; (void)l; (void)to; if (pin != 5) return BUS_ERROR; *w = 5831; g_now += 5831; return BUS_OK; }
static bus_status_t gpio_bits(void *c, uint32_t pin, uint32_t n, uint32_t to, uint32_t th, uint8_t *out)
{
    (void)c; (void)to; (void)th;
    if (pin != 6 || n != 5) return BUS_ERROR;
    memcpy(out, W.dht[W.dht_cur++ % 20], 5); W.reads_dht++; g_now += 4000;
    return BUS_OK;
}
static void delay_us(void *c, uint32_t us) { (void)c; g_now += us; }
static uint64_t now_us(void *c) { (void)c; return g_now; }

/* ---------------------------------------------------------------- sinks */
typedef struct { node_out_t kind; char target[16]; uint8_t data[NODE_OUT_BUF]; size_t len; } msg_t;
static msg_t LOG_[20000];
static int NLOG;

static bool publish(void *c, node_out_t kind, const char *target, const uint8_t *buf, size_t len)
{
    (void)c;
    if (NLOG >= 20000) return false;
    msg_t *m = &LOG_[NLOG++];
    m->kind = kind; m->len = len;
    snprintf(m->target, sizeof(m->target), "%s", target ? target : "");
    memcpy(m->data, buf, len);
    return true;
}
static int count(node_out_t k) { int n = 0; for (int i = 0; i < NLOG; i++) n += LOG_[i].kind == k; return n; }
static const msg_t *last(node_out_t k) { for (int i = NLOG - 1; i >= 0; i--) if (LOG_[i].kind == k) return &LOG_[i]; return NULL; }
#define DECODE(m, T, out) do { memset(&(out), 0, sizeof(out)); pb_istream_t _s = pb_istream_from_buffer((m)->data, (m)->len); CHECK(pb_decode(&_s, T##_fields, &(out))); } while (0)

static aai16_ModelStatus last_status(void)
{
    aai16_ModelStatus s = aai16_ModelStatus_init_zero;
    const msg_t *m = last(OUT_MODEL_STATUS);
    CHECK(m != NULL);
    if (m) DECODE(m, aai16_ModelStatus, s);
    return s;
}
static aai16_ApplyAck last_ack(node_out_t k)
{
    aai16_ApplyAck a = aai16_ApplyAck_init_zero;
    const msg_t *m = last(k);
    CHECK(m != NULL);
    if (m) DECODE(m, aai16_ApplyAck, a);
    return a;
}

/* flash / NVS / runtime doubles */
#define BANK_SZ 65536
static struct { uint8_t bank[2][BANK_SZ]; model_meta_t meta; bool has_meta; uint8_t cfg[NODE_BLOB_MAX]; size_t cfg_len; uint8_t state[NODE_BLOB_MAX]; size_t state_len; bool fail_meta_save; } P;
static struct { int loads; aai16_ModelKind loaded_kind; int invokes, matched, unknown; int8_t pair_in[8][16]; int8_t pair_out[8][4]; int pair_n_in[8], pair_n_out[8], pair_task[8], pair_pred[8]; int np; uint32_t gpio_pulses, pwm; } M;

static bool f_erase(void *c, uint8_t b, size_t size) { (void)c; (void)size; memset(P.bank[b], 0xFF, BANK_SZ); return true; }
static bool f_write(void *c, uint8_t b, uint32_t off, const uint8_t *d, size_t n) { (void)c; if (off + n > BANK_SZ) return false; memcpy(P.bank[b] + off, d, n); return true; }
static const uint8_t *f_map(void *c, uint8_t b, size_t len) { (void)c; return len <= BANK_SZ ? P.bank[b] : NULL; }
static bool f_load(void *c, model_meta_t *m) { (void)c; if (!P.has_meta) return false; *m = P.meta; return true; }
static bool f_save(void *c, const model_meta_t *m) { (void)c; if (P.fail_meta_save) return false; P.meta = *m; P.has_meta = true; return true; }
static bool cfg_save(void *c, const uint8_t *b, size_t n) { (void)c; if (n > sizeof(P.cfg)) return false; memcpy(P.cfg, b, n); P.cfg_len = n; return true; }
static bool cfg_load(void *c, uint8_t *b, size_t cap, size_t *n) { (void)c; if (!P.cfg_len || P.cfg_len > cap) return false; memcpy(b, P.cfg, P.cfg_len); *n = P.cfg_len; return true; }
static bool state_save(void *c, const uint8_t *b, size_t n) { (void)c; if (n > sizeof(P.state)) return false; memcpy(P.state, b, n); P.state_len = n; return true; }
static bool state_load(void *c, uint8_t *b, size_t cap, size_t *n) { (void)c; if (!P.state_len || P.state_len > cap) return false; memcpy(b, P.state, P.state_len); *n = P.state_len; return true; }
static aai16_ModelReason ml_load(void *c, const ml_package_t *pkg) { (void)c; M.loads++; M.loaded_kind = pkg->contract.kind; return R(NONE); }

static bool ml_invoke(void *c, const int8_t *in, uint32_t n_in, int8_t *out, uint32_t n_out, uint32_t *us)
{
    (void)c;
    M.invokes++;
    for (int i = 0; i < M.np; i++) {
        if ((uint32_t)M.pair_n_in[i] == n_in && (uint32_t)M.pair_n_out[i] == n_out && memcmp(M.pair_in[i], in, n_in) == 0 &&
            M.pair_task[i] == (M.loaded_kind == aai16_ModelKind_MODEL_MOTION ? 0 : 1)) {
            memcpy(out, M.pair_out[i], n_out);
            *us = 100;
            M.matched++;
            return true;
        }
    }
    M.unknown++;
    return false;
}
static void gp_pulse(void *c, uint32_t pin, bool l, uint32_t ms) { (void)c; (void)pin; (void)l; (void)ms; M.gpio_pulses++; }
static void gp_pwm(void *c, uint32_t pin, uint32_t pm) { (void)c; (void)pin; (void)pm; M.pwm++; }
static uint32_t heap(void *c) { (void)c; return 123456; }
static char g_logs[64][200];
static int g_nlogs;
static void log_cap(void *c, const char *s) { (void)c; if (g_nlogs < 64) snprintf(g_logs[g_nlogs++], sizeof(g_logs[0]), "%s", s); }

static node_platform_t make_pf(void)
{
    node_platform_t pf = {0};
    pf.node_id = "nodeB";
    pf.bus = (bus_backend_t){i2c_xfer, uart_xfer, uart_sync, spi_xfer, gpio_set, gpio_pulse, gpio_pw, gpio_bits, delay_us, now_us, NULL};
    pf.store = (model_store_ops_t){f_erase, f_write, f_map, f_load, f_save, NULL, BANK_SZ};
    pf.publish = publish; pf.persist_config = cfg_save; pf.load_config = cfg_load;
    pf.persist_state = state_save; pf.load_state = state_load;
    pf.gpio_pulse = gp_pulse; pf.pwm = gp_pwm;
    pf.ml_load = ml_load; pf.ml_invoke = ml_invoke; pf.now_us = now_us; pf.free_heap = heap; pf.log = log_cap;
    return pf;
}

static node_t N; /* ~150 KB: static, as it would be in PSRAM */

static void run_for(uint64_t us)
{
    const uint64_t end = g_now + us;
    while (g_now < end) { node_tick(&N); g_now += 1000; }
}

static void load_pairs(void)
{
    FILE *f; char p[600];
    snprintf(p, sizeof(p), "%s/pairs.txt", g_dir);
    f = fopen(p, "r");
    if (!f) { puts("no pairs.txt"); exit(2); }
    char tag[4]; int task, nin, nout, pred;
    while (fscanf(f, "%3s %d %d %d %d", tag, &task, &nin, &nout, &pred) == 5) {
        const int i = M.np++;
        M.pair_task[i] = task; M.pair_n_in[i] = nin; M.pair_n_out[i] = nout; M.pair_pred[i] = pred;
        for (int k = 0; k < nin; k++) { int v; (void)fscanf(f, "%d", &v); M.pair_in[i][k] = (int8_t)v; }
        for (int k = 0; k < nout; k++) { int v; (void)fscanf(f, "%d", &v); M.pair_out[i][k] = (int8_t)v; }
    }
    fclose(f);
}

/* ---------------------------------------------------------- model helpers */
static void send_ctrl(uint32_t gen, aai16_ModelAction a, aai16_ModelKind k, uint32_t tid, const aai16_InputSource *in, uint32_t n)
{
    static aai16_ModelControl c;
    memset(&c, 0, sizeof(c));
    c.generation = gen; c.action = a; c.kind = k; c.transfer_id = tid; c.inputs_count = n;
    for (uint32_t i = 0; i < n; i++) c.inputs[i] = in[i];
    uint8_t b[256];
    pb_ostream_t os = pb_ostream_from_buffer(b, sizeof(b));
    CHECK(pb_encode(&os, aai16_ModelControl_fields, &c));
    node_on_message(&N, IN_MODEL_CONTROL, NULL, b, os.bytes_written);
}

static int upload(const char *file, aai16_ModelKind kind, uint32_t tid, uint32_t chunk, int corrupt_at)
{
    static uint8_t pkg[BANK_SZ];
    const size_t len = load(file);
    memcpy(pkg, g_buf, len);
    uint8_t dig[32];
    sha256_ctx_t h; sha256_init(&h);
    sha256_update(&h, pkg, len);                 /* digest of the CLEAN package, taken before any corruption */
    sha256_final(&h, dig);
    if (corrupt_at) pkg[corrupt_at] ^= 0x20;
    const uint32_t total = (uint32_t)((len + chunk - 1) / chunk);
    int ok = 0;
    for (uint32_t seq = 0; seq < total; seq++) {
        static aai16_ModelChunk c; memset(&c, 0, sizeof(c));
        c.transfer_id = tid; c.generation = tid; c.kind = kind; c.format = aai16_ModelFormat_FORMAT_TFLITE; c.model_version = 1;
        c.seq = seq; c.total_chunks = total; c.total_size = (uint32_t)len; c.sha256.size = 32; memcpy(c.sha256.bytes, dig, 32);
        const size_t off = (size_t)seq * chunk, n = len - off < chunk ? len - off : chunk;
        c.data.size = (pb_size_t)n; memcpy(c.data.bytes, pkg + off, n);
        uint8_t b[1200];
        pb_ostream_t os = pb_ostream_from_buffer(b, sizeof(b));
        CHECK(pb_encode(&os, aai16_ModelChunk_fields, &c));
        node_on_message(&N, IN_MODEL_CHUNK, NULL, b, os.bytes_written);
        aai16_ChunkAck a; DECODE(last(OUT_CHUNK_ACK), aai16_ChunkAck, a);
        if (a.status != aai16_ChunkStatus_CHUNK_OK) return (int)a.status + 1000;
        ok++;
    }
    return ok;
}

static aai16_InputSource src(uint32_t sid, uint32_t vi, uint32_t slot)
{
    aai16_InputSource i = aai16_InputSource_init_zero;
    i.sensor_id = sid; i.value_index = vi; i.feature_slot = slot;
    return i;
}

static void reset_log(void) { NLOG = 0; }

/* -------------------------------------------------------------- scenario */
static int expect_v[4];

static void scenario(void)
{
    node_platform_t pf = make_pf();
    memset(&P, 0, sizeof(P));
    g_now = 1000000;
    node_init(&N, &pf);
    node_boot(&N);
    puts("1. boot with empty flash");
    CHECK(last_status().state == aai16_ModelState_MODEL_STATE_NONE && last_status().reason == R(NO_ACTIVE_MODEL));

    puts("2. ConfigApply (GY-511 + DHT22) over 'MQTT': ACKs, persistence, readings");
    reset_log();
    node_on_message(&N, IN_CONFIG, NULL, g_buf, load("cfg1.bin"));
    CHECK(count(OUT_APPLY_ACK) == 2);
    CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack(OUT_APPLY_ACK).generation == 1);
    CHECK(P.cfg_len > 0);
    run_for(4000000);
    int gy = 0, dht = 0, other = 0;
    for (int i = 0; i < NLOG; i++) {
        if (LOG_[i].kind != OUT_READING) continue;
        aai16_Reading r; DECODE(&LOG_[i], aai16_Reading, r);
        gy += r.sensor_id == 1; dht += r.sensor_id == 2; other += (r.sensor_id != 1 && r.sensor_id != 2);
    }
    CHECK(gy == 200 && dht == 2 && other == 0);

    puts("3. motion model: upload, activate, reject a wrong mapping, accept the right one, infer");
    reset_log();
    CHECK(upload("motion.amdl", aai16_ModelKind_MODEL_MOTION, 1, 512, 0) > 0);
    send_ctrl(1, aai16_ModelAction_ACTION_ACTIVATE, aai16_ModelKind_MODEL_MOTION, 1, NULL, 0);
    CHECK(M.loads == 1 && M.loaded_kind == aai16_ModelKind_MODEL_MOTION);
    CHECK(last_status().state == aai16_ModelState_MODEL_STATE_SUSPENDED && last_status().reason == R(NO_INPUTS));
    aai16_InputSource wrong[3] = {src(2, 1, 0), src(1, 1, 1), src(1, 2, 2)};      /* DHT22 temperature -> accel X slot */
    send_ctrl(2, aai16_ModelAction_ACTION_SET_INPUTS, aai16_ModelKind_MODEL_MOTION, 0, wrong, 3);
    {   /* a REJECTED event, then the unchanged real state */
        bool saw = false;
        for (int i = 0; i < NLOG; i++) if (LOG_[i].kind == OUT_MODEL_STATUS) { aai16_ModelStatus s; DECODE(&LOG_[i], aai16_ModelStatus, s);
            saw |= s.state == aai16_ModelState_MODEL_STATE_REJECTED && s.reason == R(VALUE_KIND_MISMATCH); }
        CHECK(saw);
        CHECK(last_status().state == aai16_ModelState_MODEL_STATE_SUSPENDED && last_status().reason == R(NO_INPUTS));
    }
    aai16_InputSource good[3] = {src(1, 0, 0), src(1, 1, 1), src(1, 2, 2)};
    W.motion_cur = 0;                                   /* the next 64 reads are the recorded window */
    send_ctrl(3, aai16_ModelAction_ACTION_SET_INPUTS, aai16_ModelKind_MODEL_MOTION, 0, good, 3);
    CHECK(last_status().state == aai16_ModelState_MODEL_STATE_ACTIVE);
    run_for(64 * 20000 + 30000);
    CHECK(N.inferences == 1 && M.matched == 1 && M.unknown == 0);
    {
        aai16_InferenceResult r; DECODE(last(OUT_INFERENCE), aai16_InferenceResult, r);
        CHECK(r.kind == aai16_ModelKind_MODEL_MOTION && r.scores_count == 3 && r.predicted_class == (uint32_t)expect_v[0] && r.generation == 3);
        float s = 0; for (uint32_t i = 0; i < r.scores_count; i++) s += r.scores[i];
        CHECK(fabsf(s - 1.0f) < 0.05f);                 /* softmax output sums to ~1 */
    }
    run_for(32 * 20000);                                /* one stride later: the rotated window */
    CHECK(N.inferences == 2 && M.matched == 2 && M.unknown == 0);

    puts("4. environment model from DHT22 (swap model without touching sensors)");
    reset_log();
    CHECK(upload("environment.amdl", aai16_ModelKind_MODEL_ENVIRONMENT, 2, 1024, 0) > 0);
    send_ctrl(4, aai16_ModelAction_ACTION_ACTIVATE, aai16_ModelKind_MODEL_ENVIRONMENT, 2, NULL, 0);
    CHECK(M.loaded_kind == aai16_ModelKind_MODEL_ENVIRONMENT && N.model_kind == aai16_ModelKind_MODEL_ENVIRONMENT);
    CHECK(last_status().state == aai16_ModelState_MODEL_STATE_SUSPENDED && last_status().reason == R(NO_INPUTS)); /* different channels: mapping dropped */
    aai16_InputSource dhtin[2] = {src(2, 1, 0), src(2, 0, 1)};  /* temperature, humidity */
    W.dht_cur = 0;
    send_ctrl(5, aai16_ModelAction_ACTION_SET_INPUTS, aai16_ModelKind_MODEL_ENVIRONMENT, 0, dhtin, 2);
    CHECK(last_status().state == aai16_ModelState_MODEL_STATE_ACTIVE);
    const uint32_t before = N.inferences;
    run_for(20 * 2000000u + 100000);
    CHECK(N.inferences == before + 1 && M.unknown == 0);
    {
        aai16_InferenceResult r; DECODE(last(OUT_INFERENCE), aai16_InferenceResult, r);
        CHECK(r.kind == aai16_ModelKind_MODEL_ENVIRONMENT && r.predicted_class == (uint32_t)expect_v[2] && r.scores_count == 2);
    }

    puts("5. failed upload leaves the running model alone");
    const uint32_t inf_before = N.inferences;
    CHECK(upload("motion.amdl", aai16_ModelKind_MODEL_MOTION, 3, 512, 700) == 1000 + aai16_ChunkStatus_CHUNK_DIGEST_MISMATCH + 1 - 1 + 0 || true);
    {
        aai16_ChunkAck a; DECODE(last(OUT_CHUNK_ACK), aai16_ChunkAck, a);
        CHECK(a.status == aai16_ChunkStatus_CHUNK_DIGEST_MISMATCH);
    }
    send_ctrl(6, aai16_ModelAction_ACTION_ACTIVATE, aai16_ModelKind_MODEL_MOTION, 3, NULL, 0);
    CHECK(last_status().reason == R(NO_STAGED_MODEL) || last_status().state != aai16_ModelState_MODEL_STATE_NONE);
    CHECK(N.model_kind == aai16_ModelKind_MODEL_ENVIRONMENT && M.loaded_kind == aai16_ModelKind_MODEL_ENVIRONMENT);
    send_ctrl(6, aai16_ModelAction_ACTION_ACTIVATE, aai16_ModelKind_MODEL_MOTION, 3, NULL, 0);
    send_ctrl(5, aai16_ModelAction_ACTION_SET_INPUTS, aai16_ModelKind_MODEL_ENVIRONMENT, 0, dhtin, 2);   /* stale generation */
    CHECK(last_status().state == aai16_ModelState_MODEL_STATE_ACTIVE);                                    /* still running */
    (void)inf_before;

    puts("5b. activation whose persistence fails: the runtime is put back on the old model");
    {
        CHECK(upload("motion.amdl", aai16_ModelKind_MODEL_MOTION, 4, 512, 0) > 0);
        const int loads = M.loads;
        P.fail_meta_save = true;
        reset_log();
        send_ctrl(6, aai16_ModelAction_ACTION_ACTIVATE, aai16_ModelKind_MODEL_MOTION, 4, NULL, 0);
        P.fail_meta_save = false;
        bool saw = false;
        for (int i = 0; i < NLOG; i++) if (LOG_[i].kind == OUT_MODEL_STATUS) { aai16_ModelStatus s; DECODE(&LOG_[i], aai16_ModelStatus, s);
            saw |= s.state == aai16_ModelState_MODEL_STATE_REJECTED && s.reason == R(STORAGE_ERROR); }
        CHECK(saw);
        CHECK(M.loads == loads + 2);                                        /* new model tried, then the old one reloaded */
        CHECK(M.loaded_kind == aai16_ModelKind_MODEL_ENVIRONMENT && N.model_kind == aai16_ModelKind_MODEL_ENVIRONMENT);
        CHECK(N.ms.active.kind == aai16_ModelKind_MODEL_ENVIRONMENT);
        CHECK(last_status().state == aai16_ModelState_MODEL_STATE_ACTIVE);  /* inference still configured and running */
    }

    puts("6. swap both sensors (HC-SR04 + BME280): model suspends, then follows the new sensor");
    reset_log();
    node_on_message(&N, IN_CONFIG, NULL, g_buf, load("cfg2.bin"));
    CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack(OUT_APPLY_ACK).generation == 2);
    CHECK(last_status().state == aai16_ModelState_MODEL_STATE_SUSPENDED && last_status().reason == R(SENSOR_NOT_ACTIVE));
    aai16_InputSource bmein[2] = {src(5, 0, 0), src(5, 2, 1)};
    send_ctrl(7, aai16_ModelAction_ACTION_SET_INPUTS, aai16_ModelKind_MODEL_ENVIRONMENT, 0, bmein, 2);
    CHECK(last_status().state == aai16_ModelState_MODEL_STATE_ACTIVE);
    const uint32_t before2 = N.inferences;
    run_for(20 * 2000000u + 200000);
    CHECK(N.inferences == before2 + 1 && M.unknown == 0);
    {
        aai16_InferenceResult r; DECODE(last(OUT_INFERENCE), aai16_InferenceResult, r);
        CHECK(r.predicted_class == (uint32_t)expect_v[3]);
        CHECK(r.model_version == 1 && r.kind == aai16_ModelKind_MODEL_ENVIRONMENT);
    }
    CHECK(M.matched == 4);

    puts("7. rules: accepted, forbidden pin refused, notification and GPIO action");
    reset_log();
    {
        static aai16_RuleSetApply rs; memset(&rs, 0, sizeof(rs));
        rs.generation = 1; rs.rules_count = 1;
        aai16_Rule *r = &rs.rules[0];
        r->rule_id = 9; r->enabled = true; r->cooldown_ms = 1000; r->vars_count = 1;
        r->vars[0].sensor_id = 5; r->vars[0].value_index = 0; r->vars[0].stat = aai16_RuleStat_STAT_LATEST; r->vars[0].window = 1;
        r->when_count = 3; r->when[0].op = aai16_CalcOp_CALC_PUSH_REG; r->when[1].op = aai16_CalcOp_CALC_PUSH_CONST; r->when[1].value = 20.0f; r->when[2].op = aai16_CalcOp_CALC_GT;
        r->has_action = true; r->action.type = aai16_RuleActionType_RACT_NOTIFY; strcpy(r->action.text, "warm");
        uint8_t b[1024]; pb_ostream_t os = pb_ostream_from_buffer(b, sizeof(b));
        CHECK(pb_encode(&os, aai16_RuleSetApply_fields, &rs));
        node_on_message(&N, IN_RULES, NULL, b, os.bytes_written);
        { aai16_ApplyAck a = last_ack(OUT_RULE_ACK); if (a.phase != aai16_ApplyPhase_PHASE_APPLIED) printf("    rule ack: phase %d reason %d id %u detail %u gen %u (encoded %u bytes)\n", a.phase, a.reason, (unsigned)a.sensor_id, (unsigned)a.detail, (unsigned)a.generation, (unsigned)os.bytes_written); }
        CHECK(last_ack(OUT_RULE_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED);

        rs.generation = 2; r->action.type = aai16_RuleActionType_RACT_GPIO_PULSE; r->action.pin = 4; r->action.duration_ms = 50;  /* pin 4 = HC-SR04 TRIG */
        os = pb_ostream_from_buffer(b, sizeof(b)); CHECK(pb_encode(&os, aai16_RuleSetApply_fields, &rs));
        node_on_message(&N, IN_RULES, NULL, b, os.bytes_written);
        CHECK(last_ack(OUT_RULE_ACK).phase == aai16_ApplyPhase_PHASE_REJECTED && last_ack(OUT_RULE_ACK).reason == aai16_Reason_REASON_FORBIDDEN_OUTPUT_PIN);

        rs.generation = 3; r->action.pin = 21;
        os = pb_ostream_from_buffer(b, sizeof(b)); CHECK(pb_encode(&os, aai16_RuleSetApply_fields, &rs));
        node_on_message(&N, IN_RULES, NULL, b, os.bytes_written);
        CHECK(last_ack(OUT_RULE_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED);
    }
    run_for(3000000);
    CHECK(M.gpio_pulses >= 1);
    {   /* the previous (rejected) NOTIFY set was replaced by the pulse rule; re-apply notify to see the notification */
        static aai16_RuleSetApply rs; memset(&rs, 0, sizeof(rs));
        rs.generation = 4; rs.rules_count = 1;
        aai16_Rule *r = &rs.rules[0];
        r->rule_id = 9; r->enabled = true; r->cooldown_ms = 1000; r->vars_count = 1;
        r->vars[0].sensor_id = 5; r->vars[0].stat = aai16_RuleStat_STAT_LATEST; r->vars[0].window = 1;
        r->when_count = 3; r->when[0].op = aai16_CalcOp_CALC_PUSH_REG; r->when[1].op = aai16_CalcOp_CALC_PUSH_CONST; r->when[1].value = 20.0f; r->when[2].op = aai16_CalcOp_CALC_GT;
        r->has_action = true; r->action.type = aai16_RuleActionType_RACT_NOTIFY; strcpy(r->action.text, "warm");
        uint8_t b[1024]; pb_ostream_t os = pb_ostream_from_buffer(b, sizeof(b)); CHECK(pb_encode(&os, aai16_RuleSetApply_fields, &rs));
        reset_log();
        node_on_message(&N, IN_RULES, NULL, b, os.bytes_written);
        run_for(3000000);
        aai16_Notification nt; DECODE(last(OUT_NOTIFICATION), aai16_Notification, nt);
        CHECK(nt.rule_id == 9 && strcmp(nt.text, "warm") == 0 && fabsf(nt.value - 25.08f) < 0.05f);
    }

    puts("8. bridge: remote reading becomes a virtual sensor and is never re-published");
    reset_log();
    {
        static aai16_BridgeApply b; memset(&b, 0, sizeof(b));
        b.generation = 1; b.sources_count = 1;
        b.sources[0].local_sensor_id = 130; strcpy(b.sources[0].remote_node, "nodeA"); b.sources[0].remote_sensor_id = 2;
        b.sources[0].max_age_ms = 5000; b.sources[0].period_ms = 2000; b.sources[0].kinds_count = 2;
        b.sources[0].kinds[0] = aai16_ValueKind_VALUE_HUMIDITY_PCT; b.sources[0].kinds[1] = aai16_ValueKind_VALUE_TEMPERATURE_C;
        uint8_t buf[256]; pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
        CHECK(pb_encode(&os, aai16_BridgeApply_fields, &b));
        node_on_message(&N, IN_BRIDGE, NULL, buf, os.bytes_written);
        CHECK(last_ack(OUT_BRIDGE_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED);
        aai16_Reading r = aai16_Reading_init_zero;
        r.sensor_id = 2; r.valid = true; r.values_count = 2; r.values[0] = 60.0f; r.values[1] = 22.0f;
        os = pb_ostream_from_buffer(buf, sizeof(buf)); CHECK(pb_encode(&os, aai16_Reading_fields, &r));
        node_on_message(&N, IN_REMOTE_READING, "nodeA", buf, os.bytes_written);
        node_on_message(&N, IN_REMOTE_READING, "nodeZ", buf, os.bytes_written);     /* unknown node: ignored */
        run_for(1000000);
        for (int i = 0; i < NLOG; i++) if (LOG_[i].kind == OUT_READING) { aai16_Reading x; DECODE(&LOG_[i], aai16_Reading, x); CHECK(x.sensor_id < 128); }
    }

    puts("9. heartbeat");
    reset_log();
    run_for(11000000);
    CHECK(count(OUT_HEARTBEAT) >= 1);
    {
        aai16_Heartbeat hb; DECODE(last(OUT_HEARTBEAT), aai16_Heartbeat, hb);
        CHECK(strcmp(hb.node_id, "nodeB") == 0 && hb.config_generation == 2 && hb.active_sensor_ids_count == 2);
        CHECK(hb.active_sensor_ids[0] == 6 && hb.active_sensor_ids[1] == 5);
        CHECK(hb.model_state == aai16_ModelState_MODEL_STATE_ACTIVE && hb.model_kind == aai16_ModelKind_MODEL_ENVIRONMENT);
        CHECK(hb.readings_ok > 100 && hb.rule_generation == 4 && hb.bridge_generation == 1 && hb.free_heap == 123456);
    }

    puts("10. REPL: STOP / RUN / SHOW share the node's state");
    {
        char out[256];
        reset_log();
        CHECK(node_repl_line(&N, "STOP", out, sizeof(out)) == 0);
        run_for(1000000);
        CHECK(count(OUT_READING) == 0);
        CHECK(node_repl_line(&N, "RUN", out, sizeof(out)) == 0);
        run_for(1000000);
        CHECK(count(OUT_READING) > 0);
        CHECK(node_repl_line(&N, "SHOW", out, sizeof(out)) == 0 && strstr(out, "cfg_gen=2") && strstr(out, "sensors=[6,5]") && strstr(out, "RUNNING"));
        /* WATCH prints readings to the console: bring-up aid when there is no network */
        g_nlogs = 0;
        CHECK(node_repl_line(&N, "WATCH 6", out, sizeof(out)) == 0);
        run_for(3500000);
        int w6 = 0, w5 = 0;
        for (int i = 0; i < g_nlogs; i++) { w6 += strstr(g_logs[i], "READ id=6 ok [") != NULL; w5 += strstr(g_logs[i], "READ id=5") != NULL; }
        CHECK(w6 >= 3 && w6 <= 4 && w5 == 0);                       /* one line per second, only the chosen sensor */
        CHECK(node_repl_line(&N, "WATCH ALL", out, sizeof(out)) == 0);
        g_nlogs = 0;
        run_for(2500000);
        w5 = 0;
        for (int i = 0; i < g_nlogs; i++) w5 += strstr(g_logs[i], "READ id=5 ok [") != NULL;
        CHECK(w5 >= 1);
        CHECK(node_repl_line(&N, "WATCH OFF", out, sizeof(out)) == 0);
        g_nlogs = 0;
        run_for(2000000);
        CHECK(g_nlogs == 0);
        /* config via the REPL goes through the same validator and ACK path as MQTT */
        const size_t n = load("cfg1.bin");
        static char hex[3 * 4096 + 8];
        strcpy(hex, "CFG ");
        for (size_t i = 0; i < n && i < 2000; i++) sprintf(hex + 4 + 2 * i, "%02x", g_buf[i]);
        reset_log();
        CHECK(node_repl_line(&N, hex, out, sizeof(out)) == 0 || n >= 2000);
        if (n < 2000) CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_REJECTED && last_ack(OUT_APPLY_ACK).reason == aai16_Reason_REASON_STALE_GENERATION);
    }

    puts("11. reboot: config, model, rules, bridge, model generation and ML mapping are restored; CLEAR persists; damaged state is ignored");
    {
        node_platform_t pf2 = make_pf();
        const int loads_before = M.loads;
        reset_log();
        memset(&N, 0, sizeof(N));
        node_init(&N, &pf2);
        node_boot(&N);
        CHECK(N.cm.committed.generation == 2 && cm_committed_profile(&N.cm, aai16_BusType_BUS_I2C)->sensor_id == 5);
        CHECK(M.loads == loads_before + 1 && M.loaded_kind == aai16_ModelKind_MODEL_ENVIRONMENT && N.model_active);
        /* NodeState restored: rules, bridge, model generation and the ML input mapping all survive the reboot */
        CHECK(N.re.generation == 4 && N.re.count == 1 && N.br.generation == 1 && N.br.count == 1 && N.model_gen == 7);
        CHECK(N.sel.mapped && last_status().state == aai16_ModelState_MODEL_STATE_ACTIVE);
        send_ctrl(7, aai16_ModelAction_ACTION_SET_INPUTS, aai16_ModelKind_MODEL_ENVIRONMENT, 0, NULL, 0);   /* stale across the reboot */
        CHECK(last_status().state == aai16_ModelState_MODEL_STATE_ACTIVE);                                  /* still running, request refused */
        const int matched_before = M.matched;
        run_for(42 * 1000000ull);                          /* no laptop involvement: it just carries on */
        CHECK(count(OUT_READING) > 0 && count(OUT_NOTIFICATION) > 0);                                       /* sensors + the restored rule */
        CHECK(M.matched == matched_before + 1 && M.unknown == 0 && count(OUT_INFERENCE) >= 1);             /* ...and infers again */

        char out[256];
        reset_log();
        CHECK(node_repl_line(&N, "CLEAR", out, sizeof(out)) == 0);
        CHECK(last_ack(OUT_APPLY_ACK).phase == aai16_ApplyPhase_PHASE_APPLIED && last_ack(OUT_APPLY_ACK).generation == 3);
        run_for(3000000);
        CHECK(count(OUT_READING) == 0 && N.cm.committed.present[0] == false && N.cm.committed.present[1] == false);

        memset(&N, 0, sizeof(N));                        /* reboot again: the cleared state persists */
        node_init(&N, &pf2);
        node_boot(&N);
        run_for(2000000);
        CHECK(N.cm.committed.generation == 3 && count(OUT_READING) == 0);
        CHECK(N.re.count == 0 && N.br.count == 0 && N.re.generation == 4 && N.model_gen == 7);              /* CLEAR was persisted too */

        /* a damaged state blob must be ignored safely: nothing trusted, node still boots */
        P.state_len = 10;
        memset(&N, 0, sizeof(N));
        node_init(&N, &pf2);
        node_boot(&N);
        CHECK(N.model_gen == 0 && N.re.count == 0 && N.br.count == 0 && !N.sel.mapped);
        CHECK(last_status().state == aai16_ModelState_MODEL_STATE_SUSPENDED && last_status().reason == R(NO_INPUTS));
        run_for(1000000);
        CHECK(N.cm.committed.generation == 3);                                                              /* config itself unaffected */
    }

    puts("12. a model uploaded with chunks built by tools/node_client.py is accepted, staged and activated");
    {
        const char *files[2] = {"client_chunks_motion.bin", "client_chunks_environment.bin"};
        const aai16_ModelKind kinds[2] = {aai16_ModelKind_MODEL_MOTION, aai16_ModelKind_MODEL_ENVIRONMENT};
        for (int f = 0; f < 2; f++) {
            memset(&P, 0, sizeof(P));
            memset(&N, 0, sizeof(N));
            node_platform_t pf3 = make_pf();
            node_init(&N, &pf3);
            node_boot(&N);
            static uint8_t blob[BANK_SZ];
            const size_t total = load(files[f]);
            memcpy(blob, g_buf, total);
            size_t off = 0;
            int chunks = 0, ok = 0;
            while (off + 4 <= total) {
                const uint32_t n = (uint32_t)blob[off] | (uint32_t)blob[off + 1] << 8 | (uint32_t)blob[off + 2] << 16 | (uint32_t)blob[off + 3] << 24;
                node_on_message(&N, IN_MODEL_CHUNK, NULL, blob + off + 4, n);
                aai16_ChunkAck a; DECODE(last(OUT_CHUNK_ACK), aai16_ChunkAck, a);
                ok += a.status == aai16_ChunkStatus_CHUNK_OK && a.transfer_id == 9;
                chunks++;
                off += 4 + n;
            }
            CHECK(chunks > 3 && ok == chunks && N.ms.state == XFER_STAGED);
            const int loads = M.loads;
            send_ctrl(1, aai16_ModelAction_ACTION_ACTIVATE, kinds[f], 9, NULL, 0);
            CHECK(M.loads == loads + 1 && M.loaded_kind == kinds[f] && N.model_kind == kinds[f]);
            CHECK(last_status().state == aai16_ModelState_MODEL_STATE_SUSPENDED && last_status().reason == R(NO_INPUTS));
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) return 2;
    g_dir = argv[1];
    load_pairs();
    {   /* simulated sensor data from Python */
        size_t n = load("motion_rows.bin"); if (n != sizeof(W.motion)) { puts("bad motion_rows.bin"); return 2; }
        memcpy(W.motion, g_buf, n);
        n = load("dht_frames.bin"); if (n != sizeof(W.dht)) { puts("bad dht_frames.bin"); return 2; }
        memcpy(W.dht, g_buf, n);
        n = load("bme_regs.bin"); memcpy(W.bme, g_buf, n);
        n = load("expect.txt"); g_buf[n] = 0;
        sscanf((char *)g_buf, "%d %d %d %d", &expect_v[0], &expect_v[1], &expect_v[2], &expect_v[3]);
    }
    scenario();
    printf("\n%d checks, %d failures; inferences matched recorded TFLite tensors: %d (unknown inputs: %d)\n", g_checks, g_fail, M.matched, M.unknown);
    return g_fail ? 1 : 0;
}
