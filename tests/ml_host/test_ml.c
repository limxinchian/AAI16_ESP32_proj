/* Host tests for ml_core. argv[1] = directory with cases.txt / sha_cases.txt / packages.txt written by
 * run.py (golden vectors and digests come from independent Python code). The flash backend and the
 * sensor table below are bounded test doubles; the packages are the real trained artifacts. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ml_features.h"
#include "ml_package.h"
#include "ml_selector.h"
#include "model_store.h"
#include "sha256.h"

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

#define R(x) aai16_ModelReason_MODEL_REASON_##x

static FILE *open_in(const char *name)
{
    char p[600];
    snprintf(p, sizeof(p), "%s/%s", g_dir, name);
    FILE *f = fopen(p, "rb");
    if (!f) {
        printf("cannot open %s\n", p);
        exit(2);
    }
    return f;
}

/* ------------------------------------------------------------------ sha256 */
static void hex2bin(const char *h, uint8_t *out, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        unsigned v;
        sscanf(h + 2 * i, "%2x", &v);
        out[i] = (uint8_t)v;
    }
}

static void test_sha256(void)
{
    puts("sha256 matches Python hashlib for all lengths, any update split");
    FILE *f = open_in("sha_cases.txt");
    static char hex[20000];
    char want[80];
    size_t n;
    while (fscanf(f, "%zu %19999s %79s", &n, hex, want) == 3) {
        static uint8_t data[6000], exp[32], got[32];
        if (n) {
            hex2bin(hex, data, n);
        }
        hex2bin(want, exp, 32);
        for (size_t split = 0; split < 3; split++) { /* one shot, byte-wise, odd chunks */
            sha256_ctx_t c;
            sha256_init(&c);
            if (split == 0) {
                sha256_update(&c, data, n);
            } else {
                const size_t step = split == 1 ? 1 : 37;
                for (size_t i = 0; i < n; i += step) {
                    sha256_update(&c, data + i, (n - i < step) ? n - i : step);
                }
            }
            sha256_final(&c, got);
            CHECK(memcmp(got, exp, 32) == 0);
        }
    }
    fclose(f);
}

/* ------------------------------------------------- features + int8 glue vs golden */
static float g_in_scale[3], g_out_scale[3];
static int g_in_zp[3], g_out_zp[3];

static void test_golden(void)
{
    puts("features + quantize + dequantize reproduce the Python golden vectors");
    FILE *f = open_in("cases.txt");
    char tag[4];
    int pre, win, ch, nf, nc, pred, vectors = 0, qlines = 0;
    while (fscanf(f, "%3s", tag) == 1) {
        if (tag[0] == 'Q') {
            double a, b;
            int za, zb;
            CHECK(fscanf(f, "%lf %d %lf %d", &a, &za, &b, &zb) == 4);
            g_in_scale[qlines] = (float)a; g_in_zp[qlines] = za; g_out_scale[qlines] = (float)b; g_out_zp[qlines] = zb;
            qlines++;
            continue;
        }
        CHECK(fscanf(f, "%d %d %d %d %d %d", &pre, &win, &ch, &nf, &nc, &pred) == 6);
        static float w[ML_MAX_WINDOW * ML_MAX_CHANNELS], feat[ML_MAX_FEATURES], ours[ML_MAX_FEATURES], probs[4], oprobs[4];
        static int8_t qin[ML_MAX_FEATURES], qout[4];
        for (int i = 0; i < win * ch; i++) CHECK(fscanf(f, "%f", &w[i]) == 1);
        for (int i = 0; i < nf; i++) CHECK(fscanf(f, "%f", &feat[i]) == 1);
        for (int i = 0; i < nf; i++) { int v; CHECK(fscanf(f, "%d", &v) == 1); qin[i] = (int8_t)v; }
        for (int i = 0; i < nc; i++) { int v; CHECK(fscanf(f, "%d", &v) == 1); qout[i] = (int8_t)v; }
        for (int i = 0; i < nc; i++) CHECK(fscanf(f, "%f", &probs[i]) == 1);

        CHECK(ml_compute_features((aai16_Preprocessing)pre, w, (uint32_t)win, (uint32_t)ch, ours, ML_MAX_FEATURES));
        const int task = pre - 1; /* 0 motion, 1 environment (scales are written after each task's vectors) */
        (void)task;
        bool feat_ok = true;
        for (int i = 0; i < nf; i++) {
            feat_ok &= fabsf(ours[i] - feat[i]) <= 1e-4f + 1e-4f * fabsf(feat[i]);
        }
        g_checks++;
        if (!feat_ok) {
            g_fail++;
            printf("  FAIL features differ (pre %d):", pre);
            for (int i = 0; i < nf; i++) printf(" %.6f/%.6f", ours[i], feat[i]);
            putchar('\n');
        }
        vectors++;
        /* int8 side is checked after the Q line is known: stash by recomputing below */
        (void)qin; (void)qout; (void)oprobs;
    }
    CHECK(vectors == 15 + 10 && qlines == 2);
    fclose(f);

    /* second pass: quantization/dequantization with the right scales per task */
    f = open_in("cases.txt");
    int seen_q = 0;
    long total = 0, exact_q = 0;
    while (fscanf(f, "%3s", tag) == 1) {
        if (tag[0] == 'Q') { double a, b; int za, zb; (void)fscanf(f, "%lf %d %lf %d", &a, &za, &b, &zb); seen_q++; continue; }
        (void)fscanf(f, "%d %d %d %d %d %d", &pre, &win, &ch, &nf, &nc, &pred);
        static float w[ML_MAX_WINDOW * ML_MAX_CHANNELS], feat[ML_MAX_FEATURES], probs[4], got[4];
        static int8_t qin[ML_MAX_FEATURES], qout[4];
        for (int i = 0; i < win * ch; i++) (void)fscanf(f, "%f", &w[i]);
        for (int i = 0; i < nf; i++) (void)fscanf(f, "%f", &feat[i]);
        for (int i = 0; i < nf; i++) { int v; (void)fscanf(f, "%d", &v); qin[i] = (int8_t)v; }
        for (int i = 0; i < nc; i++) { int v; (void)fscanf(f, "%d", &v); qout[i] = (int8_t)v; }
        for (int i = 0; i < nc; i++) (void)fscanf(f, "%f", &probs[i]);
        const int t = pre == 1 ? 0 : 1; /* tasks appear motion then environment */
        for (int i = 0; i < nf; i++) {
            total++;
            exact_q += ml_quantize(feat[i], g_in_scale[t], g_in_zp[t]) == qin[i];
        }
        const uint32_t best = ml_decode_output(qout, (uint32_t)nc, g_out_scale[t], g_out_zp[t], got);
        CHECK((int)best == pred);
        for (int i = 0; i < nc; i++) CHECK(fabsf(got[i] - probs[i]) < 1e-5f);
    }
    (void)seen_q;
    CHECK(exact_q == total);
    printf("  quantized %ld/%ld feature values identically to TFLite\n", exact_q, total);
    fclose(f);
}

/* ------------------------------------------------------------- sensor table */
typedef struct { uint32_t id; bool active; uint32_t period; uint32_t n; aai16_ValueKind k[8]; } sensor_t;
static sensor_t S[8];

static void sensors_reset(void)
{
    memset(S, 0, sizeof(S));
    S[0] = (sensor_t){1, true, 20, 3, {aai16_ValueKind_VALUE_ACCEL_X_G, aai16_ValueKind_VALUE_ACCEL_Y_G, aai16_ValueKind_VALUE_ACCEL_Z_G}};  /* GY-511 */
    S[1] = (sensor_t){2, true, 2000, 2, {aai16_ValueKind_VALUE_HUMIDITY_PCT, aai16_ValueKind_VALUE_TEMPERATURE_C}};                       /* DHT22 */
    S[2] = (sensor_t){5, false, 2000, 3, {aai16_ValueKind_VALUE_TEMPERATURE_C, aai16_ValueKind_VALUE_PRESSURE_PA, aai16_ValueKind_VALUE_HUMIDITY_PCT}}; /* BME280 */
    S[3] = (sensor_t){7, true, 2000, 1, {aai16_ValueKind_VALUE_CO2_PPM}};                                                                 /* MH-Z19 */
}

static bool lookup(uint32_t id, ml_source_t *o, void *ctx)
{
    (void)ctx;
    for (int i = 0; i < 8; i++) {
        if (S[i].id == id && S[i].active) {
            memset(o, 0, sizeof(*o));
            o->active = true; o->period_ms = S[i].period; o->output_count = S[i].n;
            memcpy(o->kinds, S[i].k, sizeof(o->kinds));
            return true;
        }
    }
    return false;
}

static sensor_t *sensor(uint32_t id) { for (int i = 0; i < 8; i++) if (S[i].id == id) return &S[i]; return NULL; }

/* -------------------------------------------------------------- packages */
static uint8_t g_pkg[2][8192];
static size_t g_pkg_len[2];
static uint8_t g_pkg_digest[2][32];

static void load_packages(void)
{
    FILE *f = open_in("packages.txt");
    char path[600], hex[80];
    for (int i = 0; i < 2; i++) {
        CHECK(fscanf(f, "%599s %79s", path, hex) == 2);
        FILE *p = open_in(path);
        g_pkg_len[i] = fread(g_pkg[i], 1, sizeof(g_pkg[i]), p);
        fclose(p);
        hex2bin(hex, g_pkg_digest[i], 32);
    }
    fclose(f);
}

static void test_package_parse(void)
{
    puts("package parsing accepts the trained models and rejects damaged ones");
    ml_package_t p;
    CHECK(ml_package_parse(g_pkg[0], g_pkg_len[0], &p) == R(NONE));
    CHECK(p.contract.kind == aai16_ModelKind_MODEL_MOTION && p.contract.channels_count == 3 && p.contract.labels_count == 3);
    CHECK(strcmp(p.contract.labels[1], "MOVING") == 0 && p.contract.feature_count == 11 && p.contract.stride_samples == 32);
    CHECK(ml_package_parse(g_pkg[1], g_pkg_len[1], &p) == R(NONE));
    CHECK(p.contract.kind == aai16_ModelKind_MODEL_ENVIRONMENT && p.contract.channels[1] == aai16_ValueKind_VALUE_HUMIDITY_PCT);

    static uint8_t bad[8192];
    memcpy(bad, g_pkg[0], g_pkg_len[0]);
    bad[g_pkg_len[0] - 10] ^= 0x01; /* flip a bit inside the tflite payload */
    CHECK(ml_package_parse(bad, g_pkg_len[0], &p) == R(DIGEST_MISMATCH));
    memcpy(bad, g_pkg[0], g_pkg_len[0]);
    bad[0] = 'X';
    CHECK(ml_package_parse(bad, g_pkg_len[0], &p) == R(BAD_PACKAGE));
    CHECK(ml_package_parse(g_pkg[0], 7, &p) == R(BAD_PACKAGE));
    CHECK(ml_package_parse(g_pkg[0], 20, &p) == R(BAD_PACKAGE));
    CHECK(ml_package_parse(g_pkg[0], g_pkg_len[0] - 100, &p) != R(NONE)); /* truncated => digest */
    memcpy(bad, g_pkg[0], g_pkg_len[0]);
    bad[4] = 0xFF; bad[5] = 0xFF; /* absurd contract length */
    CHECK(ml_package_parse(bad, g_pkg_len[0], &p) == R(BAD_PACKAGE));
}

/* ------------------------------------------------------------- selector */
static ml_package_t g_motion, g_env;
static ml_selector_t sel;

static aai16_InputSource in(uint32_t sid, uint32_t vi, uint32_t slot)
{
    aai16_InputSource i = aai16_InputSource_init_zero;
    i.sensor_id = sid; i.value_index = vi; i.feature_slot = slot;
    return i;
}

static void test_selector_compat(void)
{
    puts("input selector: valid mappings accepted, incompatible ones rejected before inference");
    sensors_reset();
    mlsel_init(&sel);
    aai16_InputSource m[3] = {in(1, 0, 0), in(1, 1, 1), in(1, 2, 2)};
    CHECK(mlsel_set_inputs(&sel, m, 3, lookup, NULL) == R(NO_ACTIVE_MODEL));

    mlsel_set_contract(&sel, &g_motion.contract);
    CHECK(sel.state == R(NO_INPUTS));
    CHECK(mlsel_set_inputs(&sel, m, 3, lookup, NULL) == R(NONE));

    /* AC-09: DHT22 temperature -> motion model is refused, previous mapping stays */
    aai16_InputSource bad[3] = {in(2, 1, 0), in(1, 1, 1), in(1, 2, 2)};
    CHECK(mlsel_set_inputs(&sel, bad, 3, lookup, NULL) == R(VALUE_KIND_MISMATCH));
    CHECK(sel.mapped && sel.map[0].sensor_id == 1 && sel.map[0].value_index == 0);
    CHECK(mlsel_set_inputs(&sel, m, 2, lookup, NULL) == R(INPUT_COUNT_MISMATCH));
    aai16_InputSource dup[3] = {in(1, 0, 0), in(1, 1, 0), in(1, 2, 2)};
    CHECK(mlsel_set_inputs(&sel, dup, 3, lookup, NULL) == R(DUPLICATE_SLOT));
    aai16_InputSource slot[3] = {in(1, 0, 0), in(1, 1, 1), in(1, 2, 3)};
    CHECK(mlsel_set_inputs(&sel, slot, 3, lookup, NULL) == R(BAD_SLOT));
    aai16_InputSource swapped[3] = {in(1, 1, 0), in(1, 0, 1), in(1, 2, 2)}; /* y into the x slot */
    CHECK(mlsel_set_inputs(&sel, swapped, 3, lookup, NULL) == R(VALUE_KIND_MISMATCH));
    aai16_InputSource idx[3] = {in(1, 0, 0), in(1, 1, 1), in(1, 3, 2)};
    CHECK(mlsel_set_inputs(&sel, idx, 3, lookup, NULL) == R(VALUE_INDEX_RANGE));
    aai16_InputSource gone[3] = {in(9, 0, 0), in(1, 1, 1), in(1, 2, 2)};
    CHECK(mlsel_set_inputs(&sel, gone, 3, lookup, NULL) == R(SENSOR_NOT_ACTIVE));
    aai16_InputSource co2[3] = {in(7, 0, 0), in(1, 1, 1), in(1, 2, 2)};
    CHECK(mlsel_set_inputs(&sel, co2, 3, lookup, NULL) == R(VALUE_KIND_MISMATCH));
    CHECK(sel.map[0].sensor_id == 1 && sel.map[2].value_index == 2); /* still the original, valid mapping */

    sensor(1)->period = 100; /* GY-511 configured at 10 Hz, model needs 50 Hz */
    CHECK(mlsel_set_inputs(&sel, m, 3, lookup, NULL) == R(PERIOD_MISMATCH));
    sensors_reset();

    /* environment model: DHT22 now, then BME280 with the SAME model contract */
    mlsel_set_contract(&sel, &g_env.contract);
    CHECK(sel.state == R(NO_INPUTS)); /* different channel kinds: the old motion mapping is dropped */
    aai16_InputSource dht[2] = {in(2, 1, 0), in(2, 0, 1)}; /* slot0 temperature, slot1 humidity */
    CHECK(mlsel_set_inputs(&sel, dht, 2, lookup, NULL) == R(NONE));
    aai16_InputSource bme[2] = {in(5, 0, 0), in(5, 2, 1)};
    CHECK(mlsel_set_inputs(&sel, bme, 2, lookup, NULL) == R(SENSOR_NOT_ACTIVE)); /* BME280 not selected yet */
    sensor(5)->active = true;
    sensor(2)->active = false; /* the GPIO bus now runs HC-SR04 instead; DHT22 is gone */
    CHECK(mlsel_revalidate(&sel, lookup, NULL) == R(SENSOR_NOT_ACTIVE)); /* suspended, model kept */
    CHECK(mlsel_set_inputs(&sel, bme, 2, lookup, NULL) == R(NONE));
    CHECK(mlsel_revalidate(&sel, lookup, NULL) == R(NONE));
    sensor(5)->period = 1000; /* BME280 at 1 Hz vs the model's 2 s spacing */
    CHECK(mlsel_set_inputs(&sel, bme, 2, lookup, NULL) == R(PERIOD_MISMATCH));
    sensors_reset();

    /* same-kind contract keeps the mapping; different kinds drop it */
    mlsel_init(&sel);
    mlsel_set_contract(&sel, &g_env.contract);
    CHECK(mlsel_set_inputs(&sel, dht, 2, lookup, NULL) == R(NONE));
    mlsel_set_contract(&sel, &g_env.contract);
    CHECK(sel.mapped && sel.state == R(NONE));
    mlsel_set_contract(&sel, &g_motion.contract);
    CHECK(!sel.mapped && sel.state == R(NO_INPUTS));
}

/* load golden window i of a task from cases.txt: returns rows (win*ch floats) and features */
static int golden_vec(int pre, int index, float *rows, float *feat, int *win, int *ch, int *nf)
{
    FILE *f = open_in("cases.txt");
    char tag[4];
    int p, nc, pred, seen = 0, found = 0;
    while (fscanf(f, "%3s", tag) == 1) {
        if (tag[0] == 'Q') { double a, b; int za, zb; (void)fscanf(f, "%lf %d %lf %d", &a, &za, &b, &zb); continue; }
        (void)fscanf(f, "%d %d %d %d %d %d", &p, win, ch, nf, &nc, &pred);
        static float w[ML_MAX_WINDOW * ML_MAX_CHANNELS], ft[ML_MAX_FEATURES];
        for (int i = 0; i < *win * *ch; i++) (void)fscanf(f, "%f", &w[i]);
        for (int i = 0; i < *nf; i++) (void)fscanf(f, "%f", &ft[i]);
        for (int i = 0; i < *nf + 2 * nc; i++) { double d; (void)fscanf(f, "%lf", &d); }
        if (p == pre && seen++ == index) {
            memcpy(rows, w, sizeof(float) * (size_t)(*win * *ch));
            memcpy(feat, ft, sizeof(float) * (size_t)*nf);
            found = 1;
            break;
        }
    }
    fclose(f);
    return found;
}

static void test_selector_stream(void)
{
    puts("selector streams readings into windows, infers on stride, suspends on config change");
    static float rows[ML_MAX_WINDOW * ML_MAX_CHANNELS], want[ML_MAX_FEATURES], got[ML_MAX_FEATURES];
    int win, ch, nf;
    CHECK(golden_vec(1, 3, rows, want, &win, &ch, &nf));
    sensors_reset();
    mlsel_init(&sel);
    mlsel_set_contract(&sel, &g_motion.contract);
    aai16_InputSource m[3] = {in(1, 0, 0), in(1, 1, 1), in(1, 2, 2)};
    CHECK(mlsel_set_inputs(&sel, m, 3, lookup, NULL) == R(NONE));

    bool ready = false;
    for (int i = 0; i < win; i++) {
        CHECK(mlsel_push(&sel, 1, 1000000u + (uint64_t)i * 20000u, true, &rows[i * 3], 3, lookup, NULL, &ready, got) == R(NONE));
        if (i < win - 1) CHECK(!ready);
    }
    CHECK(ready);
    bool same = true;
    for (int i = 0; i < nf; i++) same &= fabsf(got[i] - want[i]) <= 1e-4f + 1e-4f * fabsf(want[i]);
    CHECK(same);

    /* sliding: next inference after `stride` (32) more samples, not before */
    uint64_t ts = 1000000u + (uint64_t)win * 20000u;
    int until_ready = 0;
    for (int i = 0; i < 64; i++, ts += 20000) {
        mlsel_push(&sel, 1, ts, true, &rows[(i % win) * 3], 3, lookup, NULL, &ready, got);
        if (ready) { until_ready = i + 1; break; }
    }
    CHECK(until_ready == 32);

    /* timing gap beyond tolerance restarts the window */
    ts += 200000u; /* 220 ms instead of 20 ms */
    CHECK(mlsel_push(&sel, 1, ts, true, &rows[0], 3, lookup, NULL, &ready, got) == R(PERIOD_MISMATCH));
    CHECK(!ready && sel.count == 1);
    for (int i = 1; i < win; i++) {
        ts += 20000u;
        mlsel_push(&sel, 1, ts, true, &rows[i * 3], 3, lookup, NULL, &ready, got);
    }
    CHECK(ready); /* refilled completely before inferring again */

    /* invalid reading resets */
    ts += 20000u;
    CHECK(mlsel_push(&sel, 1, ts, false, &rows[0], 3, lookup, NULL, &ready, got) == R(READING_INVALID) && sel.count == 0);

    /* I2C bus swapped away: suspended, no inference, and no stale samples after it returns */
    for (int i = 0; i < 40; i++) { ts += 20000u; mlsel_push(&sel, 1, ts, true, &rows[(i % win) * 3], 3, lookup, NULL, &ready, got); }
    CHECK(sel.count == 40);
    sensor(1)->active = false;
    ts += 20000u;
    CHECK(mlsel_push(&sel, 1, ts, true, &rows[0], 3, lookup, NULL, &ready, got) == R(SENSOR_NOT_ACTIVE) && !ready && sel.count == 0);
    CHECK(sel.mapped && sel.has_contract); /* model and mapping preserved */
    sensor(1)->active = true;
    int pushes = 0;
    for (int i = 0; i < win; i++) {
        ts += 20000u;
        mlsel_push(&sel, 1, ts, true, &rows[i * 3], 3, lookup, NULL, &ready, got);
        pushes++;
        if (ready) break;
    }
    CHECK(ready && pushes == win); /* a full fresh window, nothing from before the suspension */

    /* environment: temperature from DHT22 (id 2), humidity from a different sensor => freshness rule */
    sensors_reset();
    sensor(5)->active = true;
    mlsel_init(&sel);
    mlsel_set_contract(&sel, &g_env.contract);
    aai16_InputSource e[2] = {in(2, 1, 0), in(5, 2, 1)};
    CHECK(mlsel_set_inputs(&sel, e, 2, lookup, NULL) == R(NONE));
    float v2[2] = {40.0f, 22.0f}, v5[3] = {22.0f, 100000.0f, 40.0f};
    uint64_t t0 = 10000000u;
    CHECK(mlsel_push(&sel, 2, t0, true, v2, 2, lookup, NULL, &ready, got) == R(INPUTS_STALE)); /* nothing from sensor 5 yet */
    mlsel_push(&sel, 5, t0 + 1000, true, v5, 3, lookup, NULL, &ready, got);
    CHECK(mlsel_push(&sel, 2, t0 + 2000000u, true, v2, 2, lookup, NULL, &ready, got) == R(NONE) && sel.count == 1);
    CHECK(mlsel_push(&sel, 2, t0 + 4000000u, true, v2, 2, lookup, NULL, &ready, got) == R(NONE) && sel.count == 2);
    CHECK(mlsel_push(&sel, 2, t0 + 12000000u, true, v2, 2, lookup, NULL, &ready, got) == R(INPUTS_STALE) && sel.count == 0); /* sensor 5 silent for 12 s > 6 s */
}

/* ------------------------------------------------------- flash backend */
#define BANK_SZ 65536
static struct {
    uint8_t bank[2][BANK_SZ];
    model_meta_t meta;
    bool has_meta;
    int writes, erases;
    int fail_write_at;   /* 1-based write index that fails (0 = never) */
    bool fail_meta_save;
    uint8_t last_erased;
} F;

static bool f_erase(void *c, uint8_t b, size_t size) { (void)c; (void)size; F.erases++; F.last_erased = b; memset(F.bank[b], 0xFF, BANK_SZ); return true; }
static bool f_write(void *c, uint8_t b, uint32_t off, const uint8_t *d, size_t n)
{
    (void)c;
    F.writes++;
    if (F.fail_write_at && F.writes == F.fail_write_at) return false;
    if (off + n > BANK_SZ) return false;
    memcpy(F.bank[b] + off, d, n);
    return true;
}
static const uint8_t *f_map(void *c, uint8_t b, size_t len) { (void)c; return len <= BANK_SZ ? F.bank[b] : NULL; }
static bool f_load(void *c, model_meta_t *m) { (void)c; if (!F.has_meta) return false; *m = F.meta; return true; }
static bool f_save(void *c, const model_meta_t *m) { (void)c; if (F.fail_meta_save) return false; F.meta = *m; F.has_meta = true; return true; }

static const model_store_ops_t OPS = {f_erase, f_write, f_map, f_load, f_save, NULL, BANK_SZ};
static model_store_t ms;

typedef struct { int corrupt_at; int skip_seq; int dup_seq; int stop_after; aai16_ModelKind kind; int digest_from; uint32_t version; } upload_t;

/* Sends package `which` in chunks; returns the last ack status. */
static aai16_ChunkStatus upload(int which, uint32_t chunk, uint32_t tid, uint32_t gen, upload_t o, uint32_t *acks_ok)
{
    static aai16_ModelChunk c;
    aai16_ChunkAck a = {0};
    const size_t len = g_pkg_len[which];
    const uint32_t total = (uint32_t)((len + chunk - 1) / chunk);
    aai16_ChunkStatus last = aai16_ChunkStatus_CHUNK_OK;
    if (acks_ok) *acks_ok = 0;
    for (uint32_t seq = 0; seq < total; seq++) {
        if ((int)seq + 1 == o.skip_seq) continue;   /* skip_seq/dup_seq are 1-based; 0 = none */
        if (o.stop_after && (int)seq >= o.stop_after) break;
        memset(&c, 0, sizeof(c));
        c.transfer_id = tid; c.generation = gen;
        c.kind = o.kind ? o.kind : (aai16_ModelKind)(which + 1);
        c.format = aai16_ModelFormat_FORMAT_TFLITE;
        c.model_version = o.version ? o.version : 1;
        c.seq = seq; c.total_chunks = total; c.total_size = (uint32_t)len;
        c.sha256.size = 32;
        memcpy(c.sha256.bytes, g_pkg_digest[o.digest_from ? o.digest_from - 1 : which], 32);
        const size_t off = (size_t)seq * chunk, n = (len - off < chunk) ? len - off : chunk;
        c.data.size = (pb_size_t)n;
        memcpy(c.data.bytes, g_pkg[which] + off, n);
        if (o.corrupt_at && (size_t)o.corrupt_at >= off && (size_t)o.corrupt_at < off + n) c.data.bytes[o.corrupt_at - off] ^= 0x40;
        model_store_on_chunk(&ms, &c, &a);
        last = a.status;
        if (a.status == aai16_ChunkStatus_CHUNK_OK && acks_ok) (*acks_ok)++;
        if ((int)seq + 1 == o.dup_seq) { model_store_on_chunk(&ms, &c, &a); CHECK(a.status == aai16_ChunkStatus_CHUNK_DUPLICATE); }
        if (a.status != aai16_ChunkStatus_CHUNK_OK && a.status != aai16_ChunkStatus_CHUNK_DUPLICATE) {
            return a.status;
        }
    }
    return last;
}

static void fresh_store(void)
{
    memset(&F, 0, sizeof(F));
    model_store_init(&ms, &OPS);
}

static aai16_ModelReason accept_all(const ml_package_t *p, void *c) { (void)p; (void)c; return R(NONE); }
static aai16_ModelReason reject_all(const ml_package_t *p, void *c) { (void)p; (void)c; return R(RUNTIME_REJECTED); }

static void test_store(void)
{
    puts("model store: chunked upload, staging, activation, rollback, reboot recovery");
    static const uint32_t sizes[3] = {256, 512, 1024};
    ml_package_t act;

    /* every allowed chunk size transfers a complete, verified package */
    for (int s = 0; s < 3; s++) {
        fresh_store();
        uint32_t ok = 0;
        CHECK(upload(0, sizes[s], 10, 1, (upload_t){0}, &ok) == aai16_ChunkStatus_CHUNK_OK);
        CHECK(ok == (g_pkg_len[0] + sizes[s] - 1) / sizes[s]);
        CHECK(ms.state == XFER_STAGED && !ms.has_active);
    }

    /* motion first */
    fresh_store();
    CHECK(model_store_active(&ms, &act) == R(NO_ACTIVE_MODEL));
    CHECK(upload(0, 512, 1, 1, (upload_t){0}, NULL) == aai16_ChunkStatus_CHUNK_OK);
    CHECK(model_store_activate(&ms, 2, accept_all, NULL) == R(TRANSFER_MISMATCH));
    CHECK(model_store_activate(&ms, 1, reject_all, NULL) == R(RUNTIME_REJECTED));
    CHECK(!ms.has_active && !F.has_meta);                     /* nothing activated by a failed check */
    F.fail_meta_save = true;
    CHECK(model_store_activate(&ms, 1, accept_all, NULL) == R(STORAGE_ERROR) && !ms.has_active);
    F.fail_meta_save = false;
    CHECK(model_store_activate(&ms, 1, accept_all, NULL) == R(NONE) && ms.has_active && ms.active.bank == 0);
    CHECK(model_store_active(&ms, &act) == R(NONE) && act.contract.kind == aai16_ModelKind_MODEL_MOTION);
    CHECK(model_store_activate(&ms, 1, accept_all, NULL) == R(NO_STAGED_MODEL));

    /* second upload (environment) goes to the OTHER bank and leaves motion intact until activation */
    uint8_t before[256];
    memcpy(before, F.bank[0], sizeof(before));
    CHECK(upload(1, 1024, 2, 2, (upload_t){0}, NULL) == aai16_ChunkStatus_CHUNK_OK);
    CHECK(ms.stage_bank == 1 && memcmp(before, F.bank[0], sizeof(before)) == 0);
    CHECK(model_store_active(&ms, &act) == R(NONE) && act.contract.kind == aai16_ModelKind_MODEL_MOTION);

    /* late duplicate of chunk 0 after staging must not destroy the staged copy */
    int erases = F.erases;
    {
        static aai16_ModelChunk c; aai16_ChunkAck a;
        memset(&c, 0, sizeof(c));
        c.transfer_id = 2; c.generation = 2; c.kind = aai16_ModelKind_MODEL_ENVIRONMENT; c.format = aai16_ModelFormat_FORMAT_TFLITE;
        c.model_version = 1; c.seq = 0; c.total_chunks = (uint32_t)((g_pkg_len[1] + 1023) / 1024); c.total_size = (uint32_t)g_pkg_len[1];
        c.sha256.size = 32; memcpy(c.sha256.bytes, g_pkg_digest[1], 32); c.data.size = 1024; memcpy(c.data.bytes, g_pkg[1], 1024);
        model_store_on_chunk(&ms, &c, &a);
        CHECK(a.status == aai16_ChunkStatus_CHUNK_DUPLICATE && F.erases == erases && ms.state == XFER_STAGED);
    }
    CHECK(model_store_activate(&ms, 2, accept_all, NULL) == R(NONE) && ms.active.bank == 1);
    CHECK(model_store_active(&ms, &act) == R(NONE) && act.contract.kind == aai16_ModelKind_MODEL_ENVIRONMENT);

    /* reboot: a new store object sees the last committed model */
    model_store_t ms2;
    model_store_init(&ms2, &OPS);
    CHECK(ms2.has_active && ms2.active.bank == 1 && ms2.active.kind == aai16_ModelKind_MODEL_ENVIRONMENT);

    /* ---- failure scenarios never disturb the active model (environment, bank 1) ---- */
    const uint8_t keep = ms.active.bank;
    aai16_ChunkStatus s;
    s = upload(0, 512, 3, 3, (upload_t){.corrupt_at = 700}, NULL);
    CHECK(s == aai16_ChunkStatus_CHUNK_DIGEST_MISMATCH);          /* corrupt payload byte */
    CHECK(ms.state == XFER_IDLE && ms.active.bank == keep && model_store_active(&ms, &act) == R(NONE));
    CHECK(model_store_activate(&ms, 3, accept_all, NULL) == R(NO_STAGED_MODEL));

    s = upload(0, 512, 4, 4, (upload_t){.digest_from = 2}, NULL);  /* header claims another file's digest */
    CHECK(s == aai16_ChunkStatus_CHUNK_DIGEST_MISMATCH);

    s = upload(0, 512, 5, 5, (upload_t){.kind = aai16_ModelKind_MODEL_ENVIRONMENT}, NULL); /* package is motion */
    CHECK(s == aai16_ChunkStatus_CHUNK_BAD_TRANSFER && ms.state == XFER_IDLE);

    s = upload(0, 512, 6, 6, (upload_t){.version = 7}, NULL);      /* version differs from the contract's */
    CHECK(s == aai16_ChunkStatus_CHUNK_BAD_TRANSFER);

    s = upload(0, 512, 7, 7, (upload_t){.skip_seq = 3}, NULL);     /* lost chunk => out of order */
    CHECK(s == aai16_ChunkStatus_CHUNK_OUT_OF_ORDER);
    {   /* the sender resends the missing chunk and carries on: transfer resumes */
        static aai16_ModelChunk c; aai16_ChunkAck a;
        const uint32_t total = (uint32_t)((g_pkg_len[0] + 511) / 512);
        for (uint32_t seq = 2; seq < total; seq++) {
            memset(&c, 0, sizeof(c));
            c.transfer_id = 7; c.generation = 7; c.kind = aai16_ModelKind_MODEL_MOTION; c.format = aai16_ModelFormat_FORMAT_TFLITE;
            c.model_version = 1; c.seq = seq; c.total_chunks = total; c.total_size = (uint32_t)g_pkg_len[0];
            c.sha256.size = 32; memcpy(c.sha256.bytes, g_pkg_digest[0], 32);
            const size_t off = (size_t)seq * 512, n = g_pkg_len[0] - off < 512 ? g_pkg_len[0] - off : 512;
            c.data.size = (pb_size_t)n; memcpy(c.data.bytes, g_pkg[0] + off, n);
            model_store_on_chunk(&ms, &c, &a);
            CHECK(a.status == aai16_ChunkStatus_CHUNK_OK);
        }
        CHECK(ms.state == XFER_STAGED && ms.transfer_id == 7);
    }

    CHECK(upload(0, 512, 8, 8, (upload_t){.dup_seq = 4}, NULL) == aai16_ChunkStatus_CHUNK_OK);  /* duplicates tolerated */
    CHECK(ms.state == XFER_STAGED);

    fresh_store();                                                  /* interrupted upload + reboot */
    CHECK(upload(0, 512, 1, 1, (upload_t){0}, NULL) == aai16_ChunkStatus_CHUNK_OK);
    CHECK(model_store_activate(&ms, 1, accept_all, NULL) == R(NONE));
    (void)upload(1, 512, 2, 2, (upload_t){.stop_after = 3}, NULL);
    CHECK(ms.state == XFER_RECEIVING);
    model_store_init(&ms, &OPS);                                    /* reboot */
    CHECK(ms.state == XFER_IDLE && ms.has_active && model_store_active(&ms, &act) == R(NONE) &&
          act.contract.kind == aai16_ModelKind_MODEL_MOTION);

    model_store_abort(&ms, 9);                                      /* unknown id: no effect */
    (void)upload(1, 512, 3, 3, (upload_t){.stop_after = 2}, NULL);
    model_store_abort(&ms, 3);
    CHECK(ms.state == XFER_IDLE);

    CHECK(upload(1, 512, 4, 4, (upload_t){0}, NULL) == aai16_ChunkStatus_CHUNK_OK);
    F.fail_write_at = F.writes + 2;                                 /* flash write error mid-upload */
    fresh_store(); /* clear counters, keep going with a store whose active model is none */
    F.fail_write_at = 3;
    CHECK(upload(0, 512, 5, 5, (upload_t){0}, NULL) == aai16_ChunkStatus_CHUNK_STORAGE_ERROR);
    CHECK(ms.state == XFER_IDLE);

    /* an active model whose flash was corrupted is not trusted after reboot */
    fresh_store();
    CHECK(upload(0, 512, 1, 1, (upload_t){0}, NULL) == aai16_ChunkStatus_CHUNK_OK);
    CHECK(model_store_activate(&ms, 1, accept_all, NULL) == R(NONE));
    F.bank[0][g_pkg_len[0] - 5] ^= 0xFF;
    model_store_init(&ms, &OPS);
    CHECK(!ms.has_active);

    /* malformed chunk fields */
    fresh_store();
    static aai16_ModelChunk c; aai16_ChunkAck a;
    memset(&c, 0, sizeof(c));
    c.transfer_id = 1; c.kind = aai16_ModelKind_MODEL_MOTION; c.format = aai16_ModelFormat_FORMAT_TFLITE; c.total_chunks = 2;
    c.total_size = 100; c.seq = 5; c.sha256.size = 32; c.data.size = 10;
    model_store_on_chunk(&ms, &c, &a);
    CHECK(a.status == aai16_ChunkStatus_CHUNK_BAD_LENGTH);          /* seq >= total_chunks */
    c.seq = 1; model_store_on_chunk(&ms, &c, &a);
    CHECK(a.status == aai16_ChunkStatus_CHUNK_BAD_TRANSFER);        /* mid-transfer chunk with no start */
    c.seq = 0; c.format = aai16_ModelFormat_FORMAT_ESPDL; model_store_on_chunk(&ms, &c, &a);
    CHECK(a.status == aai16_ChunkStatus_CHUNK_BAD_LENGTH);          /* unsupported format */
    c.format = aai16_ModelFormat_FORMAT_TFLITE; c.total_size = BANK_SZ + 1; model_store_on_chunk(&ms, &c, &a);
    CHECK(a.status == aai16_ChunkStatus_CHUNK_BAD_LENGTH);          /* larger than a bank */
    c.total_size = 100; c.data.size = 0; model_store_on_chunk(&ms, &c, &a);
    CHECK(a.status == aai16_ChunkStatus_CHUNK_BAD_LENGTH);
    c.data.size = 60; c.seq = 0; model_store_on_chunk(&ms, &c, &a);
    CHECK(a.status == aai16_ChunkStatus_CHUNK_OK);
    c.seq = 1; c.data.size = 60; model_store_on_chunk(&ms, &c, &a);
    CHECK(a.status == aai16_ChunkStatus_CHUNK_BAD_LENGTH && ms.state == XFER_IDLE); /* 120 > total_size */
}

int main(int argc, char **argv)
{
    if (argc < 2) return 2;
    g_dir = argv[1];
    load_packages();
    CHECK(ml_package_parse(g_pkg[0], g_pkg_len[0], &g_motion) == R(NONE));
    CHECK(ml_package_parse(g_pkg[1], g_pkg_len[1], &g_env) == R(NONE));
    /* the parsed packages point into g_pkg, which stays alive for the whole run */

    test_sha256();
    test_golden();
    test_package_parse();
    test_selector_compat();
    test_selector_stream();
    test_store();

    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
