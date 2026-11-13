/* Host tests for Model C (anomaly detector). argv[1] = directory with cases.txt and anomaly.amdl written by run.py.
 * Golden numbers come from the independent Python reference (ml/anomaly.py + tests/anomaly_host/run.py); the package
 * is the real trained artifact. The sensor table below is a bounded test double. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ml_features.h"
#include "ml_package.h"
#include "ml_selector.h"

static int g_checks, g_fail;

#define CHECK(cond)                                                     \
    do {                                                                \
        g_checks++;                                                     \
        if (!(cond)) {                                                  \
            g_fail++;                                                   \
            printf("  FAIL line %d: %s\n", __LINE__, #cond);            \
        }                                                               \
    } while (0)

#define R(x) aai16_ModelReason_MODEL_REASON_##x
#define K(x) aai16_ValueKind_VALUE_##x

static bool near(double a, double b, double abs_tol, double rel_tol)
{
    return fabs(a - b) <= abs_tol + rel_tol * fabs(b);
}

/* ---------------------------------------------------------------- sensor table (test double) */
typedef struct {
    bool active;
    uint32_t period_ms, n;
    aai16_ValueKind kinds[8];
} sensor_t;

static sensor_t g_sensor[16];

static void sensors_reset(void)
{
    memset(g_sensor, 0, sizeof(g_sensor));
    g_sensor[1] = (sensor_t){true, 100, 3, {K(ACCEL_X_G), K(ACCEL_Y_G), K(ACCEL_Z_G)}};         /* GY-511 */
    g_sensor[2] = (sensor_t){true, 2000, 2, {K(HUMIDITY_PCT), K(TEMPERATURE_C)}};                /* DHT22 */
    g_sensor[5] = (sensor_t){true, 500, 3, {K(TEMPERATURE_C), K(PRESSURE_PA), K(HUMIDITY_PCT)}}; /* BME280 */
    g_sensor[7] = (sensor_t){true, 2000, 1, {K(CO2_PPM)}};                                       /* MH-Z19B */
    g_sensor[8] = (sensor_t){true, 250, 1, {K(TEMPERATURE_C)}};                                  /* MAX6675 */
    g_sensor[3] = (sensor_t){true, 100, 3, {K(OTHER), K(RADAR_DISTANCE_MM), K(OTHER)}};          /* LD2450 */
    g_sensor[11] = (sensor_t){true, 10000, 1, {K(CO2_PPM)}};                                     /* far too slow */
}

static bool lookup(uint32_t id, ml_source_t *out, void *ctx)
{
    (void)ctx;
    if (id >= 16 || !g_sensor[id].active) return false;
    memset(out, 0, sizeof(*out));
    out->active = true;
    out->period_ms = g_sensor[id].period_ms;
    out->output_count = g_sensor[id].n;
    memcpy(out->kinds, g_sensor[id].kinds, sizeof(out->kinds));
    return true;
}

static aai16_ModelReason map_inputs(ml_selector_t *s, const uint32_t (*m)[3], uint32_t n)
{
    aai16_InputSource in[12];
    for (uint32_t i = 0; i < n; i++) {
        in[i].feature_slot = m[i][0];
        in[i].sensor_id = m[i][1];
        in[i].value_index = m[i][2];
    }
    return mlsel_set_inputs(s, in, n, lookup, NULL);
}

static char g_dir[512];

static FILE *open_in(const char *name)
{
    char p[700];
    snprintf(p, sizeof(p), "%s/%s", g_dir, name);
    FILE *f = fopen(p, "rb");
    if (!f) {
        printf("cannot open %s\n", p);
        exit(2);
    }
    return f;
}

static uint8_t g_pkg_buf[20000];
static ml_package_t g_pkg;
static double g_s_in, g_s_out, g_threshold;
static int g_z_in, g_z_out;
static unsigned g_max_age_ms, g_period_ms;

/* ------------------------------------------------------------------ package */
static void test_package(void)
{
    puts("the trained anomaly package parses and its contract is what the firmware expects");
    FILE *f = open_in("anomaly.amdl");
    const size_t len = fread(g_pkg_buf, 1, sizeof(g_pkg_buf), f);
    fclose(f);
    const aai16_ModelReason r = ml_package_parse(g_pkg_buf, len, &g_pkg);
    CHECK(r == R(NONE));
    const aai16_ModelContract *c = &g_pkg.contract;
    CHECK(c->kind == aai16_ModelKind_MODEL_ANOMALY);
    CHECK(c->preprocessing == aai16_Preprocessing_PREPROC_SLOT_STATS_V1);
    CHECK(c->feature_count == 32 && c->output_count == 32 && c->channels_count == 10 && c->labels_count == 2);
    CHECK(c->channels[0] == K(ACCEL_X_G) && c->channels[8] == K(RADAR_DISTANCE_MM) && c->channels[9] == K(TEMPERATURE_C));
    CHECK(c->window_samples == 1 && c->stride_samples == 1 && c->anomaly_threshold > 0);

    /* a damaged package is refused: flip one byte of the model */
    static uint8_t bad[20000];
    memcpy(bad, g_pkg_buf, len);
    bad[len - 5] ^= 0xFF;
    ml_package_t p2;
    CHECK(ml_package_parse(bad, len, &p2) == R(DIGEST_MISMATCH));
}

/* ------------------------------------------------------------ slot statistics vs Python */
static void test_series_and_vectors(void)
{
    puts("slot statistics, scoring and per-slot errors match the Python reference");
    FILE *f = open_in("cases.txt");
    char tag[4];
    int series = 0, vectors = 0;
    static double t[400], x[400];
    while (fscanf(f, "%3s", tag) == 1) {
        if (tag[0] == 'Q') {
            double s_in, s_out, thr;
            int z_in, z_out;
            unsigned age, per;
            if (fscanf(f, "%lf %d %lf %d %lf %u %u", &s_in, &z_in, &s_out, &z_out, &thr, &age, &per) != 7) exit(2);
            g_s_in = s_in; g_z_in = z_in; g_s_out = s_out; g_z_out = z_out; g_threshold = thr;
            g_max_age_ms = age; g_period_ms = per;
        } else if (tag[0] == 'S') {
            int slot, n;
            if (fscanf(f, "%d %d", &slot, &n) != 2) exit(2);
            for (int i = 0; i < n; i++) if (fscanf(f, "%lf", &t[i]) != 1) exit(2);
            for (int i = 0; i < n; i++) if (fscanf(f, "%lf", &x[i]) != 1) exit(2);
            double want[4];
            for (int i = 0; i < 4; i++) if (fscanf(f, "%lf", &want[i]) != 1) exit(2);
            ml_slot_t st;
            ml_slot_reset(&st);
            for (int i = 0; i < n; i++) ml_slot_update(&st, x[i], t[i]);
            float got[4];
            ml_slot_features(&st, (uint32_t)slot, got);
            bool ok = st.n == (uint32_t)n;
            for (int i = 0; i < 4; i++) ok = ok && near(got[i], want[i], 3e-5, 1e-5);
            CHECK(ok);
            series++;
        } else if (tag[0] == 'V') {
            unsigned mask, anomaly;
            double score;
            if (fscanf(f, "%u %u %lf", &mask, &anomaly, &score) != 3) exit(2);
            int8_t in[32], out[32];
            double err[8];
            for (int i = 0; i < 32; i++) { int v; if (fscanf(f, "%d", &v) != 1) exit(2); in[i] = (int8_t)v; }
            for (int i = 0; i < 32; i++) { int v; if (fscanf(f, "%d", &v) != 1) exit(2); out[i] = (int8_t)v; }
            for (int i = 0; i < 8; i++) if (fscanf(f, "%lf", &err[i]) != 1) exit(2);
            float got_score, got_err[8];
            CHECK(ml_anomaly_score(&g_pkg.contract, in, out, mask, &got_score, got_err));
            bool ok = near(got_score, score, 1e-7, 2e-4);
            for (int i = 0; i < 8; i++) ok = ok && near(got_err[i], err[i], 1e-7, 2e-4);
            CHECK(ok);
            CHECK((got_score > g_pkg.contract.anomaly_threshold) == (anomaly != 0));
            vectors++;
        } else {
            break; /* scenario records follow */
        }
    }
    fclose(f);
    CHECK(series == 16 && vectors == 16);
    CHECK(near(g_pkg.contract.anomaly_threshold, g_threshold, 1e-9, 1e-5));
    float s, e[8];
    int8_t z[32] = {0};
    CHECK(!ml_anomaly_score(&g_pkg.contract, z, z, 0, &s, e)); /* no slot present: refused */
}

/* ---------------------------------------------------------------- mapping rules */
static ml_selector_t g_sel;

static void test_mapping(void)
{
    puts("partial mappings are accepted; wrong kinds, partial motion, duplicates and slow sensors are refused");
    sensors_reset();
    mlsel_init(&g_sel);
    mlsel_set_contract(&g_sel, &g_pkg.contract);
    CHECK(g_sel.state == R(NO_INPUTS));
    const uint32_t ok1[][3] = {{3, 5, 0}, {4, 5, 2}};  /* temperature + humidity from the BME280 only */
    CHECK(map_inputs(&g_sel, ok1, 2) == R(NONE));
    CHECK(mlsel_channel_mapped(&g_sel, 3) && mlsel_channel_mapped(&g_sel, 4) && !mlsel_channel_mapped(&g_sel, 5));

    const uint32_t bad_kind[][3] = {{3, 5, 1}};        /* pressure into the temperature channel */
    CHECK(map_inputs(&g_sel, bad_kind, 1) == R(VALUE_KIND_MISMATCH));
    const uint32_t motion_part[][3] = {{0, 1, 0}, {1, 1, 1}};
    CHECK(map_inputs(&g_sel, motion_part, 2) == R(INPUT_COUNT_MISMATCH));
    const uint32_t dup[][3] = {{3, 5, 0}, {3, 8, 0}};
    CHECK(map_inputs(&g_sel, dup, 2) == R(DUPLICATE_SLOT));
    const uint32_t range[][3] = {{10, 5, 0}};
    CHECK(map_inputs(&g_sel, range, 1) == R(BAD_SLOT));
    const uint32_t absent[][3] = {{3, 9, 0}};
    CHECK(map_inputs(&g_sel, absent, 1) == R(SENSOR_NOT_ACTIVE));
    const uint32_t slow[][3] = {{6, 11, 0}};
    CHECK(map_inputs(&g_sel, slow, 1) == R(PERIOD_MISMATCH));
    const uint32_t vidx[][3] = {{3, 5, 3}};
    CHECK(map_inputs(&g_sel, vidx, 1) == R(VALUE_INDEX_RANGE));
    CHECK(map_inputs(&g_sel, ok1, 0) == R(INPUT_COUNT_MISMATCH));
    const uint32_t radar[][3] = {{8, 3, 1}};           /* LD2450 distance into the radar channel */
    CHECK(map_inputs(&g_sel, radar, 1) == R(NONE));
    const uint32_t radar_bad[][3] = {{8, 3, 0}};       /* a value of kind OTHER is not a radar distance */
    CHECK(map_inputs(&g_sel, radar_bad, 1) == R(VALUE_KIND_MISMATCH));
    /* a refused request leaves the previous mapping in force */
    CHECK(mlsel_channel_mapped(&g_sel, 8) && !mlsel_channel_mapped(&g_sel, 3));
}

/* -------------------------------------------------------------------- scenario */
typedef struct {
    uint32_t sensor, n;
    bool valid;
    uint64_t ts;
    float v[8];
} ev_t;

static ev_t g_ev[2000];
static uint32_t g_nev;

static void test_scenario(void)
{
    puts("readings from three sensors produce exactly the inferences, masks and features of the Python reference");
    FILE *f = open_in("cases.txt");
    char tag[4];
    static float want_f[100][32];
    static uint64_t want_ts[100];
    static uint32_t want_mask[100];
    uint32_t nwant = 0;
    while (fscanf(f, "%3s", tag) == 1) {
        if (tag[0] == 'E') {
            ev_t *e = &g_ev[g_nev++];
            unsigned sid, valid, n;
            unsigned long long ts;
            if (fscanf(f, "%u %u %llu %u", &sid, &valid, &ts, &n) != 4) exit(2);
            e->sensor = sid; e->valid = valid != 0; e->ts = ts; e->n = n;
            for (unsigned i = 0; i < n; i++) { double d; if (fscanf(f, "%lf", &d) != 1) exit(2); e->v[i] = (float)d; }
        } else if (tag[0] == 'X') {
            unsigned long long ts;
            unsigned mask;
            if (fscanf(f, "%llu %u", &ts, &mask) != 2) exit(2);
            want_ts[nwant] = ts; want_mask[nwant] = mask;
            for (int i = 0; i < 32; i++) { double d; if (fscanf(f, "%lf", &d) != 1) exit(2); want_f[nwant][i] = (float)d; }
            nwant++;
        }
    }
    fclose(f);
    CHECK(g_nev > 450 && nwant > 20);

    sensors_reset();
    mlsel_init(&g_sel);
    mlsel_set_contract(&g_sel, &g_pkg.contract);
    const uint32_t m[][3] = {{0, 1, 0}, {1, 1, 1}, {2, 1, 2}, {3, 5, 0}, {4, 5, 2}, {5, 5, 1}, {6, 7, 0}};
    CHECK(map_inputs(&g_sel, m, 7) == R(NONE));

    uint32_t got = 0, mismatched = 0, mask_seen_without_co2 = 0, mask_seen_after_reset = 0;
    for (uint32_t i = 0; i < g_nev; i++) {
        bool ready = false;
        float feat[ML_MAX_FEATURES];
        (void)mlsel_push(&g_sel, g_ev[i].sensor, g_ev[i].ts, g_ev[i].valid, g_ev[i].v, g_ev[i].n, lookup, NULL, &ready, feat);
        if (!ready) continue;
        if (got < nwant) {
            bool ok = want_ts[got] == g_ev[i].ts && want_mask[got] == g_sel.active_mask;
            for (int k = 0; k < 32; k++) ok = ok && near(feat[k], want_f[got][k], 3e-5, 1e-5);
            if (!ok) {
                if (mismatched == 0) {
                    printf("  first mismatch at inference #%u: ts C=%llu py=%llu, mask C=%u py=%u\n", got,
                           (unsigned long long)g_ev[i].ts, (unsigned long long)want_ts[got], g_sel.active_mask, want_mask[got]);
                    for (int k = 0; k < 32; k++)
                        if (!near(feat[k], want_f[got][k], 3e-5, 1e-5)) printf("    f[%d] C=%.7g py=%.7g\n", k, feat[k], want_f[got][k]);
                }
                mismatched++;
            }
            if (!(want_mask[got] & (1u << 4))) mask_seen_without_co2++;
            if (!(want_mask[got] & (1u << 1))) mask_seen_after_reset++;
        }
        got++;
    }
    CHECK(got == nwant);
    CHECK(mismatched == 0);
    CHECK(mask_seen_without_co2 > 3);   /* the silent CO2 sensor really dropped out of the mask */
    CHECK(mask_seen_after_reset >= 1);  /* the invalid BME280 reading removed its slots for a while */
}

/* ------------------------------------------------- sensors disappearing, rule-requested inference */
static void test_revalidate(void)
{
    puts("a vanished sensor only removes its slots; with none left the model is suspended; it resumes cleanly");
    sensors_reset();
    mlsel_init(&g_sel);
    mlsel_set_contract(&g_sel, &g_pkg.contract);
    const uint32_t m[][3] = {{3, 5, 0}, {4, 5, 2}, {6, 7, 0}};
    CHECK(map_inputs(&g_sel, m, 3) == R(NONE));
    float feat[ML_MAX_FEATURES];
    bool ready = false;
    uint64_t t = 1000000;
    for (int k = 0; k < 12; k++, t += 500000) {
        const float v5[3] = {25.0f, 101300.0f, 60.0f}, v7[1] = {600.0f};
        (void)mlsel_push(&g_sel, 5, t, true, v5, 3, lookup, NULL, &ready, feat);
        if (k % 4 == 0) (void)mlsel_push(&g_sel, 7, t, true, v7, 1, lookup, NULL, &ready, feat);
    }
    CHECK(mlsel_current_features(&g_sel, feat));
    CHECK(g_sel.active_mask == ((1u << 1) | (1u << 2)) || g_sel.active_mask == ((1u << 1) | (1u << 2) | (1u << 4)));

    g_sensor[5].active = false;                                     /* BME280 removed from the configuration */
    CHECK(mlsel_revalidate(&g_sel, lookup, NULL) == R(NONE));       /* CO2 is still mapped and running */
    CHECK(!g_sel.suspended && g_sel.slot[1].n == 0 && g_sel.slot[2].n == 0);
    g_sensor[7].active = false;
    CHECK(mlsel_revalidate(&g_sel, lookup, NULL) == R(SENSOR_NOT_ACTIVE));
    CHECK(g_sel.suspended && !mlsel_current_features(&g_sel, feat));
    g_sensor[5].active = true; g_sensor[7].active = true;
    CHECK(mlsel_revalidate(&g_sel, lookup, NULL) == R(NONE) && !g_sel.suspended);
    CHECK(g_sel.slot[4].n == 0); /* nothing older than the suspension is reused */
}

int main(int argc, char **argv)
{
    if (argc < 2) return 2;
    snprintf(g_dir, sizeof(g_dir), "%s", argv[1]);
    test_package();
    test_series_and_vectors();
    test_mapping();
    test_scenario();
    test_revalidate();
    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
