/* Host tests: rule engine, bridge, heartbeat, REPL. Hooks are test doubles that record calls. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bridge.h"
#include "config_manager.h"
#include "diag.h"
#include "pb_decode.h"
#include "repl.h"
#include "rule_engine.h"

static int g_checks, g_fail;
static const char *g_dir;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fail++; printf("  FAIL line %d: %s\n", __LINE__, #cond); } } while (0)

static uint8_t g_buf[8192];
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

/* ------------------------------------------------------------ rule hooks */
static uint64_t g_now;
static struct {
    int notify_n; uint32_t notify_rule; char notify_text[40]; double notify_value;
    int set_n, toggle_n, pulse_n, pwm_n, ml_n, remote_n;
    uint32_t pin; bool level; uint32_t ms, permille;
    char node[20]; aai16_RemoteCmd cmd;
    bool allow_10, allow_11, allow_12;
} H;

static void h_set(void *c, uint32_t p, bool l) { (void)c; H.set_n++; H.pin = p; H.level = l; }
static void h_toggle(void *c, uint32_t p) { (void)c; H.toggle_n++; H.pin = p; }
static void h_pulse(void *c, uint32_t p, bool l, uint32_t ms) { (void)c; H.pulse_n++; H.pin = p; H.level = l; H.ms = ms; }
static void h_pwm(void *c, uint32_t p, uint32_t pm) { (void)c; H.pwm_n++; H.pin = p; H.permille = pm; }
static void h_notify(void *c, uint32_t id, const char *t, double v) { (void)c; H.notify_n++; H.notify_rule = id; snprintf(H.notify_text, sizeof(H.notify_text), "%s", t); H.notify_value = v; }
static void h_ml(void *c) { (void)c; H.ml_n++; }
static void h_remote(void *c, const char *n, aai16_RemoteCmd cmd) { (void)c; H.remote_n++; snprintf(H.node, sizeof(H.node), "%s", n); H.cmd = cmd; }
static bool h_pin_ok(void *c, uint32_t p) { (void)c; return (p == 10 && H.allow_10) || (p == 11 && H.allow_11) || (p == 12 && H.allow_12); }
static uint64_t h_now(void *c) { (void)c; return g_now; }
static const re_hooks_t HK = {h_set, h_toggle, h_pulse, h_pwm, h_notify, h_ml, h_remote, h_pin_ok, h_now, NULL};

static rule_engine_t re;
static aai16_RuleSetApply g_set;

static void fresh_engine(void)
{
    memset(&H, 0, sizeof(H));
    H.allow_10 = H.allow_11 = H.allow_12 = true;
    g_now = 1000000;
    re_init(&re, &HK);
}

static aai16_Rule *add_rule(uint32_t id, uint32_t cooldown)
{
    aai16_Rule *r = &g_set.rules[g_set.rules_count++];
    memset(r, 0, sizeof(*r));
    r->rule_id = id; r->enabled = true; r->cooldown_ms = cooldown;
    return r;
}
static void var(aai16_Rule *r, uint32_t sensor, uint32_t idx, aai16_RuleStat st, uint32_t win)
{
    aai16_RuleVar *v = &r->vars[r->vars_count++];
    v->sensor_id = sensor; v->value_index = idx; v->stat = st; v->window = win;
}
static void tok(aai16_Rule *r, aai16_CalcOp op, uint32_t reg, float value)
{
    aai16_CalcToken *t = &r->when[r->when_count++];
    t->op = op; t->reg = reg; t->value = value;
}
static void new_set(uint32_t gen) { memset(&g_set, 0, sizeof(g_set)); g_set.generation = gen; }
static void reading(uint32_t sensor, float a, float b) { float v[2] = {a, b}; re_on_reading(&re, sensor, true, v, 2); }

/* ------------------------------------------------------------------ tests */
static void test_rule_from_python(void)
{
    puts("rule engine: Python-encoded rule applies, fires with cooldown, ignores invalid readings");
    fresh_engine();
    aai16_RuleSetApply set = aai16_RuleSetApply_init_zero;
    pb_istream_t is = pb_istream_from_buffer(g_buf, load("rules_ok.bin"));
    CHECK(pb_decode(&is, aai16_RuleSetApply_fields, &set));
    re_error_t e;
    CHECK(re_apply(&re, &set, &e) && re.count == 1 && re.generation == 1);

    reading(2, 60.0f, 29.0f);
    CHECK(H.notify_n == 0);
    reading(2, 60.0f, 31.0f);
    CHECK(H.notify_n == 1 && H.notify_rule == 1 && strcmp(H.notify_text, "hot") == 0 && H.notify_value == 31.0);
    g_now += 500000; reading(2, 60.0f, 33.0f);
    CHECK(H.notify_n == 1);                           /* cooldown 1 s */
    g_now += 600000; reading(2, 60.0f, 33.0f);
    CHECK(H.notify_n == 2);

    g_now += 2000000;
    float v[2] = {60.0f, 40.0f};
    re_on_reading(&re, 2, false, v, 2);               /* invalid reading must not evaluate */
    CHECK(H.notify_n == 2);
    reading(1, 0, 99.0f);                             /* other sensor: untouched */
    CHECK(H.notify_n == 2);
    re_on_reading(&re, 2, true, v, 1);                /* value_index 1 missing => no data */
    CHECK(H.notify_n == 2);
}

static void test_stats_edge_actions(void)
{
    puts("rule engine: window statistics, edge trigger, GPIO/PWM/ML/remote actions");
    fresh_engine();
    new_set(1);
    aai16_Rule *r = add_rule(1, 100);          /* mean of 4 temperature samples >= 28 -> PWM */
    var(r, 2, 1, aai16_RuleStat_STAT_MEAN, 4);
    tok(r, aai16_CalcOp_CALC_PUSH_REG, 0, 0); tok(r, aai16_CalcOp_CALC_PUSH_CONST, 0, 28.0f); tok(r, aai16_CalcOp_CALC_GE, 0, 0);
    r->action.type = aai16_RuleActionType_RACT_PWM; r->action.pin = 11; r->action.duty_permille = 500;

    r = add_rule(2, 100);                      /* |delta| of accel x > 0.5, edge triggered -> pulse */
    r->edge_triggered = true;
    var(r, 1, 0, aai16_RuleStat_STAT_DELTA, 2);
    tok(r, aai16_CalcOp_CALC_PUSH_REG, 0, 0); tok(r, aai16_CalcOp_CALC_ABS, 0, 0); tok(r, aai16_CalcOp_CALC_PUSH_CONST, 0, 0.5f); tok(r, aai16_CalcOp_CALC_GT, 0, 0);
    r->action.type = aai16_RuleActionType_RACT_GPIO_PULSE; r->action.pin = 10; r->action.level = true; r->action.duration_ms = 250;

    r = add_rule(3, 100);                      /* stddev over 8 samples > 0.1 AND latest < 5 -> ML trigger */
    var(r, 1, 1, aai16_RuleStat_STAT_STDDEV, 8); var(r, 1, 1, aai16_RuleStat_STAT_LATEST, 1);
    tok(r, aai16_CalcOp_CALC_PUSH_REG, 0, 0); tok(r, aai16_CalcOp_CALC_PUSH_CONST, 0, 0.1f); tok(r, aai16_CalcOp_CALC_GT, 0, 0);
    tok(r, aai16_CalcOp_CALC_PUSH_REG, 1, 0); tok(r, aai16_CalcOp_CALC_PUSH_CONST, 0, 5.0f); tok(r, aai16_CalcOp_CALC_LT, 0, 0);
    tok(r, aai16_CalcOp_CALC_AND, 0, 0);
    r->action.type = aai16_RuleActionType_RACT_ML_TRIGGER;

    r = add_rule(4, 1000);                     /* NOT(temp > 0) false...; here: temp < 0 -> remote STOP */
    var(r, 2, 1, aai16_RuleStat_STAT_LATEST, 1);
    tok(r, aai16_CalcOp_CALC_PUSH_REG, 0, 0); tok(r, aai16_CalcOp_CALC_PUSH_CONST, 0, 0.0f); tok(r, aai16_CalcOp_CALC_GT, 0, 0); tok(r, aai16_CalcOp_CALC_NOT, 0, 0);
    r->action.type = aai16_RuleActionType_RACT_REMOTE_CMD; strcpy(r->action.target_node, "nodeA"); r->action.cmd = aai16_RemoteCmd_RCMD_STOP;
    re_error_t e;
    CHECK(re_apply(&re, &g_set, &e));

    /* mean needs a FULL window of 4 */
    for (int i = 0; i < 3; i++) { g_now += 200000; reading(2, 50, 30.0f); }
    CHECK(H.pwm_n == 0);
    g_now += 200000; reading(2, 50, 30.0f);
    CHECK(H.pwm_n == 1 && H.pin == 11 && H.permille == 500);

    /* delta needs 2 samples; edge trigger fires once while it stays true */
    g_now += 200000; reading(1, 0.0f, 1.0f);
    CHECK(H.pulse_n == 0);
    g_now += 200000; reading(1, 1.0f, 1.0f);          /* delta = 1.0 > 0.5 */
    CHECK(H.pulse_n == 1 && H.pin == 10 && H.level && H.ms == 250);
    g_now += 200000; reading(1, 2.0f, 1.0f);          /* still true, edge already used */
    CHECK(H.pulse_n == 1);
    g_now += 200000; reading(1, 2.0f, 1.0f);          /* delta 0: false */
    g_now += 200000; reading(1, 3.0f, 1.0f);          /* true again => fires again */
    CHECK(H.pulse_n == 2);

    /* ML trigger needs 8 samples of variation and latest < 5 */
    CHECK(H.ml_n == 0);
    for (int i = 0; i < 8; i++) { g_now += 200000; reading(1, 0.0f, (i % 2) ? 2.0f : 0.0f); }
    CHECK(H.ml_n >= 1);

    /* NOT(temp > 0) */
    g_now += 2000000; reading(2, 50, -3.0f);
    CHECK(H.remote_n == 1 && strcmp(H.node, "nodeA") == 0 && H.cmd == aai16_RemoteCmd_RCMD_STOP);
    g_now += 100000; reading(2, 50, -3.0f);
    CHECK(H.remote_n == 1);                           /* 1 s cooldown for remote commands */

    /* pin policy is re-checked at fire time: a sensor claimed GPIO 11 after the rule was accepted */
    H.allow_11 = false;
    const int errs = (int)re.errors, pwm = H.pwm_n;
    for (int i = 0; i < 4; i++) { g_now += 200000; reading(2, 50, 30.0f); }
    CHECK(H.pwm_n == pwm && (int)re.errors > errs);
}

typedef struct { const char *name; aai16_Reason reason; void (*build)(void); } rneg_t;
static aai16_Rule *good_rule(void)
{
    aai16_Rule *r = add_rule(1, 1000);
    var(r, 2, 1, aai16_RuleStat_STAT_LATEST, 1);
    tok(r, aai16_CalcOp_CALC_PUSH_REG, 0, 0); tok(r, aai16_CalcOp_CALC_PUSH_CONST, 0, 30.0f); tok(r, aai16_CalcOp_CALC_GT, 0, 0);
    r->action.type = aai16_RuleActionType_RACT_LOG;
    return r;
}
static void n_underflow(void) { aai16_Rule *r = good_rule(); r->when_count = 2; r->when[1].op = aai16_CalcOp_CALC_ADD; r->when[1].reg = 0; }
static void n_leftover(void) { aai16_Rule *r = good_rule(); tok(r, aai16_CalcOp_CALC_PUSH_CONST, 0, 1.0f); }
static void n_badreg(void) { aai16_Rule *r = good_rule(); r->when[0].reg = 3; }
static void n_nan(void) { aai16_Rule *r = good_rule(); r->when[1].value = NAN; }
static void n_nop(void) { aai16_Rule *r = good_rule(); r->when[2].op = aai16_CalcOp_CALC_NOP; }
static void n_empty_when(void) { aai16_Rule *r = good_rule(); r->when_count = 0; }
static void n_novars(void) { aai16_Rule *r = good_rule(); r->vars_count = 0; }
static void n_win0(void) { aai16_Rule *r = good_rule(); r->vars[0].window = 0; }
static void n_win17(void) { aai16_Rule *r = good_rule(); r->vars[0].window = 17; }
static void n_delta1(void) { aai16_Rule *r = good_rule(); r->vars[0].stat = aai16_RuleStat_STAT_DELTA; r->vars[0].window = 1; }
static void n_sensor0(void) { aai16_Rule *r = good_rule(); r->vars[0].sensor_id = 0; }
static void n_vidx(void) { aai16_Rule *r = good_rule(); r->vars[0].value_index = 8; }
static void n_cool(void) { aai16_Rule *r = good_rule(); r->cooldown_ms = 50; }
static void n_dup(void) { good_rule(); good_rule(); }
static void n_id0(void) { aai16_Rule *r = good_rule(); r->rule_id = 0; }
static void n_noaction(void) { aai16_Rule *r = good_rule(); r->action.type = aai16_RuleActionType_RACT_UNSPECIFIED; }
static void n_pin(void) { aai16_Rule *r = good_rule(); r->action.type = aai16_RuleActionType_RACT_GPIO_SET; r->action.pin = 4; }
static void n_pulse0(void) { aai16_Rule *r = good_rule(); r->action.type = aai16_RuleActionType_RACT_GPIO_PULSE; r->action.pin = 10; r->action.duration_ms = 0; }
static void n_pwm(void) { aai16_Rule *r = good_rule(); r->action.type = aai16_RuleActionType_RACT_PWM; r->action.pin = 10; r->action.duty_permille = 1001; }
static void n_remote_cool(void) { aai16_Rule *r = good_rule(); r->action.type = aai16_RuleActionType_RACT_REMOTE_CMD; strcpy(r->action.target_node, "x"); r->action.cmd = aai16_RemoteCmd_RCMD_RUN; r->cooldown_ms = 500; }
static void n_remote_node(void) { aai16_Rule *r = good_rule(); r->action.type = aai16_RuleActionType_RACT_REMOTE_CMD; r->action.cmd = aai16_RemoteCmd_RCMD_RUN; }
static void n_toomany(void) { for (int i = 0; i < 8; i++) good_rule()->rule_id = (uint32_t)i + 1; g_set.rules_count = 9; }

static const rneg_t RNEG[] = {
    {"stack underflow", aai16_Reason_REASON_BAD_RULE, n_underflow}, {"two values left", aai16_Reason_REASON_BAD_RULE, n_leftover},
    {"variable index out of range", aai16_Reason_REASON_BAD_RULE, n_badreg}, {"NaN constant", aai16_Reason_REASON_BAD_RULE, n_nan},
    {"NOP token", aai16_Reason_REASON_BAD_RULE, n_nop}, {"empty when", aai16_Reason_REASON_BAD_RULE, n_empty_when},
    {"no variables", aai16_Reason_REASON_BAD_RULE, n_novars}, {"window 0", aai16_Reason_REASON_BAD_RULE, n_win0},
    {"window 17", aai16_Reason_REASON_BAD_RULE, n_win17}, {"DELTA with window 1", aai16_Reason_REASON_BAD_RULE, n_delta1},
    {"sensor id 0", aai16_Reason_REASON_BAD_RULE, n_sensor0}, {"value index 8", aai16_Reason_REASON_BAD_RULE, n_vidx},
    {"cooldown 50 ms", aai16_Reason_REASON_BAD_RULE, n_cool}, {"duplicate rule id", aai16_Reason_REASON_BAD_RULE, n_dup},
    {"rule id 0", aai16_Reason_REASON_BAD_RULE, n_id0}, {"unspecified action", aai16_Reason_REASON_BAD_ACTION, n_noaction},
    {"GPIO on a forbidden pin", aai16_Reason_REASON_FORBIDDEN_OUTPUT_PIN, n_pin}, {"pulse 0 ms", aai16_Reason_REASON_BAD_ACTION, n_pulse0},
    {"pwm 1001", aai16_Reason_REASON_BAD_ACTION, n_pwm}, {"remote cmd cooldown 500 ms", aai16_Reason_REASON_BAD_ACTION, n_remote_cool},
    {"remote cmd without node", aai16_Reason_REASON_BAD_ACTION, n_remote_node}, {"9 rules", aai16_Reason_REASON_TOO_MANY_RULES, n_toomany},
};

static void test_rule_validation(void)
{
    puts("rule engine: invalid rule sets are rejected wholesale; the running set survives");
    for (size_t i = 0; i < sizeof(RNEG) / sizeof(RNEG[0]); i++) {
        fresh_engine();
        new_set(1);
        good_rule();
        re_error_t e;
        CHECK(re_apply(&re, &g_set, &e));              /* baseline running set */
        new_set(2);
        RNEG[i].build();
        const bool ok = re_apply(&re, &g_set, &e);
        g_checks++;
        if (ok || e.reason != RNEG[i].reason || re.generation != 1 || re.count != 1) {
            g_fail++;
            printf("  FAIL [%s]: applied=%d reason=%d (want %d) gen=%u\n", RNEG[i].name, ok, e.reason, RNEG[i].reason, re.generation);
        }
        reading(2, 50, 31.0f);                         /* old rule still active */
        CHECK(H.notify_n == 1);
    }
    fresh_engine();
    new_set(5); good_rule();
    re_error_t e;
    CHECK(re_apply(&re, &g_set, &e));
    new_set(5); good_rule();
    CHECK(!re_apply(&re, &g_set, &e) && e.reason == aai16_Reason_REASON_DUPLICATE_GENERATION);
    new_set(4); good_rule();
    CHECK(!re_apply(&re, &g_set, &e) && e.reason == aai16_Reason_REASON_STALE_GENERATION);
}

/* ---------------------------------------------------------------- bridge */
static bridge_t br;
static aai16_BridgeApply g_br;

static bool br_apply_file(const char *file, aai16_Reason *r, uint32_t *i)
{
    memset(&g_br, 0, sizeof(g_br));
    pb_istream_t is = pb_istream_from_buffer(g_buf, load(file));
    if (!pb_decode(&is, aai16_BridgeApply_fields, &g_br)) { puts("  fixture decode failed"); exit(2); }
    return bridge_apply(&br, &g_br, r, i);
}

static void bset(uint32_t gen, uint32_t local, const char *node, uint32_t remote)
{
    memset(&g_br, 0, sizeof(g_br));
    g_br.generation = gen; g_br.sources_count = 1;
    aai16_VirtualSource *s = &g_br.sources[0];
    s->local_sensor_id = local; strcpy(s->remote_node, node); s->remote_sensor_id = remote;
    s->max_age_ms = 5000; s->period_ms = 2000; s->kinds_count = 2;
    s->kinds[0] = aai16_ValueKind_VALUE_HUMIDITY_PCT; s->kinds[1] = aai16_ValueKind_VALUE_TEMPERATURE_C;
}

static void test_bridge(void)
{
    puts("bridge: remote readings become fresh-or-invalid virtual sensors; loops and bad configs refused");
    bridge_init(&br, "nodeB");
    aai16_Reason reason; uint32_t idx;
    CHECK(br_apply_file("bridge_ok.bin", &reason, &idx) && br.count == 1);

    static aai16_Reading out;
    const uint64_t t0 = 50000000;
    size_t n = load("remote_ok.bin");
    CHECK(bridge_on_remote(&br, "nodeA", g_buf, n, t0, &out) == BR_OK);
    CHECK(out.sensor_id == 130 && out.valid && out.values_count == 2 && fabsf(out.values[0] - 65.2f) < 1e-4f && out.timestamp_us == t0);
    CHECK(bridge_on_remote(&br, "nodeZ", g_buf, n, t0, &out) == BR_UNKNOWN_SOURCE);
    n = load("remote_other_sensor.bin");
    CHECK(bridge_on_remote(&br, "nodeA", g_buf, n, t0, &out) == BR_UNKNOWN_SOURCE);
    n = load("remote_garbage.bin");
    CHECK(bridge_on_remote(&br, "nodeA", g_buf, n, t0, &out) == BR_MALFORMED);
    n = load("remote_shape.bin");
    CHECK(bridge_on_remote(&br, "nodeA", g_buf, n, t0, &out) == BR_SHAPE_MISMATCH);
    n = load("remote_invalid.bin");
    CHECK(bridge_on_remote(&br, "nodeA", g_buf, n, t0, &out) == BR_REMOTE_INVALID && !out.valid && out.sensor_id == 130);

    /* freshness: silent source is reported invalid ONCE, then recovers on the next reading */
    aai16_Reading st[2];
    CHECK(bridge_poll(&br, t0 + 4000000, st, 2) == 0);
    CHECK(bridge_poll(&br, t0 + 6000000, st, 2) == 1 && st[0].sensor_id == 130 && !st[0].valid && st[0].error == aai16_ReadError_READ_TIMEOUT);
    CHECK(bridge_poll(&br, t0 + 9000000, st, 2) == 0);
    n = load("remote_ok.bin");
    CHECK(bridge_on_remote(&br, "nodeA", g_buf, n, t0 + 10000000, &out) == BR_OK);
    CHECK(bridge_poll(&br, t0 + 20000000, st, 2) == 1);

    /* virtual sensors feed rules like any other sensor */
    fresh_engine();
    new_set(1);
    aai16_Rule *r = good_rule();
    r->vars[0].sensor_id = 130;
    CHECK(re_apply(&re, &g_set, &(re_error_t){0}));
    n = load("remote_ok.bin");
    bridge_on_remote(&br, "nodeA", g_buf, n, t0, &out);
    out.values[1] = 35.0f;
    re_on_reading(&re, out.sensor_id, out.valid, out.values, out.values_count);
    CHECK(H.notify_n == 1 && H.notify_value == 35.0);

    /* ML selector can validate against virtual sensors through bridge_lookup */
    ml_source_t src;
    CHECK(bridge_lookup(&br, 130, &src) && src.output_count == 2 && src.kinds[1] == aai16_ValueKind_VALUE_TEMPERATURE_C && src.period_ms == 2000);
    CHECK(!bridge_lookup(&br, 131, &src));

    /* validation */
    struct { const char *name; void (*mut)(void); } bad[] = {{"x", NULL}};
    (void)bad;
    bridge_init(&br, "nodeB");
    bset(1, 130, "nodeB", 2);  CHECK(!bridge_apply(&br, &g_br, &reason, &idx) && reason == aai16_Reason_REASON_BAD_BRIDGE); /* ourselves => loop */
    bset(1, 100, "nodeA", 2);  CHECK(!bridge_apply(&br, &g_br, &reason, &idx));                                           /* not a virtual id */
    bset(1, 256, "nodeA", 2);  CHECK(!bridge_apply(&br, &g_br, &reason, &idx));
    bset(1, 130, "nodeA", 130); CHECK(!bridge_apply(&br, &g_br, &reason, &idx));                                          /* no multi-hop */
    bset(1, 130, "nodeA", 0);  CHECK(!bridge_apply(&br, &g_br, &reason, &idx));
    bset(1, 130, "", 2);       CHECK(!bridge_apply(&br, &g_br, &reason, &idx));
    bset(1, 130, "nodeA", 2); g_br.sources[0].max_age_ms = 50; CHECK(!bridge_apply(&br, &g_br, &reason, &idx));
    bset(1, 130, "nodeA", 2); g_br.sources[0].kinds_count = 0; CHECK(!bridge_apply(&br, &g_br, &reason, &idx));
    bset(1, 130, "nodeA", 2); g_br.sources[0].kinds[0] = (aai16_ValueKind)0; CHECK(!bridge_apply(&br, &g_br, &reason, &idx));
    bset(1, 130, "nodeA", 2); g_br.sources[1] = g_br.sources[0]; g_br.sources[1].local_sensor_id = 131; g_br.sources_count = 2;
    CHECK(!bridge_apply(&br, &g_br, &reason, &idx) && idx == 1);                                                          /* same remote twice */
    bset(1, 130, "nodeA", 2); g_br.sources[1] = g_br.sources[0]; g_br.sources[1].remote_sensor_id = 3; g_br.sources_count = 2;
    CHECK(!bridge_apply(&br, &g_br, &reason, &idx) && idx == 1);                                                          /* same local id twice */
    CHECK(br.count == 0 && br.generation == 0);                                                                           /* nothing leaked in */
    bset(2, 130, "nodeA", 2); CHECK(bridge_apply(&br, &g_br, &reason, &idx));
    bset(2, 131, "nodeA", 2); CHECK(!bridge_apply(&br, &g_br, &reason, &idx) && reason == aai16_Reason_REASON_DUPLICATE_GENERATION);
    bset(1, 131, "nodeA", 2); CHECK(!bridge_apply(&br, &g_br, &reason, &idx) && reason == aai16_Reason_REASON_STALE_GENERATION);
    CHECK(bridge_is_virtual(128) && bridge_is_virtual(255) && !bridge_is_virtual(127));
}

/* ------------------------------------------------------------------ misc */
static void test_pin_in_use(void)
{
    puts("config_manager: cm_pin_in_use sees pins of running sensors");
    static config_manager_t cm;
    cm_init(&cm, &(cm_hooks_t){0});
    CHECK(!cm_pin_in_use(&cm, 8));
    aai16_ConfigApply req = aai16_ConfigApply_init_zero;
    req.generation = 1; req.profiles_count = 1;
    aai16_SensorProfile *p = &req.profiles[0];
    p->sensor_id = 1; p->bus = aai16_BusType_BUS_I2C; p->enabled = true; p->period_ms = 100;
    p->which_params = aai16_SensorProfile_i2c_tag;
    p->params.i2c.sda = 8; p->params.i2c.scl = 9; p->params.i2c.addr = 0x19; p->params.i2c.clock_hz = 400000;
    p->ops_count = 1; p->ops[0].type = aai16_OpType_OP_READ; p->ops[0].read_len = 2;
    cm_handle_apply(&cm, &req);
    CHECK(cm_pin_in_use(&cm, 8) && cm_pin_in_use(&cm, 9) && !cm_pin_in_use(&cm, 10));
}

static void test_heartbeat(void)
{
    puts("heartbeat: fields carried, bounded, encodes");
    const uint32_t ids[4] = {1, 2, 3, 4};
    diag_inputs_t in = {.node_id = "nodeB", .uptime_ms = 123456, .config_generation = 4, .rule_generation = 1, .bridge_generation = 2,
                        .active_sensor_ids = ids, .n_active = 4, .model_state = aai16_ModelState_MODEL_STATE_ACTIVE,
                        .model_kind = aai16_ModelKind_MODEL_MOTION, .model_version = 1, .model_reason = aai16_ModelReason_MODEL_REASON_NONE,
                        .readings_ok = 1000, .read_errors = 7, .overruns = 2, .rules_fired = 3, .rule_errors = 0, .free_heap = 123456};
    aai16_Heartbeat hb;
    diag_fill_heartbeat(&in, &hb);
    uint8_t buf[256]; size_t n = 0;
    CHECK(diag_encode_heartbeat(&hb, buf, sizeof(buf), &n) && n > 0 && n <= aai16_Heartbeat_size);
    CHECK(!diag_encode_heartbeat(&hb, buf, 4, &n));
    char p[600]; snprintf(p, sizeof(p), "%s/heartbeat.bin", g_dir);
    FILE *f = fopen(p, "wb"); CHECK(f != NULL);
    if (f) { fwrite(buf, 1, n, f); fclose(f); }
    in.node_id = "a-very-long-node-name-over-16"; in.n_active = 12;           /* over-long inputs are truncated, not overrun */
    diag_fill_heartbeat(&in, &hb);
    CHECK(strlen(hb.node_id) == 15 && hb.active_sensor_ids_count == 8);
}

/* ------------------------------------------------------------------ REPL */
static struct { int run, stop, clear, cfg, rules, bridge; uint8_t last[64]; size_t last_len; bool fail_next; } RP;
static bool r_run(void *c) { (void)c; RP.run++; return !RP.fail_next; }
static bool r_stop(void *c) { (void)c; RP.stop++; return true; }
static bool r_clear(void *c) { (void)c; RP.clear++; return true; }
static void keep(const uint8_t *b, size_t n) { RP.last_len = n; memcpy(RP.last, b, n < 64 ? n : 64); }
static void r_cfg(void *c, const uint8_t *b, size_t n) { (void)c; RP.cfg++; keep(b, n); }
static void r_rules(void *c, const uint8_t *b, size_t n) { (void)c; RP.rules++; keep(b, n); }
static void r_bridge(void *c, const uint8_t *b, size_t n) { (void)c; RP.bridge++; keep(b, n); }
static int g_reboots, g_watch = -1;
static void r_watch(void *c, int which) { (void)c; g_watch = which; }
static void r_reboot(void *c) { (void)c; g_reboots++; }
static size_t r_show(void *c, char *o, size_t cap) { (void)c; return (size_t)snprintf(o, cap, "gen=4 sensors=1,2,3,4"); }

static void test_repl(void)
{
    puts("repl: commands share the MQTT handlers; malformed input is refused");
    const repl_ops_t ops = {r_run, r_stop, r_clear, r_cfg, r_rules, r_bridge, r_show, NULL};
    char out[128];
    memset(&RP, 0, sizeof(RP));
    CHECK(repl_exec(&ops, "run", out, sizeof(out)) == 0 && RP.run == 1 && strcmp(out, "OK") == 0);
    CHECK(repl_exec(&ops, "  STOP  ", out, sizeof(out)) == 0 && RP.stop == 1);
    CHECK(repl_exec(&ops, "Clear", out, sizeof(out)) == 0 && RP.clear == 1);
    CHECK(repl_exec(&ops, "SHOW", out, sizeof(out)) == 0 && strstr(out, "gen=4") != NULL);
    CHECK(repl_exec(&ops, "HELP", out, sizeof(out)) == 0 && strstr(out, "CFG") != NULL);
    CHECK(repl_exec(&ops, "", out, sizeof(out)) == 0 && out[0] == '\0');
    CHECK(repl_exec(&ops, "RUN now", out, sizeof(out)) < 0 && RP.run == 1);
    RP.fail_next = true;
    CHECK(repl_exec(&ops, "RUN", out, sizeof(out)) < 0 && strncmp(out, "ERR", 3) == 0);
    CHECK(repl_exec(&ops, "FLY", out, sizeof(out)) < 0 && strstr(out, "unknown") != NULL);
    CHECK(repl_exec(&ops, "REBOOT", out, sizeof(out)) < 0 && strstr(out, "not supported") != NULL && g_reboots == 0);   /* optional */
    const repl_ops_t rb = {r_run, r_stop, r_clear, r_cfg, r_rules, r_bridge, r_show, NULL, r_reboot, r_watch};
    CHECK(repl_exec(&rb, "reboot", out, sizeof(out)) == 0 && g_reboots == 1 && strstr(out, "rebooting") != NULL);
    CHECK(repl_exec(&ops, "WATCH ALL", out, sizeof(out)) < 0 && strstr(out, "not supported") != NULL);          /* optional */
    CHECK(repl_exec(&rb, "WATCH ALL", out, sizeof(out)) == 0 && g_watch == 256);
    CHECK(repl_exec(&rb, "watch 7", out, sizeof(out)) == 0 && g_watch == 7);
    CHECK(repl_exec(&rb, "WATCH off", out, sizeof(out)) == 0 && g_watch == 0 && strstr(out, "off") != NULL);
    g_watch = -1;
    CHECK(repl_exec(&rb, "WATCH", out, sizeof(out)) < 0 && repl_exec(&rb, "WATCH 0", out, sizeof(out)) < 0 &&
          repl_exec(&rb, "WATCH 256", out, sizeof(out)) < 0 && repl_exec(&rb, "WATCH x1", out, sizeof(out)) < 0 && g_watch == -1);

    CHECK(repl_exec(&ops, "CFG 0a0B ff", out, sizeof(out)) < 0 && RP.cfg == 0);                   /* spaces inside payload */
    CHECK(repl_exec(&ops, "CFG 0a0Bff", out, sizeof(out)) == 0 && RP.cfg == 1 && RP.last_len == 3 &&
          RP.last[0] == 0x0A && RP.last[1] == 0x0B && RP.last[2] == 0xFF);
    CHECK(repl_exec(&ops, "rules 0102", out, sizeof(out)) == 0 && RP.rules == 1 && RP.last_len == 2);
    CHECK(repl_exec(&ops, "BRIDGE 01", out, sizeof(out)) == 0 && RP.bridge == 1);
    CHECK(repl_exec(&ops, "CFG", out, sizeof(out)) < 0 && repl_exec(&ops, "CFG xyz", out, sizeof(out)) < 0 &&
          repl_exec(&ops, "CFG abc", out, sizeof(out)) < 0 && RP.cfg == 1);                       /* odd length / non-hex */
    static char big[REPL_MAX_PAYLOAD * 2 + 64];
    strcpy(big, "CFG ");
    memset(big + 4, 'a', REPL_MAX_PAYLOAD * 2 + 2);
    big[4 + REPL_MAX_PAYLOAD * 2 + 2] = '\0';
    CHECK(repl_exec(&ops, big, out, sizeof(out)) < 0 && RP.cfg == 1);                              /* oversized */
    big[4 + REPL_MAX_PAYLOAD * 2] = '\0';
    CHECK(repl_exec(&ops, big, out, sizeof(out)) == 0 && RP.cfg == 2 && RP.last_len == REPL_MAX_PAYLOAD);   /* exactly at the limit */
}

int main(int argc, char **argv)
{
    if (argc < 2) return 2;
    g_dir = argv[1];
    test_rule_from_python();
    test_stats_edge_actions();
    test_rule_validation();
    test_bridge();
    test_pin_in_use();
    test_heartbeat();
    test_repl();
    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
