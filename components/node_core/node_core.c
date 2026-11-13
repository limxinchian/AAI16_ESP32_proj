/* Node core: wires the config manager, sensor runtime, rule engine, bridge, ML selector, model store, REPL and heartbeat
 * into one node and handles every incoming message kind. Also starts/stops dormant sensors for rules and persists the
 * rules, bridge settings and ML input mapping. */
#include "node_core.h"

#include <stdio.h>
#include <string.h>

#include "diag.h"
#include "pb_decode.h"
#include "pb_encode.h"

#define R(x) aai16_ModelReason_MODEL_REASON_##x

/* ------------------------------------------------------------------ output */
static void logf_(node_t *n, const char *s)
{
    if (n->pf.log) n->pf.log(n->pf.ctx, s);
}

static void emit(node_t *n, node_out_t kind, const char *target, const pb_msgdesc_t *fields, const void *msg)
{
    pb_ostream_t os = pb_ostream_from_buffer(n->out, sizeof(n->out));
    if (!pb_encode(&os, fields, msg)) {
        logf_(n, "encode failed (message too large for NODE_OUT_BUF)");
        return;
    }
    n->pf.publish(n->pf.ctx, kind, target, n->out, os.bytes_written);
}

static void ack_out(node_t *n, node_out_t kind, uint32_t gen, aai16_ApplyPhase phase, aai16_Reason why,
                    uint32_t id, uint32_t detail)
{
    aai16_ApplyAck a = aai16_ApplyAck_init_zero;
    a.generation = gen; a.phase = phase; a.reason = why; a.sensor_id = id; a.detail = detail;
    emit(n, kind, NULL, aai16_ApplyAck_fields, &a);
}

/* ------------------------------------------------------- sensor lookup */
static bool lookup(uint32_t id, ml_source_t *o, void *c)
{
    node_t *n = c;
    if (id >= BR_VIRTUAL_MIN) {
        return bridge_lookup(&n->br, id, o);
    }
    for (int b = aai16_BusType_BUS_GPIO; b <= aai16_BusType_BUS_SPI; b++) {
        const aai16_SensorProfile *p = cm_committed_profile(&n->cm, (aai16_BusType)b);
        if (!p) {
            const aai16_SensorProfile *d = cm_dormant_by_id(&n->cm, id);
            if (d && (int)d->bus == b) p = d; /* asleep: mappings stay valid, readings simply do not arrive */
        }
        if (p && p->sensor_id == id) {
            memset(o, 0, sizeof(*o));
            o->active = true;
            o->period_ms = p->period_ms;
            o->output_count = p->output_count;
            for (uint32_t i = 0; i < p->output_kinds_count && i < 8; i++) o->kinds[i] = p->output_kinds[i];
            return true;
        }
    }
    return false;
}

/* --------------------------------------------------------- model status */
static bool model_running(const node_t *n) { return n->model_active && n->sel.mapped && !n->sel.suspended; }

static void publish_status(node_t *n, aai16_ModelState st, aai16_ModelReason why)
{
    aai16_ModelStatus m = aai16_ModelStatus_init_zero;
    m.generation = n->model_gen; m.kind = n->model_kind; m.model_version = n->model_version;
    m.state = st; m.reason = why;
    n->rep_state = st; n->rep_reason = why;
    emit(n, OUT_MODEL_STATUS, NULL, aai16_ModelStatus_fields, &m);
}

static void current_status(const node_t *n, aai16_ModelState *st, aai16_ModelReason *why)
{
    if (!n->model_active) { *st = aai16_ModelState_MODEL_STATE_NONE; *why = R(NO_ACTIVE_MODEL); }
    else if (model_running(n)) { *st = aai16_ModelState_MODEL_STATE_ACTIVE; *why = R(NONE); }
    else { *st = aai16_ModelState_MODEL_STATE_SUSPENDED; *why = n->sel.mapped ? n->sel.state : R(NO_INPUTS); }
}

static void sync_status(node_t *n, bool force)
{
    aai16_ModelState st; aai16_ModelReason why;
    current_status(n, &st, &why);
    if (force || st != n->rep_state || why != n->rep_reason) publish_status(n, st, why);
}

/* ------------------------------------------------------------ inference */
static void infer(node_t *n, const float *feat)
{
    const aai16_ModelContract *c = &n->sel.contract;
    const bool anomaly = c->kind == aai16_ModelKind_MODEL_ANOMALY;
    int8_t in[ML_MAX_FEATURES], out[ML_MAX_FEATURES] = {0};
    const uint32_t nc = anomaly ? (c->output_count < ML_MAX_FEATURES ? c->output_count : ML_MAX_FEATURES)
                                : (c->labels_count < 4 ? c->labels_count : 4);
    for (uint32_t i = 0; i < c->feature_count && i < ML_MAX_FEATURES; i++) {
        in[i] = ml_quantize(feat[i], c->input_scale, c->input_zero_point);
    }
    uint32_t us = 0;
    if (!n->pf.ml_invoke(n->pf.ctx, in, c->feature_count, out, nc, &us)) {
        logf_(n, "inference failed");
        return;
    }
    aai16_InferenceResult r = aai16_InferenceResult_init_zero;
    if (anomaly) {
        float score = 0.0f, err[ML_SLOT_COUNT];
        if (!ml_anomaly_score(c, in, out, n->sel.active_mask, &score, err)) {
            logf_(n, "anomaly score failed");
            return;
        }
        r.anomaly_score = score;
        r.anomaly_threshold = c->anomaly_threshold;
        r.anomaly = score > c->anomaly_threshold;
        r.slot_mask = n->sel.active_mask;
        r.slot_errors_count = ML_SLOT_COUNT;
        for (uint32_t i = 0; i < ML_SLOT_COUNT; i++) r.slot_errors[i] = err[i];
        r.predicted_class = r.anomaly ? 1u : 0u;
    } else {
        r.scores_count = nc;
        r.predicted_class = ml_decode_output(out, nc, c->output_scale, c->output_zero_point, r.scores);
    }
    r.kind = n->model_kind; r.model_version = n->model_version; r.generation = n->model_gen;
    r.timestamp_us = n->pf.now_us(n->pf.ctx); r.inference_us = us;
    emit(n, OUT_INFERENCE, NULL, aai16_InferenceResult_fields, &r);
    n->inferences++;
}

static void watch_print(node_t *n, const aai16_Reading *r)
{
    if (!n->watch || (n->watch != 256 && (uint32_t)n->watch != r->sensor_id)) return;
    const uint32_t ms = (uint32_t)(r->timestamp_us / 1000u);
    uint32_t *last = &n->watch_last_ms[r->sensor_id & 255u];
    if (*last && (uint32_t)(ms - *last) < 1000u) return; /* at most one line per second per sensor */
    *last = ms ? ms : 1u;
    static const char *const err[] = {"ok", "timeout", "bus_error", "bad_frame", "not_active", "calc_error"};
    char line[200];
    int k = snprintf(line, sizeof(line), "READ id=%u %s", (unsigned)r->sensor_id, (unsigned)r->error < 6 ? err[r->error] : "error");
    if (r->valid) {
        k += snprintf(line + k, sizeof(line) - (size_t)k, " [");
        for (uint32_t i = 0; i < r->values_count && k < (int)sizeof(line) - 16; i++) {
            k += snprintf(line + k, sizeof(line) - (size_t)k, "%s%.3f", i ? ", " : "", (double)r->values[i]);
        }
        snprintf(line + k, sizeof(line) - (size_t)k, "]");
    }
    logf_(n, line);
}

void node_feed_reading(node_t *n, const aai16_Reading *r)
{
    watch_print(n, r);
    re_on_reading(&n->re, r->sensor_id, r->valid, r->values, r->values_count);
    if (!n->model_active) return;
    bool ready = false;
    float feat[ML_MAX_FEATURES];
    mlsel_push(&n->sel, r->sensor_id, r->timestamp_us, r->valid, r->values, r->values_count, lookup, n, &ready, feat);
    if (ready) infer(n, feat);
    sync_status(n, false);
}

static void on_reading(const aai16_Reading *r, void *c)
{
    node_t *n = c;
    if (r->sensor_id < BR_VIRTUAL_MIN) { /* only local physical sensors leave the node */
        emit(n, OUT_READING, NULL, aai16_Reading_fields, r);
    }
    node_feed_reading(n, r);
}

/* --------------------------------------------------------- rule hooks */
static void rh_set(void *c, uint32_t p, bool l) { node_t *n = c; if (n->pf.gpio_set) n->pf.gpio_set(n->pf.ctx, p, l); }
static void rh_toggle(void *c, uint32_t p) { node_t *n = c; if (n->pf.gpio_toggle) n->pf.gpio_toggle(n->pf.ctx, p); }
static void rh_pulse(void *c, uint32_t p, bool l, uint32_t ms) { node_t *n = c; if (n->pf.gpio_pulse) n->pf.gpio_pulse(n->pf.ctx, p, l, ms); }
static void rh_pwm(void *c, uint32_t p, uint32_t pm) { node_t *n = c; if (n->pf.pwm) n->pf.pwm(n->pf.ctx, p, pm); }
static uint64_t rh_now(void *c) { node_t *n = c; return n->pf.now_us(n->pf.ctx); }
static bool rh_pin_ok(void *c, uint32_t p) { node_t *n = c; return cm_pin_allowed(p) && !cm_pin_in_use(&n->cm, p); }

static void rh_notify(void *c, uint32_t id, const char *text, double v)
{
    node_t *n = c;
    aai16_Notification m = aai16_Notification_init_zero;
    m.rule_id = id; m.value = (float)v; m.timestamp_us = n->pf.now_us(n->pf.ctx);
    snprintf(m.text, sizeof(m.text), "%s", text);
    emit(n, OUT_NOTIFICATION, NULL, aai16_Notification_fields, &m);
}

static void rh_ml(void *c)
{
    node_t *n = c;
    float feat[ML_MAX_FEATURES];
    if (n->model_active && mlsel_current_features(&n->sel, feat)) infer(n, feat);
}

static void rh_remote(void *c, const char *node, aai16_RemoteCmd cmd)
{
    node_t *n = c;
    aai16_NodeCommand m = aai16_NodeCommand_init_zero;
    m.cmd = cmd;
    emit(n, OUT_NODE_COMMAND, node, aai16_NodeCommand_fields, &m);
}

/* Dormant sensors: a rule can start one for a while, or until a rule / config stops it. */
static void wake_note(node_t *n, uint32_t rule_id, uint32_t sensor_id, const char *what)
{
    char t[40];
    snprintf(t, sizeof(t), "sensor %u %s", (unsigned)sensor_id, what);
    rh_notify(n, rule_id, t, (double)sensor_id);
}

static bool rh_sensor(void *c, uint32_t rule_id, uint32_t sensor_id, bool start, uint32_t duration_ms)
{
    node_t *n = c;
    const uint64_t now = n->pf.now_us(n->pf.ctx);
    if (start) {
        const aai16_SensorProfile *d = cm_dormant_by_id(&n->cm, sensor_id);
        if (!d) return false; /* not stored as dormant (or already a configured, running sensor) */
        const int s = (int)d->bus - 1;
        if (!n->wake[s].on) {
            n->wake_scratch = *d;
            n->wake_scratch.enabled = true;
            n->wake_scratch.dormant = false;
            if (!rt_apply(&n->wake_scratch, &n->rt)) return false;
            n->wake[s].on = true;
            wake_note(n, rule_id, sensor_id, duration_ms ? "started (timed)" : "started");
        }
        n->wake[s].stop_us = duration_ms ? now + (uint64_t)duration_ms * 1000u : 0; /* a new tap re-arms the timer */
        return true;
    }
    for (int s = 0; s < CM_BUS_COUNT; s++) {
        const rt_slot_t *sl = &n->rt.slot[s];
        if (n->wake[s].on && sl->active && sl->profile.sensor_id == sensor_id) {
            rt_stop((aai16_BusType)(s + 1), &n->rt);
            n->wake[s].on = false;
            wake_note(n, rule_id, sensor_id, "stopped");
            return true;
        }
    }
    return false; /* only sensors a rule woke can be stopped by a rule */
}

/* ----------------------------------------------------- config manager hooks */
static bool cmh_apply(const aai16_SensorProfile *p, void *c)
{
    node_t *n = c;
    if (p->bus >= aai16_BusType_BUS_GPIO && p->bus <= aai16_BusType_BUS_SPI) n->wake[(int)p->bus - 1].on = false;
    return rt_apply(p, &n->rt);
}

static void cmh_stop(aai16_BusType b, void *c)
{
    node_t *n = c;
    if (b >= aai16_BusType_BUS_GPIO && b <= aai16_BusType_BUS_SPI) n->wake[(int)b - 1].on = false;
    rt_stop(b, &n->rt);
}

static bool cmh_persist(const aai16_CommittedConfig *cfg, void *c)
{
    node_t *n = c;
    pb_ostream_t os = pb_ostream_from_buffer(n->blob, sizeof(n->blob));
    if (!pb_encode(&os, aai16_CommittedConfig_fields, cfg)) return false;
    return n->pf.persist_config ? n->pf.persist_config(n->pf.ctx, n->blob, os.bytes_written) : true;
}

static void cmh_ack(const aai16_ApplyAck *a, void *c)
{
    node_t *n = c;
    emit(n, OUT_APPLY_ACK, NULL, aai16_ApplyAck_fields, a);
    if (a->phase == aai16_ApplyPhase_PHASE_APPLIED) {
        rt_set_generation(&n->rt, a->generation);
        if (n->model_active) {
            mlsel_revalidate(&n->sel, lookup, n); /* a sensor swap may invalidate the mapping */
            sync_status(n, false);
        }
    }
}

/* ------------------------------------------------------ persisted node state */
static void save_state(node_t *n)
{
    if (!n->pf.persist_state) return;
    aai16_NodeState *s = &n->state;
    memset(s, 0, sizeof(*s));
    s->schema_version = 1;
    s->has_rules = true;
    s->rules.generation = n->re.generation;
    s->rules.rules_count = n->re.count;
    for (uint32_t i = 0; i < n->re.count; i++) s->rules.rules[i] = n->re.rules[i].rule;
    s->has_bridge = true;
    s->bridge.generation = n->br.generation;
    s->bridge.sources_count = n->br.count;
    for (uint32_t i = 0; i < n->br.count; i++) s->bridge.sources[i] = n->br.slot[i].src;
    s->model_generation = n->model_gen;
    if (n->model_active && n->sel.mapped) {
        s->mapped_kind = n->model_kind;
        s->inputs_count = 0;
        for (uint32_t c = 0; c < n->sel.contract.channels_count && s->inputs_count < 10; c++) {
            if (!mlsel_channel_mapped(&n->sel, c)) continue;
            const uint32_t k = s->inputs_count++;
            s->inputs[k].sensor_id = n->sel.map[c].sensor_id;
            s->inputs[k].value_index = n->sel.map[c].value_index;
            s->inputs[k].feature_slot = c;
        }
    }
    pb_ostream_t os = pb_ostream_from_buffer(n->blob, sizeof(n->blob));
    if (!pb_encode(&os, aai16_NodeState_fields, s) || !n->pf.persist_state(n->pf.ctx, n->blob, os.bytes_written)) {
        logf_(n, "could not persist node state (rules / bridge / ML mapping will be lost on reboot)");
    }
}

/* Restores through the live validators; whatever no longer validates is dropped, never trusted. */
static void restore_state(node_t *n)
{
    size_t len = 0;
    if (!n->pf.load_state || !n->pf.load_state(n->pf.ctx, n->blob, sizeof(n->blob), &len) || len == 0) return;
    aai16_NodeState *s = &n->state;
    memset(s, 0, sizeof(*s));
    pb_istream_t is = pb_istream_from_buffer(n->blob, len);
    if (!pb_decode(&is, aai16_NodeState_fields, s) || s->schema_version != 1) {
        logf_(n, "stored node state unreadable or from another schema; ignored");
        return;
    }
    if (s->has_rules && s->rules.generation > 0) {
        re_error_t e;
        if (!re_apply(&n->re, &s->rules, &e)) logf_(n, "stored rules no longer validate; dropped");
    }
    if (s->has_bridge && s->bridge.generation > 0) {
        aai16_Reason why;
        uint32_t idx;
        if (!bridge_apply(&n->br, &s->bridge, &why, &idx)) logf_(n, "stored bridge settings no longer validate; dropped");
    }
    n->model_gen = s->model_generation;
    if (n->model_active && s->inputs_count > 0 && s->mapped_kind == n->model_kind) {
        if (mlsel_set_inputs(&n->sel, s->inputs, s->inputs_count, lookup, n) != R(NONE)) {
            logf_(n, "stored ML input mapping no longer valid; waiting for SET_INPUTS");
        }
    }
}

/* ------------------------------------------------------------- lifecycle */
void node_init(node_t *n, const node_platform_t *pf)
{
    memset(n, 0, sizeof(*n));
    n->pf = *pf;
    const cm_hooks_t ch = {cmh_apply, cmh_stop, cmh_persist, cmh_ack, n};
    cm_init(&n->cm, &ch);
    rt_init(&n->rt, &n->pf.bus, on_reading, n);
    const re_hooks_t rh = {rh_set, rh_toggle, rh_pulse, rh_pwm, rh_notify, rh_ml, rh_remote, rh_pin_ok, rh_now, n, rh_sensor};
    re_init(&n->re, &rh);
    bridge_init(&n->br, pf->node_id);
    mlsel_init(&n->sel);
    model_store_init(&n->ms, &n->pf.store);
    n->rep_state = (aai16_ModelState)-1; /* force the first status publication */
}

void node_boot(node_t *n)
{
    static ml_package_t pkg;
    n->boot_us = n->pf.now_us(n->pf.ctx);
    n->next_heartbeat_us = n->boot_us + NODE_HEARTBEAT_US;
    size_t len = 0;
    if (n->pf.load_config && n->pf.load_config(n->pf.ctx, n->blob, sizeof(n->blob), &len) && len > 0) {
        if (cm_restore(&n->cm, n->blob, len)) {
            rt_set_generation(&n->rt, n->cm.committed.generation);
        } else {
            logf_(n, "stored configuration failed validation; starting empty");
        }
    }
    if (n->ms.has_active && model_store_active(&n->ms, &pkg) == R(NONE)) {
        if (n->pf.ml_load(n->pf.ctx, &pkg) == R(NONE)) {
            n->model_active = true;
            n->model_kind = pkg.contract.kind;
            n->model_version = pkg.contract.model_version;
            mlsel_set_contract(&n->sel, &pkg.contract);
        } else {
            logf_(n, "active model rejected by runtime at boot");
        }
    }
    restore_state(n);
    n->booted = true;
    sync_status(n, true);
}

/* ---------------------------------------------------------------- control */
static void do_clear(node_t *n)
{
    aai16_ConfigApply *req = &n->cm.rx; /* scratch owned by the config manager, same task */
    memset(req, 0, sizeof(*req));
    req->generation = n->cm.committed.generation + 1;
    req->profiles_count = CM_BUS_COUNT;
    for (uint32_t b = 0; b < CM_BUS_COUNT; b++) {
        req->profiles[b].sensor_id = b + 1;
        req->profiles[b].bus = (aai16_BusType)(b + 1);
        req->profiles[b].enabled = false;
    }
    cm_handle_apply(&n->cm, req);
    re_clear(&n->re);
    n->br.count = 0;
    memset(n->br.slot, 0, sizeof(n->br.slot));
    n->paused = false;
    if (n->model_active) {
        mlsel_revalidate(&n->sel, lookup, n);
        sync_status(n, false);
    }
    save_state(n);
}

static void apply_cmd(node_t *n, aai16_RemoteCmd cmd)
{
    if (cmd == aai16_RemoteCmd_RCMD_RUN) n->paused = false;
    else if (cmd == aai16_RemoteCmd_RCMD_STOP) n->paused = true;
    else if (cmd == aai16_RemoteCmd_RCMD_CLEAR) do_clear(n);
}

typedef struct { node_t *n; bool loaded; } act_ctx_t;

static aai16_ModelReason act_check(const ml_package_t *pkg, void *c)
{
    act_ctx_t *a = c;
    if (pkg->contract.kind != a->n->ctrl_rx.kind) return R(KIND_MISMATCH);
    const aai16_ModelReason r = a->n->pf.ml_load(a->n->pf.ctx, pkg);
    a->loaded = (r == R(NONE));
    return r;
}

static void on_control(node_t *n, const uint8_t *buf, size_t len)
{
    static ml_package_t pkg;
    memset(&n->ctrl_rx, 0, sizeof(n->ctrl_rx));
    pb_istream_t is = pb_istream_from_buffer(buf, len);
    if (!pb_decode(&is, aai16_ModelControl_fields, &n->ctrl_rx)) {
        publish_status(n, aai16_ModelState_MODEL_STATE_REJECTED, R(BAD_PACKAGE));
        return;
    }
    const aai16_ModelControl *c = &n->ctrl_rx;
    if (c->action == aai16_ModelAction_ACTION_QUERY) { sync_status(n, true); return; }
    if (c->action == aai16_ModelAction_ACTION_ABORT_TRANSFER) { model_store_abort(&n->ms, c->transfer_id); return; }
    if (c->action != aai16_ModelAction_ACTION_ACTIVATE && c->action != aai16_ModelAction_ACTION_SET_INPUTS) {
        publish_status(n, aai16_ModelState_MODEL_STATE_REJECTED, R(BAD_PACKAGE));
        return;
    }
    if (c->generation == 0 || c->generation <= n->model_gen) {
        publish_status(n, aai16_ModelState_MODEL_STATE_REJECTED, R(STALE_GENERATION));
        sync_status(n, true);
        return;
    }

    if (c->action == aai16_ModelAction_ACTION_ACTIVATE) {
        act_ctx_t a = {n, false};
        const aai16_ModelReason r = model_store_activate(&n->ms, c->transfer_id, act_check, &a);
        if (r != R(NONE)) {
            if (a.loaded && n->model_active && model_store_active(&n->ms, &pkg) == R(NONE)) {
                (void)n->pf.ml_load(n->pf.ctx, &pkg); /* storage failed after the runtime took the new model: restore the old */
            }
            publish_status(n, aai16_ModelState_MODEL_STATE_REJECTED, r);
            sync_status(n, true);
            return;
        }
        if (model_store_active(&n->ms, &pkg) == R(NONE)) {
            n->model_active = true;
            n->model_kind = pkg.contract.kind;
            n->model_version = pkg.contract.model_version;
            n->model_gen = c->generation;
            mlsel_set_contract(&n->sel, &pkg.contract);
            save_state(n);
        }
        sync_status(n, true);
        return;
    }

    const aai16_ModelReason r = mlsel_set_inputs(&n->sel, c->inputs, c->inputs_count, lookup, n);
    if (r != R(NONE)) {
        publish_status(n, aai16_ModelState_MODEL_STATE_REJECTED, r); /* previous mapping stays in force */
    } else {
        n->model_gen = c->generation;
        save_state(n);
    }
    sync_status(n, true);
}

void node_on_message(node_t *n, node_in_t kind, const char *from_node, const uint8_t *buf, size_t len)
{
    switch (kind) {
    case IN_CONFIG:
        cm_handle_bytes(&n->cm, buf, len);
        break;
    case IN_RULES: {
        memset(&n->rules_rx, 0, sizeof(n->rules_rx));
        pb_istream_t is = pb_istream_from_buffer(buf, len);
        if (!pb_decode(&is, aai16_RuleSetApply_fields, &n->rules_rx)) {
            ack_out(n, OUT_RULE_ACK, 0, aai16_ApplyPhase_PHASE_REJECTED, aai16_Reason_REASON_MALFORMED, 0, 0);
            break;
        }
        re_error_t e = {0};
        if (re_apply(&n->re, &n->rules_rx, &e)) {
            ack_out(n, OUT_RULE_ACK, n->rules_rx.generation, aai16_ApplyPhase_PHASE_APPLIED, aai16_Reason_REASON_NONE, 0, 0);
            save_state(n);
        } else {
            ack_out(n, OUT_RULE_ACK, n->rules_rx.generation, aai16_ApplyPhase_PHASE_REJECTED, e.reason, e.rule_id, e.detail);
        }
        break;
    }
    case IN_BRIDGE: {
        memset(&n->bridge_rx, 0, sizeof(n->bridge_rx));
        pb_istream_t is = pb_istream_from_buffer(buf, len);
        if (!pb_decode(&is, aai16_BridgeApply_fields, &n->bridge_rx)) {
            ack_out(n, OUT_BRIDGE_ACK, 0, aai16_ApplyPhase_PHASE_REJECTED, aai16_Reason_REASON_MALFORMED, 0, 0);
            break;
        }
        aai16_Reason why; uint32_t idx;
        if (bridge_apply(&n->br, &n->bridge_rx, &why, &idx)) {
            ack_out(n, OUT_BRIDGE_ACK, n->bridge_rx.generation, aai16_ApplyPhase_PHASE_APPLIED, aai16_Reason_REASON_NONE, 0, 0);
            if (n->model_active) { mlsel_revalidate(&n->sel, lookup, n); sync_status(n, false); }
            save_state(n);
        } else {
            ack_out(n, OUT_BRIDGE_ACK, n->bridge_rx.generation, aai16_ApplyPhase_PHASE_REJECTED, why, 0, idx);
        }
        break;
    }
    case IN_MODEL_CHUNK: {
        memset(&n->chunk_rx, 0, sizeof(n->chunk_rx));
        aai16_ChunkAck a = aai16_ChunkAck_init_zero;
        pb_istream_t is = pb_istream_from_buffer(buf, len);
        if (!pb_decode(&is, aai16_ModelChunk_fields, &n->chunk_rx)) {
            a.status = aai16_ChunkStatus_CHUNK_BAD_LENGTH;
        } else {
            model_store_on_chunk(&n->ms, &n->chunk_rx, &a);
        }
        emit(n, OUT_CHUNK_ACK, NULL, aai16_ChunkAck_fields, &a);
        break;
    }
    case IN_MODEL_CONTROL:
        on_control(n, buf, len);
        break;
    case IN_COMMAND: {
        aai16_NodeCommand cmd = aai16_NodeCommand_init_zero;
        pb_istream_t is = pb_istream_from_buffer(buf, len);
        if (pb_decode(&is, aai16_NodeCommand_fields, &cmd)) apply_cmd(n, cmd.cmd); /* never forwarded: no command loops */
        break;
    }
    case IN_REMOTE_READING: {
        static aai16_Reading r;
        const br_result_t br = bridge_on_remote(&n->br, from_node ? from_node : "", buf, len, n->pf.now_us(n->pf.ctx), &r);
        if (br == BR_OK || br == BR_REMOTE_INVALID) node_feed_reading(n, &r);
        break;
    }
    }
}

/* ------------------------------------------------------------------- tick */
static void send_heartbeat(node_t *n)
{
    uint32_t ids[4], k = 0, ok = 0, err = 0, over = 0;
    for (int b = aai16_BusType_BUS_GPIO; b <= aai16_BusType_BUS_SPI; b++) {
        const rt_slot_t *sl = &n->rt.slot[b - 1];
        if (sl->active) ids[k++] = sl->profile.sensor_id;
        const rt_stats_t *s = rt_stats(&n->rt, (aai16_BusType)b);
        if (s) { ok += s->ok; err += s->errors; over += s->overruns; }
    }
    aai16_ModelState st; aai16_ModelReason why;
    current_status(n, &st, &why);
    diag_inputs_t in = {.node_id = n->pf.node_id, .uptime_ms = (n->pf.now_us(n->pf.ctx) - n->boot_us) / 1000u,
                        .config_generation = n->cm.committed.generation, .rule_generation = n->re.generation,
                        .bridge_generation = n->br.generation, .active_sensor_ids = ids, .n_active = k,
                        .model_state = st, .model_kind = n->model_kind, .model_version = n->model_version,
                        .model_reason = why, .readings_ok = ok, .read_errors = err, .overruns = over,
                        .rules_fired = n->re.fired, .rule_errors = n->re.errors,
                        .free_heap = n->pf.free_heap ? n->pf.free_heap(n->pf.ctx) : 0};
    aai16_Heartbeat hb;
    diag_fill_heartbeat(&in, &hb);
    emit(n, OUT_HEARTBEAT, NULL, aai16_Heartbeat_fields, &hb);
}

void node_tick(node_t *n)
{
    if (!n->paused) rt_poll(&n->rt);
    const uint64_t now = n->pf.now_us(n->pf.ctx);
    for (int s = 0; s < CM_BUS_COUNT; s++) {
        if (n->wake[s].on && n->wake[s].stop_us && now >= n->wake[s].stop_us) {
            const uint32_t id = n->rt.slot[s].profile.sensor_id;
            rt_stop((aai16_BusType)(s + 1), &n->rt);
            n->wake[s].on = false;
            wake_note(n, 0, id, "stopped (timer)");
        }
    }
    static aai16_Reading stale[BR_MAX_SOURCES];
    const uint32_t k = bridge_poll(&n->br, now, stale, BR_MAX_SOURCES);
    for (uint32_t i = 0; i < k; i++) node_feed_reading(n, &stale[i]);
    if (now >= n->next_heartbeat_us) {
        send_heartbeat(n);
        n->next_heartbeat_us = now + NODE_HEARTBEAT_US;
    }
}

/* ------------------------------------------------------------------- REPL */
static bool rp_run(void *c) { ((node_t *)c)->paused = false; return true; }
static bool rp_stop(void *c) { ((node_t *)c)->paused = true; return true; }
static bool rp_clear(void *c) { do_clear(c); return true; }
static void rp_watch(void *c, int which) { node_t *n = c; n->watch = which; memset(n->watch_last_ms, 0, sizeof(n->watch_last_ms)); }
static void rp_reboot(void *c) { node_t *n = c; if (n->pf.reboot) n->pf.reboot(n->pf.ctx); }
static void rp_cfg(void *c, const uint8_t *b, size_t l) { node_on_message(c, IN_CONFIG, NULL, b, l); }
static void rp_rules(void *c, const uint8_t *b, size_t l) { node_on_message(c, IN_RULES, NULL, b, l); }
static void rp_bridge(void *c, const uint8_t *b, size_t l) { node_on_message(c, IN_BRIDGE, NULL, b, l); }

static size_t rp_show(void *c, char *out, size_t cap)
{
    node_t *n = c;
    aai16_ModelState st; aai16_ModelReason why;
    current_status(n, &st, &why);
    char ids[40] = "";
    for (int b = aai16_BusType_BUS_GPIO; b <= aai16_BusType_BUS_SPI; b++) {
        const rt_slot_t *sl = &n->rt.slot[b - 1];
        if (sl->active) { char t[8]; snprintf(t, sizeof(t), "%s%u", ids[0] ? "," : "", (unsigned)sl->profile.sensor_id); strncat(ids, t, sizeof(ids) - strlen(ids) - 1); }
    }
    return (size_t)snprintf(out, cap, "node=%s %s cfg_gen=%u sensors=[%s] rules=%u(gen %u) bridge=%u model=%s state=%d reason=%d inferences=%u",
                            n->pf.node_id, n->paused ? "STOPPED" : "RUNNING", (unsigned)n->cm.committed.generation, ids,
                            (unsigned)n->re.count, (unsigned)n->re.generation, (unsigned)n->br.count,
                            n->model_active ? "loaded" : "none", (int)st, (int)why, (unsigned)n->inferences);
}

int node_repl_line(node_t *n, const char *line, char *out, size_t cap)
{
    const repl_ops_t ops = {rp_run, rp_stop, rp_clear, rp_cfg, rp_rules, rp_bridge, rp_show, n, n->pf.reboot ? rp_reboot : NULL, rp_watch};
    return repl_exec(&ops, line, out, cap);
}
