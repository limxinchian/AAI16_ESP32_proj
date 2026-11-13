/* Rule engine: validation and evaluation of bounded RPN rules over sensor values, and their actions (log/notify, GPIO,
 * PWM, ML trigger, remote command, start/stop of dormant sensors). See rule_engine.h for the safety properties. */
#include "rule_engine.h"

#include <math.h>
#include <string.h>

static bool fail(re_error_t *e, aai16_Reason r, uint32_t rule_id, uint32_t detail)
{
    if (e) {
        e->reason = r;
        e->rule_id = rule_id;
        e->detail = detail;
    }
    return false;
}

void re_init(rule_engine_t *re, const re_hooks_t *hk)
{
    memset(re, 0, sizeof(*re));
    re->hk = *hk;
}

void re_clear(rule_engine_t *re)
{
    re->count = 0;
    memset(re->rules, 0, sizeof(re->rules));
}

/* ----------------------------------------------------------------- validation */
static bool validate_when(const aai16_Rule *r, re_error_t *e)
{
    if (r->when_count == 0 || r->when_count > 24) {
        return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, r->when_count);
    }
    uint32_t depth = 0;
    for (uint32_t t = 0; t < r->when_count; t++) {
        const aai16_CalcToken *tk = &r->when[t];
        switch (tk->op) {
        case aai16_CalcOp_CALC_PUSH_CONST:
            if (!isfinite(tk->value) || depth >= 8) return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, t);
            depth++;
            break;
        case aai16_CalcOp_CALC_PUSH_REG:
            if (tk->reg >= r->vars_count || depth >= 8) return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, t);
            depth++;
            break;
        case aai16_CalcOp_CALC_ADD: case aai16_CalcOp_CALC_SUB: case aai16_CalcOp_CALC_MUL:
        case aai16_CalcOp_CALC_DIV: case aai16_CalcOp_CALC_MIN: case aai16_CalcOp_CALC_MAX:
        case aai16_CalcOp_CALC_GT:  case aai16_CalcOp_CALC_LT:  case aai16_CalcOp_CALC_GE:
        case aai16_CalcOp_CALC_LE:  case aai16_CalcOp_CALC_EQ:  case aai16_CalcOp_CALC_AND:
        case aai16_CalcOp_CALC_OR:
            if (depth < 2) return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, t);
            depth--;
            break;
        case aai16_CalcOp_CALC_NEG: case aai16_CalcOp_CALC_ABS: case aai16_CalcOp_CALC_SQRT:
        case aai16_CalcOp_CALC_NOT:
            if (depth < 1) return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, t);
            break;
        default:
            return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, t);
        }
    }
    return depth == 1 ? true : fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, r->when_count);
}

static bool validate_action(const aai16_Rule *r, bool (*pin_ok)(void *, uint32_t), void *ctx, re_error_t *e)
{
    const aai16_RuleAction *a = &r->action;
    const bool needs_pin = a->type == aai16_RuleActionType_RACT_GPIO_SET || a->type == aai16_RuleActionType_RACT_GPIO_TOGGLE ||
                           a->type == aai16_RuleActionType_RACT_GPIO_PULSE || a->type == aai16_RuleActionType_RACT_PWM;
    if (a->type < aai16_RuleActionType_RACT_LOG || a->type > aai16_RuleActionType_RACT_SENSOR_STOP) {
        return fail(e, aai16_Reason_REASON_BAD_ACTION, r->rule_id, 0);
    }
    if (needs_pin && (pin_ok == NULL || !pin_ok(ctx, a->pin))) {
        return fail(e, aai16_Reason_REASON_FORBIDDEN_OUTPUT_PIN, r->rule_id, a->pin);
    }
    if (a->type == aai16_RuleActionType_RACT_GPIO_PULSE && (a->duration_ms < 1 || a->duration_ms > 60000)) {
        return fail(e, aai16_Reason_REASON_BAD_ACTION, r->rule_id, a->duration_ms);
    }
    if (a->type == aai16_RuleActionType_RACT_PWM && a->duty_permille > 1000) {
        return fail(e, aai16_Reason_REASON_BAD_ACTION, r->rule_id, a->duty_permille);
    }
    if (a->type == aai16_RuleActionType_RACT_SENSOR_START || a->type == aai16_RuleActionType_RACT_SENSOR_STOP) {
        /* physical sensors only (1..127); virtual bridge sensors have no hardware to start */
        if (a->sensor_id < 1 || a->sensor_id > 127 || a->duration_ms > RE_MAX_WAKE_MS ||
            (a->type == aai16_RuleActionType_RACT_SENSOR_STOP && a->duration_ms != 0)) {
            return fail(e, aai16_Reason_REASON_BAD_ACTION, r->rule_id, a->sensor_id);
        }
    }
    if (a->type == aai16_RuleActionType_RACT_REMOTE_CMD) {
        if (a->target_node[0] == '\0' || a->cmd < aai16_RemoteCmd_RCMD_RUN || a->cmd > aai16_RemoteCmd_RCMD_CLEAR ||
            r->cooldown_ms < RE_MIN_REMOTE_COOLDOWN_MS) {
            return fail(e, aai16_Reason_REASON_BAD_ACTION, r->rule_id, 0);
        }
    }
    return true;
}

bool re_validate(const aai16_RuleSetApply *set, bool (*pin_ok)(void *, uint32_t), void *ctx, re_error_t *e)
{
    if (set->rules_count > RE_MAX_RULES) {
        return fail(e, aai16_Reason_REASON_TOO_MANY_RULES, 0, set->rules_count);
    }
    for (uint32_t i = 0; i < set->rules_count; i++) {
        const aai16_Rule *r = &set->rules[i];
        if (r->rule_id < 1 || r->rule_id > 255) {
            return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, 0);
        }
        for (uint32_t j = 0; j < i; j++) {
            if (set->rules[j].rule_id == r->rule_id) return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, 0);
        }
        if (r->vars_count < 1 || r->vars_count > RE_MAX_VARS) {
            return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, r->vars_count);
        }
        for (uint32_t v = 0; v < r->vars_count; v++) {
            const aai16_RuleVar *var = &r->vars[v];
            if (var->sensor_id < 1 || var->sensor_id > 255 || var->value_index > 7 || var->window < 1 ||
                var->window > RE_HIST || var->stat > aai16_RuleStat_STAT_STDDEV ||
                ((var->stat == aai16_RuleStat_STAT_PREV || var->stat == aai16_RuleStat_STAT_DELTA) && var->window < 2)) {
                return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, v);
            }
        }
        if (r->cooldown_ms < RE_MIN_COOLDOWN_MS || r->cooldown_ms > RE_MAX_COOLDOWN_MS) {
            return fail(e, aai16_Reason_REASON_BAD_RULE, r->rule_id, r->cooldown_ms);
        }
        if (!validate_when(r, e) || !validate_action(r, pin_ok, ctx, e)) {
            return false;
        }
    }
    return true;
}

bool re_apply(rule_engine_t *re, const aai16_RuleSetApply *set, re_error_t *err)
{
    if (set->generation == 0 || set->generation <= re->generation) {
        return fail(err, set->generation == re->generation ? aai16_Reason_REASON_DUPLICATE_GENERATION
                                                           : aai16_Reason_REASON_STALE_GENERATION, 0, 0);
    }
    if (!re_validate(set, re->hk.pin_ok, re->hk.ctx, err)) {
        return false;
    }
    re_clear(re);
    for (uint32_t i = 0; i < set->rules_count; i++) {
        re->rules[i].rule = set->rules[i];
    }
    re->count = set->rules_count;
    re->generation = set->generation;
    return true;
}

/* -------------------------------------------------------------- evaluation */
static void hist_push(re_hist_t *h, float v)
{
    h->v[h->head] = v;
    h->head = (uint8_t)((h->head + 1) % RE_HIST);
    if (h->n < RE_HIST) h->n++;
}

static float hist_at(const re_hist_t *h, uint8_t back) /* 0 = newest */
{
    return h->v[(h->head + RE_HIST - 1 - back) % RE_HIST];
}

static bool var_value(const re_hist_t *h, const aai16_RuleVar *var, double *out)
{
    const uint8_t w = (uint8_t)var->window;
    if (h->n == 0) return false;
    switch (var->stat) {
    case aai16_RuleStat_STAT_LATEST: *out = hist_at(h, 0); return true;
    case aai16_RuleStat_STAT_PREV:   if (h->n < 2) return false; *out = hist_at(h, 1); return true;
    case aai16_RuleStat_STAT_DELTA:  if (h->n < 2) return false; *out = (double)hist_at(h, 0) - hist_at(h, 1); return true;
    default: break;
    }
    if (h->n < w) return false; /* window not yet full: no statistic */
    double sum = 0, mn = hist_at(h, 0), mx = mn;
    for (uint8_t i = 0; i < w; i++) {
        const double x = hist_at(h, i);
        sum += x; mn = x < mn ? x : mn; mx = x > mx ? x : mx;
    }
    const double mean = sum / w;
    switch (var->stat) {
    case aai16_RuleStat_STAT_MEAN: *out = mean; return true;
    case aai16_RuleStat_STAT_MIN:  *out = mn; return true;
    case aai16_RuleStat_STAT_MAX:  *out = mx; return true;
    default: {
        double ss = 0;
        for (uint8_t i = 0; i < w; i++) { const double d = hist_at(h, i) - mean; ss += d * d; }
        *out = sqrt(ss / w);
        return true;
    }
    }
}

static bool eval_when(const aai16_Rule *r, const double *vars, double *result)
{
    double st[8];
    uint32_t sp = 0;
    for (uint32_t t = 0; t < r->when_count; t++) {
        const aai16_CalcToken *tk = &r->when[t];
        switch (tk->op) {
        case aai16_CalcOp_CALC_PUSH_CONST: st[sp++] = tk->value; break;
        case aai16_CalcOp_CALC_PUSH_REG:   st[sp++] = vars[tk->reg]; break;
        case aai16_CalcOp_CALC_ADD: st[sp - 2] += st[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_SUB: st[sp - 2] -= st[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_MUL: st[sp - 2] *= st[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_DIV: if (st[sp - 1] == 0.0) return false; st[sp - 2] /= st[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_MIN: st[sp - 2] = fmin(st[sp - 2], st[sp - 1]); sp--; break;
        case aai16_CalcOp_CALC_MAX: st[sp - 2] = fmax(st[sp - 2], st[sp - 1]); sp--; break;
        case aai16_CalcOp_CALC_GT:  st[sp - 2] = st[sp - 2] > st[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_LT:  st[sp - 2] = st[sp - 2] < st[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_GE:  st[sp - 2] = st[sp - 2] >= st[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_LE:  st[sp - 2] = st[sp - 2] <= st[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_EQ:  st[sp - 2] = st[sp - 2] == st[sp - 1]; sp--; break;
        case aai16_CalcOp_CALC_AND: st[sp - 2] = (st[sp - 2] != 0) && (st[sp - 1] != 0); sp--; break;
        case aai16_CalcOp_CALC_OR:  st[sp - 2] = (st[sp - 2] != 0) || (st[sp - 1] != 0); sp--; break;
        case aai16_CalcOp_CALC_NOT: st[sp - 1] = st[sp - 1] == 0; break;
        case aai16_CalcOp_CALC_NEG: st[sp - 1] = -st[sp - 1]; break;
        case aai16_CalcOp_CALC_ABS: st[sp - 1] = fabs(st[sp - 1]); break;
        case aai16_CalcOp_CALC_SQRT: if (st[sp - 1] < 0) return false; st[sp - 1] = sqrt(st[sp - 1]); break;
        default: return false;
        }
        if (sp && !isfinite(st[sp - 1])) return false;
    }
    if (sp != 1) return false;
    *result = st[0];
    return true;
}

static void run_action(rule_engine_t *re, const aai16_Rule *r, double first_var)
{
    const re_hooks_t *h = &re->hk;
    const aai16_RuleAction *a = &r->action;
    switch (a->type) {
    case aai16_RuleActionType_RACT_LOG:
    case aai16_RuleActionType_RACT_NOTIFY:
        if (h->notify) h->notify(h->ctx, r->rule_id, a->text, first_var);
        break;
    case aai16_RuleActionType_RACT_ML_TRIGGER:
        if (h->ml_trigger) h->ml_trigger(h->ctx);
        break;
    case aai16_RuleActionType_RACT_REMOTE_CMD:
        if (h->remote_cmd) h->remote_cmd(h->ctx, a->target_node, a->cmd);
        break;
    case aai16_RuleActionType_RACT_SENSOR_START:
    case aai16_RuleActionType_RACT_SENSOR_STOP:
        if (!h->sensor_ctl ||
            !h->sensor_ctl(h->ctx, r->rule_id, a->sensor_id, a->type == aai16_RuleActionType_RACT_SENSOR_START, a->duration_ms)) {
            re->errors++; /* refused: the sensor is not dormant on this node, or its bus is busy */
        }
        break;
    default: /* GPIO / PWM: re-check ownership at fire time (a sensor may have claimed the pin since) */
        if (!h->pin_ok || !h->pin_ok(h->ctx, a->pin)) { re->errors++; return; }
        if (a->type == aai16_RuleActionType_RACT_GPIO_SET && h->gpio_set) h->gpio_set(h->ctx, a->pin, a->level);
        else if (a->type == aai16_RuleActionType_RACT_GPIO_TOGGLE && h->gpio_toggle) h->gpio_toggle(h->ctx, a->pin);
        else if (a->type == aai16_RuleActionType_RACT_GPIO_PULSE && h->gpio_pulse) h->gpio_pulse(h->ctx, a->pin, a->level, a->duration_ms);
        else if (a->type == aai16_RuleActionType_RACT_PWM && h->pwm) h->pwm(h->ctx, a->pin, a->duty_permille);
        break;
    }
}

void re_on_reading(rule_engine_t *re, uint32_t sensor_id, bool valid, const float *values, uint32_t n)
{
    for (uint32_t i = 0; i < re->count; i++) {
        re_rule_t *rr = &re->rules[i];
        if (!rr->rule.enabled) continue;
        bool touched = false;
        for (uint32_t v = 0; v < rr->rule.vars_count; v++) {
            const aai16_RuleVar *var = &rr->rule.vars[v];
            if (var->sensor_id != sensor_id) continue;
            touched = true;
            if (!valid || var->value_index >= n) { /* never evaluate on stale/invalid history */
                memset(&rr->hist[v], 0, sizeof(rr->hist[v]));
                rr->prev_true = false;
            } else {
                hist_push(&rr->hist[v], values[var->value_index]);
            }
        }
        if (!touched || !valid) continue;

        double vars[RE_MAX_VARS] = {0};
        bool ready = true;
        for (uint32_t v = 0; v < rr->rule.vars_count && ready; v++) {
            ready = var_value(&rr->hist[v], &rr->rule.vars[v], &vars[v]);
        }
        if (!ready) continue;
        double result;
        if (!eval_when(&rr->rule, vars, &result)) { re->errors++; continue; }
        const bool truth = result != 0.0;
        const bool edge_ok = !rr->rule.edge_triggered || !rr->prev_true;
        rr->prev_true = truth;
        if (!truth || !edge_ok) continue;
        const uint64_t now = re->hk.now_us ? re->hk.now_us(re->hk.ctx) : 0;
        if (rr->ever_fired && now - rr->last_fire_us < (uint64_t)rr->rule.cooldown_ms * 1000u) continue;
        rr->ever_fired = true;
        rr->last_fire_us = now;
        re->fired++;
        run_action(re, &rr->rule, vars[0]);
    }
}
