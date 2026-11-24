#pragma once
/* Rule Engine: bounded, Protobuf-configured rules over sensor values.
 *
 * Each rule has up to 6 variables (a statistic over a sliding window of one sensor value), an RPN
 * `when` expression over them and ONE action. Safety properties enforced by construction/validation:
 *  - expressions are stack-checked at apply time; evaluation is bounded (<= 24 tokens, depth 8)
 *  - actions never re-enter the engine (no recursion / loops); remote commands need >= 1 s cooldown
 *  - output pins must pass the node's pin policy at apply time AND again when firing
 *  - a new rule set replaces the old one atomically, or is rejected wholesale
 * Pure C; all side effects go through re_hooks_t.
 */
#include <stdbool.h>
#include <stdint.h>

#include "aai16.pb.h"

#define RE_MAX_RULES 8
#define RE_MAX_VARS  6
#define RE_HIST      16
#define RE_MIN_COOLDOWN_MS        100
#define RE_MIN_REMOTE_COOLDOWN_MS 1000
#define RE_MAX_COOLDOWN_MS        3600000
#define RE_MAX_WAKE_MS            3600000 /* longest RACT_SENSOR_START duration */

typedef struct {
    void (*gpio_set)(void *c, uint32_t pin, bool level);
    void (*gpio_toggle)(void *c, uint32_t pin);
    void (*gpio_pulse)(void *c, uint32_t pin, bool level, uint32_t ms);
    void (*pwm)(void *c, uint32_t pin, uint32_t permille);
    void (*notify)(void *c, uint32_t rule_id, const char *text, double value);
    void (*ml_trigger)(void *c);
    void (*remote_cmd)(void *c, const char *node, aai16_RemoteCmd cmd);
    bool (*pin_ok)(void *c, uint32_t pin); /* output allowed and not owned by a running sensor */
    uint64_t (*now_us)(void *c);
    void *ctx;
    /* RACT_SENSOR_START (start = true; duration_ms > 0 = stop again after that long) / RACT_SENSOR_STOP.
     * Returns false when refused (unknown or not dormant, bus busy, ...). Appended last so existing initialisers stay valid. */
    bool (*sensor_ctl)(void *c, uint32_t rule_id, uint32_t sensor_id, bool start, uint32_t duration_ms);
} re_hooks_t;

typedef struct {
    float v[RE_HIST];
    uint8_t n, head; /* ring of the last n samples; head = next write index */
} re_hist_t;

typedef struct {
    aai16_Rule rule;
    re_hist_t hist[RE_MAX_VARS];
    uint64_t last_fire_us;
    bool ever_fired, prev_true;
} re_rule_t;

typedef struct {
    re_hooks_t hk;
    re_rule_t rules[RE_MAX_RULES];
    uint32_t count, generation;
    uint32_t fired, errors; /* counters for the heartbeat */
} rule_engine_t;

typedef struct {
    aai16_Reason reason;
    uint32_t rule_id;
    uint32_t detail; /* token / variable index */
} re_error_t;

void re_init(rule_engine_t *re, const re_hooks_t *hk);
bool re_validate(const aai16_RuleSetApply *set, bool (*pin_ok)(void *, uint32_t), void *ctx, re_error_t *err);
/* Validate then replace all rules; stale/duplicate generation or any invalid rule leaves the old set. */
bool re_apply(rule_engine_t *re, const aai16_RuleSetApply *set, re_error_t *err);
/* Feed one reading; evaluates every rule that watches `sensor_id` and may fire actions. */
void re_on_reading(rule_engine_t *re, uint32_t sensor_id, bool valid, const float *values, uint32_t n);
void re_clear(rule_engine_t *re);
