#include "config_transactions.h"
#include <string.h>
#define R(x) aai16_ResultCode_RESULT_##x
#define P(x) aai16_Phase_PHASE_##x

static const struct { sensor_id_t local; aai16_SensorId wire; } sensors[] = {
    {SENSOR_DHT22, aai16_SensorId_SENSOR_DHT22}, {SENSOR_GY511, aai16_SensorId_SENSOR_GY511},
    {SENSOR_RC522, aai16_SensorId_SENSOR_RC522}, {SENSOR_LD2450, aai16_SensorId_SENSOR_LD2450},
    {SENSOR_HCSR04P, aai16_SensorId_SENSOR_HCSR04P}, {SENSOR_MHZ1911A, aai16_SensorId_SENSOR_MHZ1911A},
    {SENSOR_BME280, aai16_SensorId_SENSOR_BME280}, {SENSOR_MAX6675, aai16_SensorId_SENSOR_MAX6675}
};
bool config_sensor_from_wire(aai16_SensorId wire, sensor_id_t *sensor)
{
    for (unsigned i = 0; i < sizeof(sensors)/sizeof(*sensors); ++i)
        if (sensors[i].wire == wire) { *sensor = sensors[i].local; return true; }
    return false;
}
aai16_SensorId config_sensor_to_wire(sensor_id_t sensor)
{
    for (unsigned i = 0; i < sizeof(sensors)/sizeof(*sensors); ++i)
        if (sensors[i].local == sensor) return sensors[i].wire;
    return aai16_SensorId_SENSOR_UNSPECIFIED;
}
void config_requested(const mqtt_config_update_t *u, aai16_SensorConfig *out)
{
    *out = (aai16_SensorConfig)aai16_SensorConfig_init_zero;
    out->has_sensor = true; out->sensor = config_sensor_to_wire(u->sensor);
    out->has_enabled = u->operation == MQTT_CONFIG_ENABLED;
    out->has_interval_ms = u->operation == MQTT_CONFIG_INTERVAL;
    if (out->has_enabled) out->enabled = u->enabled;
    if (out->has_interval_ms) out->interval_ms = u->interval_ms;
    out->has_expected_generation = u->has_expected_generation;
    out->expected_generation = u->expected_generation;
}
static bool same(const mqtt_config_update_t *a, const mqtt_config_update_t *b)
{
    return a->request_id == b->request_id && a->sensor == b->sensor && a->operation == b->operation &&
        (a->operation == MQTT_CONFIG_ENABLED ? a->enabled == b->enabled : a->interval_ms == b->interval_ms) &&
        a->has_expected_generation == b->has_expected_generation &&
        (!a->has_expected_generation || a->expected_generation == b->expected_generation) &&
        a->has_target_boot_id == b->has_target_boot_id &&
        (!a->has_target_boot_id || a->target_boot_id == b->target_boot_id);
}
static config_history_t *find(config_transactions_t *s, uint64_t id)
{
    for (unsigned i = 0; i < CONFIG_HISTORY_SIZE; ++i)
        if (s->history[i].used && s->history[i].receipt.update.request_id == id) return &s->history[i];
    return NULL;
}
int config_admit(config_transactions_t *s, const aai16_Envelope *e,
                 uint32_t boot_id, mqtt_config_update_t *u, config_receipt_t *reply)
{
    const aai16_SensorConfig *c = &e->body.sensor_config;
    memset(u, 0, sizeof(*u));
    if (!config_sensor_from_wire(c->sensor, &u->sensor)) return 0; // Validated caller contract.
    u->request_id = e->request_id;
    u->operation = c->has_enabled ? MQTT_CONFIG_ENABLED : MQTT_CONFIG_INTERVAL;
    if (c->has_enabled) u->enabled = c->enabled; else u->interval_ms = c->interval_ms;
    u->has_expected_generation = c->has_expected_generation; u->expected_generation = c->expected_generation;
    u->has_target_boot_id = e->has_boot_id; u->target_boot_id = e->boot_id;
    *reply = (config_receipt_t){.update = *u, .phase = P(VALIDATE), .generation = s->generation[u->sensor]};
    if (e->has_boot_id && e->boot_id != boot_id) { reply->result = R(STALE_GENERATION); return -1; }
    config_history_t *h = find(s, u->request_id);
    if (h) {
        if (!same(&h->receipt.update, u)) { reply->result = R(BUSY); return -1; }
        if (!h->complete) return 0;
        *reply = h->receipt; return -1;
    }
    // A bounded cache cannot remember every ID. Require monotonically increasing
    // request IDs per boot; evicted IDs fail closed instead of applying twice.
    if (u->request_id <= s->high_water_request) { reply->result = R(STALE_GENERATION); return -1; }
    for (unsigned n = 0; n < CONFIG_HISTORY_SIZE; ++n) {
        unsigned i = (s->replacement + n) % CONFIG_HISTORY_SIZE;
        if (!s->history[i].used || s->history[i].complete) {
            h = &s->history[i]; *h = (config_history_t){.receipt = *reply, .used = true};
            s->replacement = (i + 1) % CONFIG_HISTORY_SIZE;
            s->high_water_request = u->request_id;
            return 1;
        }
    }
    reply->result = R(BUSY); return -1;
}
bool config_finish(config_transactions_t *s, const mqtt_config_update_t *u,
                   aai16_ResultCode result, config_receipt_t *reply)
{
    config_history_t *h = find(s, u->request_id);
    if (!h || h->complete || !same(&h->receipt.update, u) || (result == R(OK) && !h->started)) return false;
    if (result == R(OK)) ++s->generation[u->sensor];
    h->receipt.result = result;
    h->receipt.phase = h->started ? P(APPLY) : P(VALIDATE);
    h->receipt.generation = s->generation[u->sensor];
    h->complete = true; *reply = h->receipt;
    return true;
}
bool config_begin(config_transactions_t *s, mqtt_config_update_t *u, config_receipt_t *failure)
{
    config_history_t *h = find(s, u->request_id);
    if (!h || h->complete || h->started || !same(&h->receipt.update, u)) return false;
    h->started = true;
    aai16_SensorConfig c; config_requested(u, &c);
    aai16_ResultCode r = aai16_config_validate(&c);
    uint32_t current = s->generation[u->sensor];
    if (r == R(OK) && ((u->has_expected_generation && u->expected_generation != current) || current == UINT32_MAX))
        r = R(STALE_GENERATION);
    if (r != R(OK)) { (void)config_finish(s, u, r, failure); return false; }
    u->generation = current + 1;
    return true;
}
bool config_stage(config_transactions_t *s, const mqtt_config_update_t *u)
{
    if ((unsigned)u->sensor >= SENSOR_STATUS_CAPACITY || s->occupied[u->sensor]) return false;
    config_history_t *h = find(s, u->request_id);
    if (!h || h->complete || h->started || !same(&h->receipt.update, u)) return false;
    s->sidecar[u->sensor] = *u; s->occupied[u->sensor] = true; return true;
}
bool config_take(config_transactions_t *s, sensor_id_t sensor, mqtt_config_update_t *u)
{
    if ((unsigned)sensor >= SENSOR_STATUS_CAPACITY || !s->occupied[sensor]) return false;
    *u = s->sidecar[sensor]; s->occupied[sensor] = false; return true;
}
void config_fragment_expire(config_fragments_t *s, int64_t now)
{
    if (s->active && now - s->started_us >= CONFIG_FRAGMENT_TIMEOUT_US) {
        s->active = false; ++s->expired;
    }
}
size_t config_fragment_feed(config_fragments_t *s, int64_t now, bool matches,
                            bool topic_present, bool retained, int id,
                            int offset, int total, const void *data, int length)
{
    config_fragment_expire(s, now);
    if (offset == 0 && topic_present) {
        if (s->active) ++s->rejected;
        s->active = false;
        if (!matches || retained) return 0;
        s->received = 0; s->total = total; s->message_id = id; s->started_us = now;
        s->active = true;
    }
    if (!s->active) return 0;
    if (retained || (topic_present && !matches) || id != s->message_id ||
        total <= 0 || total > AAI16_CONFIG_FRAME_MAX || total != s->total ||
        offset < 0 || offset != s->received || length < 0 ||
        (length == 0 && (!topic_present || offset != 0)) ||
        (length > 0 && !data) || length > total - offset) {
        s->active = false; ++s->rejected; return 0;
    }
    // ESP-MQTT may initially deliver only the topic/header. The following
    // data-only event then also has offset zero. Keep the original deadline.
    if (length == 0) return 0;
    memcpy(s->data + offset, data, (size_t)length);
    s->received += length;
    if (s->received != total) return 0;
    s->active = false;
    return (size_t)total;
}
