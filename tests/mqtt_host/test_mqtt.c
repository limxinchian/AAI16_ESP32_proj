#include <assert.h>
#include <float.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#include "platform.h"
#include "mqtt_manager_under_test.c"

struct fake_queue { unsigned capacity, size, count; unsigned char *items; };
static jmp_buf worker_idle;
static int64_t clock_us;
static int outbox_size, enqueue_result, enqueue_calls, starts, allocs;
static int publish_result, publish_calls;
static bool disconnect_during_publish;
static int outbox_queries;
static int64_t outbox_query_delay_us;
static int fail_step, step;
static int last_qos;
static char last_payload[TX_PAYLOAD_SIZE], last_topic[64];
static bool failure(void) { return ++step == fail_step; }

QueueHandle_t xQueueCreate(unsigned capacity, unsigned size)
{
    if (failure()) return NULL;
    QueueHandle_t q = calloc(1, sizeof(*q));
    assert(q);
    q->capacity = capacity; q->size = size;
    q->items = calloc(capacity, size);
    assert(q->items);
    allocs++;
    return q;
}
int xQueueSend(QueueHandle_t q, const void *item, unsigned wait)
{
    assert(wait == 0);
    if (q->count == q->capacity) return 0;
    memcpy(q->items + q->count++ * q->size, item, q->size);
    return pdTRUE;
}
int xQueueReceive(QueueHandle_t q, void *item, unsigned wait)
{
    if (!q->count) {
        if (wait != 0) longjmp(worker_idle, 1);
        return 0;
    }
    memcpy(item, q->items, q->size);
    q->count--;
    memmove(q->items, q->items + q->size, q->count * q->size);
    return pdTRUE;
}
unsigned uxQueueMessagesWaiting(QueueHandle_t q) { return q->count; }
int xQueuePeek(QueueHandle_t q, void *item, unsigned wait)
{
    assert(wait == 0);
    if (!q->count) return 0;
    memcpy(item, q->items, q->size);
    return pdTRUE;
}
void vQueueDelete(QueueHandle_t q) { free(q->items); free(q); allocs--; }
EventGroupHandle_t xEventGroupCreate(void)
{
    if (failure()) return NULL;
    allocs++; return calloc(1, sizeof(unsigned));
}
unsigned xEventGroupGetBits(EventGroupHandle_t g) { return *g; }
void xEventGroupSetBits(EventGroupHandle_t g, unsigned b) { *g |= b; }
void xEventGroupClearBits(EventGroupHandle_t g, unsigned b) { *g &= ~b; }
void vEventGroupDelete(EventGroupHandle_t g) { free(g); allocs--; }
int xTaskCreate(void (*f)(void *), const char *n, unsigned s, void *a,
                unsigned p, TaskHandle_t *h)
{ if (failure()) return 0; *h = (void *)1; return pdPASS; }
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t t) { return 1024; }
esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *c)
{
    assert(c->outbox.limit == 4096 && !c->network.disable_auto_reconnect);
    assert(c->network.timeout_ms == 10000);
    if (failure()) return NULL;
    allocs++; return (void *)1;
}
int esp_mqtt_client_register_event(esp_mqtt_client_handle_t c, int i,
    void (*h)(void *, esp_event_base_t, int32_t, void *), void *a)
{ return failure() ? ESP_FAIL : ESP_OK; }
int esp_mqtt_client_start(esp_mqtt_client_handle_t c)
{ starts++; return failure() ? ESP_FAIL : ESP_OK; }
int esp_mqtt_client_stop(esp_mqtt_client_handle_t c) { return ESP_OK; }
int esp_mqtt_client_destroy(esp_mqtt_client_handle_t c) { allocs--; return ESP_OK; }
int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t c, const char *t, int q)
{ return 1; }
int esp_mqtt_client_enqueue(esp_mqtt_client_handle_t c, const char *t,
                           const char *d, int len, int qos, int retain, bool store)
{
    assert(qos == 1 && store && !retain && len > 0 && len < TX_PAYLOAD_SIZE);
    enqueue_calls++;
    memcpy(last_payload, d, len); last_payload[len] = 0;
    snprintf(last_topic, sizeof(last_topic), "%s", t);
    last_qos = qos;
    clock_us += 20000; // Simulated SDK latency, not measured ESP32 performance.
    return enqueue_result;
}
int esp_mqtt_client_publish(esp_mqtt_client_handle_t c, const char *t,
                           const char *d, int len, int qos, int retain)
{
    assert(qos == 0 && !retain && len > 0 && len < TX_PAYLOAD_SIZE);
    publish_calls++;
    memcpy(last_payload, d, len); last_payload[len] = 0;
    snprintf(last_topic, sizeof(last_topic), "%s", t);
    last_qos = qos;
    clock_us += 20000;
    if (disconnect_during_publish) {
        mqtt_event_handler(NULL, NULL, MQTT_EVENT_DISCONNECTED, NULL);
        return -1;
    }
    return publish_result;
}
int esp_mqtt_client_get_outbox_size(esp_mqtt_client_handle_t c)
{ outbox_queries++; clock_us += outbox_query_delay_us; return outbox_size; }
int64_t esp_timer_get_time(void) { return clock_us; }
uint32_t esp_random(void) { return 0x12345678; }
uint32_t esp_get_free_heap_size(void) { return 100000; }
uint32_t esp_get_minimum_free_heap_size(void) { return 90000; }

static void pump(void) { if (!setjmp(worker_idle)) mqtt_tx_task(NULL); }
static void event(int id) { mqtt_event_handler(NULL, NULL, id, NULL); }
static void command(const char *payload, int len, int total, int offset, bool retain)
{
    esp_mqtt_event_t e = {
        .topic = CONFIG_SET_TOPIC, .topic_len = strlen(CONFIG_SET_TOPIC),
        .data = (char *)payload, .data_len = len, .total_data_len = total,
        .current_data_offset = offset, .retain = retain
    };
    mqtt_event_handler(NULL, NULL, MQTT_EVENT_DATA, &e);
}
static void json_command(const char *s) { command(s, strlen(s), strlen(s), 0, false); }
static void rejected(const char *s, const char *reason)
{
    json_command(s); pump();
    assert(strstr(last_payload, "\"status\":\"rejected\""));
    assert(strstr(last_payload, reason));
    assert(last_qos == 1);
    mqtt_interval_update_t update;
    assert(!mqtt_manager_take_interval_update(&update));
}

int main(void)
{
    // Failure injection at every allocating/registration/start/task-init step.
    for (int i = 1; i <= 7; i++) {
        fail_step = i; step = 0;
        assert(mqtt_manager_init() != ESP_OK);
        assert(!s_client && !s_tx_queue && !s_mqtt_events && !s_config_queue);
        assert(allocs == 0);
    }
    fail_step = 0; step = 0; starts = 0;
    assert(mqtt_manager_init() == ESP_OK && starts == 1);
    assert(mqtt_manager_init() == ESP_ERR_INVALID_STATE);
    // No GOT_IP event needed; publishing offline is explicitly counted.
    assert(mqtt_manager_publish_dht22(25, 50) == ESP_ERR_INVALID_STATE);
    assert(s_stats[TX_DHT22].attempts == 1 && s_stats[TX_DHT22].offline == 1);
    event(MQTT_EVENT_CONNECTED);
    clock_us = 1234000;
    assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK);
    assert(enqueue_calls == 0 && publish_calls == 0); // Sensor never invokes MQTT.
    pump();
    cJSON *data = cJSON_Parse(last_payload);
    assert(data && cJSON_GetObjectItem(data, "seq")->valueint == 2);
    assert(cJSON_GetObjectItem(data, "uptime_ms")->valueint == 1234);
    assert(!strcmp(cJSON_GetObjectItem(data, "boot_id")->valuestring, "12345678"));
    cJSON_Delete(data);
    assert(last_qos == 0 && s_ack_pubacked == 0);
    assert(s_stats[TX_DHT22].queued == 1 && s_stats[TX_DHT22].publish_ok == 1);
    assert(s_stats[TX_DHT22].enqueued == 0 && enqueue_calls == 0);
    assert(s_stats[TX_DHT22].max_tx_call_us == 20000);
    int queries = outbox_queries;
    mqtt_manager_log_diagnostics();
    assert(outbox_queries == queries); // Diagnostics must never take SDK locks.

    assert(mqtt_manager_publish_gy511(1, -1, 0, 12, -23, 34) == ESP_OK);
    pump();
    assert(!strcmp(last_topic, GY511_DATA_TOPIC));
    data = cJSON_Parse(last_payload);
    assert(data && cJSON_GetObjectItem(data, "seq")->valueint == 1);
    assert(cJSON_IsObject(cJSON_GetObjectItem(data, "magnetic_field_ut")));
    cJSON_Delete(data);
    assert(mqtt_manager_publish_dht22(NAN, 50) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_publish_gy511(INFINITY, 0, 0, 0, 0, 0) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_publish_gy511(FLT_MAX, FLT_MAX, FLT_MAX,
        FLT_MAX, FLT_MAX, FLT_MAX) == ESP_ERR_INVALID_SIZE);

    for (int i = 0; i < TX_QUEUE_SIZE; i++)
        assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK);
    assert(mqtt_manager_publish_dht22(25, 50) == ESP_ERR_NO_MEM);
    assert(s_stats[TX_DHT22].queue_full == 1);
    pump();
    assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK);
    clock_us += 2001000; pump();
    assert(s_stats[TX_DHT22].stale == 1);
    assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK);
    int publications = publish_calls;
    outbox_query_delay_us = 2001000; pump();
    outbox_query_delay_us = 0;
    // A slow diagnostic query now happens AFTER sending this fresh reading.
    assert(publish_calls == publications + 1 && s_stats[TX_DHT22].stale == 1);
    assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK);
    event(MQTT_EVENT_DISCONNECTED); pump();
    assert(s_stats[TX_DHT22].offline == 2);
    event(MQTT_EVENT_CONNECTED);
    // Regression: telemetry must not be rejected by either the old 3072-byte
    // allowance or an ACK-filled outbox. It creates no SDK outbox entry.
    const int occupancies[] = {3014, 3034, 4096};
    for (unsigned i = 0; i < 3; ++i) {
        outbox_size = occupancies[i];
        publications = publish_calls;
        assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK);
        assert(mqtt_manager_publish_gy511(1, 0, 0, 10, 20, 30) == ESP_OK);
        pump();
        assert(publish_calls == publications + 2 && outbox_size == occupancies[i]);
        assert(s_stats[TX_DHT22].outbox_full == 0 && s_stats[TX_GY511].outbox_full == 0);
    }
    assert(mqtt_manager_publish_config_ack(SENSOR_DHT22, 5000) == ESP_OK); pump();
    assert(strstr(last_payload, "\"status\":\"applied\"") && last_qos == 1);
    mqtt_interval_update_t unavailable = {SENSOR_DHT22, 5000};
    assert(mqtt_manager_reject_config(&unavailable, "sensor_unavailable") == ESP_OK); pump();
    assert(!strcmp(last_topic, CONFIG_ACK_TOPIC) && last_qos == 1);
    assert(strstr(last_payload, "\"sensor\":\"dht22\""));
    assert(strstr(last_payload, "\"status\":\"rejected\""));
    assert(strstr(last_payload, "\"reason\":\"sensor_unavailable\""));
    assert(!strstr(last_payload, "applied") && strstr(last_payload, "\"interval_ms\":5000"));
    unavailable = (mqtt_interval_update_t){SENSOR_GY511, 1000};
    assert(mqtt_manager_reject_config(&unavailable, "sensor_unavailable") == ESP_OK); pump();
    assert(strstr(last_payload, "\"sensor\":\"gy511\"") && last_qos == 1);
    assert(strstr(last_payload, "\"interval_ms\":1000"));
    outbox_size = 0;
    enqueue_result = -2;
    assert(mqtt_manager_publish_config_ack(SENSOR_DHT22, 5000) == ESP_OK); pump();
    assert(s_stats[TX_CONFIG_ACK].outbox_full == 1);
    enqueue_result = -1;
    assert(mqtt_manager_publish_config_ack(SENSOR_DHT22, 5000) == ESP_OK); pump();
    assert(s_stats[TX_CONFIG_ACK].enqueue_failed == 1);
    publish_result = -1;
    assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK); pump();
    assert(s_stats[TX_DHT22].publish_failed == 1);
    publish_result = 0;
    disconnect_during_publish = true;
    assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK); pump();
    assert(s_stats[TX_DHT22].publish_failed == 2);
    disconnect_during_publish = false;
    event(MQTT_EVENT_CONNECTED);
    enqueue_result = 0;
    event(MQTT_EVENT_PUBLISHED); event(MQTT_EVENT_DELETED);
    assert(s_ack_pubacked == 1 && s_outbox_deleted == 1);

    const unsigned valid[] = {2000, 5000, 10000};
    for (unsigned i = 0; i < 3; i++) {
        char payload[100]; mqtt_interval_update_t update;
        snprintf(payload, sizeof(payload), "{\"sensor\":\"dht22\",\"interval_ms\":%u}", valid[i]);
        json_command(payload);
        assert(mqtt_manager_take_interval_update(&update) && update.interval_ms == valid[i]);
        assert(update.sensor == SENSOR_DHT22);
    }
    rejected("{", "invalid_json");
    assert(strstr(last_payload, "\"sensor\":null"));
    rejected("[]", "invalid_json");
    rejected("{} trailing", "invalid_json");
    rejected("{}", "missing_or_invalid_sensor");
    rejected("{\"sensor\":\"future\",\"interval_ms\":5000}", "unsupported_sensor");
    assert(strstr(last_payload, "\"sensor\":\"future\""));
    rejected("{\"sensor\":\"dht22\",\"interval_ms\":\"5000\"}", "missing_or_invalid_interval_ms");
    rejected("{\"sensor\":\"dht22\",\"interval_ms\":1999}", "integer from 2000 to 10000");
    rejected("{\"sensor\":\"dht22\",\"interval_ms\":10001}", "integer from 2000 to 10000");
    rejected("{\"sensor\":\"dht22\",\"interval_ms\":2500.5}", "integer from 2000 to 10000");
    rejected("{\"sensor\":\"dht22\",\"interval_ms\":1e999}", "integer from 2000 to 10000");
    const char *valid_cmd = "{\"sensor\":\"dht22\",\"interval_ms\":5000}";
    int len = strlen(valid_cmd), calls = enqueue_calls;
    command(valid_cmd, len, len, 0, true); pump();
    command(valid_cmd, len, len, 1, false); pump();
    assert(enqueue_calls == calls); // Retained commands and continuations ignored.
    command(valid_cmd, len - 1, len, 0, false); pump();
    assert(strstr(last_payload, "invalid_payload_size_or_fragmented"));
    const char embedded[] = "{}\0x";
    command(embedded, sizeof(embedded) - 1, sizeof(embedded) - 1, 0, false); pump();
    assert(strstr(last_payload, "invalid_json"));
    char oversized[257]; memset(oversized, ' ', sizeof(oversized));
    command(oversized, sizeof(oversized), sizeof(oversized), 0, false); pump();
    assert(strstr(last_payload, "invalid_payload_size_or_fragmented"));
    for (int i = 0; i < CONFIG_QUEUE_SIZE + 1; i++) json_command(valid_cmd);
    pump(); assert(strstr(last_payload, "queue_full"));
    for (int i = 0; i < CONFIG_QUEUE_SIZE; i++) {
        mqtt_interval_update_t update;
        assert(mqtt_manager_take_interval_update(&update) && update.interval_ms == 5000);
        assert(update.sensor == SENSOR_DHT22);
    }

    // One parser and FIFO for both sensors; peeking must not consume or ACK.
    const mqtt_interval_update_t burst[] = {
        {SENSOR_GY511, 1000}, {SENSOR_DHT22, 5000},
        {SENSOR_GY511, 2000}, {SENSOR_GY511, 500}
    };
    uint32_t ack_attempts = s_stats[TX_CONFIG_ACK].attempts;
    for (unsigned i = 0; i < 4; ++i) {
        char payload[100];
        snprintf(payload, sizeof(payload), "{\"sensor\":\"%s\",\"interval_ms\":%u}",
                 burst[i].sensor == SENSOR_GY511 ? "gy511" : "dht22", burst[i].interval_ms);
        json_command(payload);
    }
    assert(s_stats[TX_CONFIG_ACK].attempts == ack_attempts); // Queued is not applied.
    json_command("{\"sensor\":\"gy511\",\"interval_ms\":10000}"); pump();
    assert(strstr(last_payload, "queue_full") && strstr(last_payload, "\"sensor\":\"gy511\""));
    assert(strstr(last_payload, "\"interval_ms\":10000"));
    for (unsigned i = 0; i < 4; ++i) {
        mqtt_interval_update_t peek, update;
        assert(mqtt_manager_peek_interval_update(&peek));
        assert(uxQueueMessagesWaiting(s_config_queue) == 4 - i);
        assert(mqtt_manager_take_interval_update(&update));
        assert(peek.sensor == burst[i].sensor && peek.interval_ms == burst[i].interval_ms);
        assert(update.sensor == peek.sensor && update.interval_ms == peek.interval_ms);
        assert(mqtt_manager_publish_config_ack(update.sensor, update.interval_ms) == ESP_OK); pump();
        data = cJSON_Parse(last_payload);
        assert(!strcmp(cJSON_GetObjectItem(data, "sensor")->valuestring,
                       update.sensor == SENSOR_GY511 ? "gy511" : "dht22"));
        assert(cJSON_GetObjectItem(data, "interval_ms")->valueint == (int)update.interval_ms);
        assert(last_qos == 1); cJSON_Delete(data);
    }
    const unsigned gy_valid[] = {500, 501, 1000, 2000, 10000};
    for (unsigned i = 0; i < sizeof(gy_valid) / sizeof(gy_valid[0]); ++i) {
        char payload[100]; mqtt_interval_update_t update;
        snprintf(payload, sizeof(payload), "{\"sensor\":\"gy511\",\"interval_ms\":%u}", gy_valid[i]);
        json_command(payload);
        assert(mqtt_manager_take_interval_update(&update) && update.sensor == SENSOR_GY511);
        assert(update.interval_ms == gy_valid[i]);
    }
    rejected("{\"sensor\":\"gy511\",\"interval_ms\":499}", "integer from 500 to 10000");
    assert(strstr(last_payload, "\"sensor\":\"gy511\"") && strstr(last_payload, "\"interval_ms\":499"));
    rejected("{\"sensor\":\"gy511\",\"interval_ms\":10001}", "integer from 500 to 10000");
    rejected("{\"sensor\":\"gy511\",\"interval_ms\":500.5}", "integer from 500 to 10000");
    rejected("{\"sensor\":\"gy511\",\"interval_ms\":-1}", "integer from 500 to 10000");
    rejected("{\"sensor\":\"gy511\",\"interval_ms\":1e999}", "integer from 500 to 10000");
    rejected("{\"sensor\":\"gy511\",\"interval_ms\":\"1000\"}", "missing_or_invalid_interval_ms");
    rejected("{\"sensor\":\"gy511\"}", "missing_or_invalid_interval_ms");
    rejected("{\"sensor\":\"odd\\\"name\",\"interval_ms\":1000}", "unsupported_sensor");
    data = cJSON_Parse(last_payload);
    assert(data && !strcmp(cJSON_GetObjectItem(data, "sensor")->valuestring, "odd\"name"));
    cJSON_Delete(data);

    // GY-511 obeys the existing retained/fragmented-input policy too.
    valid_cmd = "{\"sensor\":\"gy511\",\"interval_ms\":1000}";
    len = strlen(valid_cmd); calls = enqueue_calls;
    command(valid_cmd, len, len, 0, true); pump();
    command(valid_cmd, len, len, 1, false); pump();
    assert(enqueue_calls == calls);
    command(valid_cmd, len - 1, len, 0, false); pump();
    assert(strstr(last_payload, "invalid_payload_size_or_fragmented"));
    assert(strstr(last_payload, "\"sensor\":null"));
    for (int i = 0; i < TX_KIND_COUNT; i++) {
        tx_stats_t *s = &s_stats[i];
        assert(s->attempts == s->enqueued + s->publish_ok + s->offline + s->queue_full +
               s->invalid + s->stale + s->outbox_full + s->enqueue_failed + s->publish_failed);
    }
    puts("PASS: init cleanup, bounded handoff, sequence/JSON, drop accounting, ACKs, configuration validation");
    return 0;
}
