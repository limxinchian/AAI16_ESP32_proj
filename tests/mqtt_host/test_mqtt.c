#include <assert.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#include <float.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>
#include "platform.h"
#include "cJSON.h"
#include "mqtt_manager_under_test.c"

struct fake_queue { unsigned capacity, size, count; unsigned char *items; };
static jmp_buf worker_idle;
static int64_t clock_us;
static int outbox_size, enqueue_result, enqueue_calls, starts, allocs;
static int publish_result, publish_calls;
static bool disconnect_during_publish;
static bool returned_idle, model_event_outbox, disconnect_during_enqueue, ack_during_query;
static int outbox_queries;
static int64_t outbox_query_delay_us;
static int fail_step, step;
static int last_qos, last_payload_len;
static esp_err_t queue_test_ack(void);
static char last_payload[AAI16_FRAME_MAX + 1], last_topic[64];
static unsigned model_received, model_disconnected;
bool model_upload_start(uint32_t boot, model_upload_reply_fn callback)
{ assert(boot == s_boot_id && callback == queue_model_reply); return true; }
void model_upload_receive(const char *topic, size_t topic_length, const void *data,
                          int length, int offset, int total, int id, bool retained)
{ ++model_received; }
void model_upload_disconnected(void) { ++model_disconnected; }
static _Thread_local unsigned critical_depth;
static pthread_mutex_t critical_mutex = PTHREAD_MUTEX_INITIALIZER;
static void (*publish_hook)(void);
static void (*distance_reader)(void *);
static void (*co2_reader)(void *);
static void (*reader_delay_hook)(TickType_t);
static void (*timer_hook)(void);
static unsigned timer_hook_countdown, last_worker_wait;
void test_enter_critical(void) { assert(!critical_depth); assert(!pthread_mutex_lock(&critical_mutex)); critical_depth++; }
void test_exit_critical(void) { assert(critical_depth == 1); critical_depth--; assert(!pthread_mutex_unlock(&critical_mutex)); }
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
    assert(wait == 0 && !critical_depth);
    if (q->count == q->capacity) return 0;
    memcpy(q->items + q->count++ * q->size, item, q->size);
    return pdTRUE;
}
int xQueueReceive(QueueHandle_t q, void *item, unsigned wait)
{
    assert(!critical_depth);
    if (wait) last_worker_wait = wait;
    if (!q->count) {
        if (wait != 0) {
            if (returned_idle) longjmp(worker_idle, 1);
            returned_idle = true;
        }
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
{
    assert(s == 4096 && p == 3 && !a);
    if (!strcmp(n, "hcsr04p_reader")) {
        assert(h); *h = (void *)2; distance_reader = f; return pdPASS;
    }
    if (!strcmp(n, "mhz1911a_reader")) {
        assert(h); *h = &co2_reader; co2_reader = f; return pdPASS;
    }
    assert(!strcmp(n, "mqtt_tx") && h);
    if (failure()) return 0;
    *h = (void *)1; return pdPASS;
}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t t) { return 1024; }
esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *c)
{
    assert(c->outbox.limit == 5120 && !c->network.disable_auto_reconnect);
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
    assert(!critical_depth && qos == 1 && store && !retain && len > 0 && len <= AAI16_FRAME_MAX);
    assert(!strcmp(t, MODEL_ACK_TOPIC) || !strcmp(t, MODEL_STATUS_TOPIC) || len < TX_PAYLOAD_SIZE);
    enqueue_calls++;
    memcpy(last_payload, d, len); last_payload[len] = 0; last_payload_len = len;
    snprintf(last_topic, sizeof(last_topic), "%s", t);
    last_qos = qos;
    clock_us += 20000; // Simulated SDK latency, not measured ESP32 performance.
    if (model_event_outbox && enqueue_result >= 0) outbox_size += len + strlen(t) + 9;
    if (disconnect_during_enqueue) mqtt_event_handler(NULL, NULL, MQTT_EVENT_DISCONNECTED, NULL);
    return enqueue_result;
}
int esp_mqtt_client_publish(esp_mqtt_client_handle_t c, const char *t,
                           const char *d, int len, int qos, int retain)
{
    assert(!critical_depth && qos == 0 && !retain && len > 0 && len < (int)sizeof(last_payload));
    assert(!strcmp(t, LD2450_DATA_TOPIC) || len < TX_PAYLOAD_SIZE);
    publish_calls++;
    memcpy(last_payload, d, len); last_payload[len] = 0; last_payload_len = len;
    snprintf(last_topic, sizeof(last_topic), "%s", t);
    last_qos = qos;
    clock_us += 20000;
    if (publish_hook) publish_hook();
    if (disconnect_during_publish) {
        mqtt_event_handler(NULL, NULL, MQTT_EVENT_DISCONNECTED, NULL);
        return -1;
    }
    return publish_result;
}
int esp_mqtt_client_get_outbox_size(esp_mqtt_client_handle_t c)
{
    assert(!critical_depth);
    outbox_queries++; clock_us += outbox_query_delay_us;
    if (ack_during_query) {
        ack_during_query = false;
        assert(queue_test_ack() == ESP_OK);
    }
    return outbox_size;
}
int64_t esp_timer_get_time(void)
{
    assert(!critical_depth);
    if (timer_hook && --timer_hook_countdown == 0) {
        void (*hook)(void) = timer_hook; timer_hook = NULL; hook();
    }
    return clock_us;
}
uint32_t esp_random(void) { return 0x12345678; }
uint32_t esp_get_free_heap_size(void) { return 100000; }
uint32_t esp_get_minimum_free_heap_size(void) { return 90000; }

static void pump(void) { returned_idle = false; if (!setjmp(worker_idle)) mqtt_tx_task(NULL); }
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
#include "test_config_protocol.inc"
#include "test_motion_protocol.inc"

// RC522 events are binary aai16.Envelope sensor telemetry, never JSON. Decode the
// exact bytes handed to the SDK, using the bounded protocol wrapper.
static aai16_Envelope decode_last_event(void)
{
    aai16_Envelope e;
    assert(last_payload_len > 0 && last_payload_len < TX_PAYLOAD_SIZE);
    assert(aai16_protocol_decode((const uint8_t *)last_payload, (size_t)last_payload_len, &e) == aai16_ResultCode_RESULT_OK);
    assert(e.has_protocol_version && e.protocol_version == AAI16_PROTOCOL_VERSION);
    assert(e.which_body == aai16_Envelope_sensor_telemetry_tag);
    assert(e.has_boot_id && e.has_sequence && e.has_generation && !e.has_request_id && !e.has_phase);
    return e;
}

static void test_rfid_events(void)
{
    // Embedded zero bytes (index 4) must survive: the payload is length-delimited.
    const uint8_t uid[] = {0x5a, 0x2d, 0xfd, 0x03, 0, 0x80, 0xff, 0x01, 0x10, 0xab};
    tx_stats_t *stats = &s_stats[TX_RC522_EVENT];
    outbox_size = 0; enqueue_result = 1; clock_us = 127595000;
    for (unsigned i = 0; i < 3; ++i) {
        unsigned length = 4 + 3 * i;
        uint8_t temporary[10]; memcpy(temporary, uid, sizeof(uid));
        int calls = enqueue_calls, publications = publish_calls;
        int64_t submitted_us = clock_us;
        assert(mqtt_manager_publish_rc522_event(temporary, length) == ESP_OK);
        memset(temporary, 0, sizeof(temporary)); // Local queue owns the complete event.
        assert(enqueue_calls == calls && publish_calls == publications);
        service_event_queue();
        assert(enqueue_calls == calls + 1 && last_qos == 1 && !strcmp(last_topic, RC522_EVENT_TOPIC));
        aai16_Envelope e = decode_last_event();
        const aai16_SensorTelemetry *t = &e.body.sensor_telemetry;
        assert(e.boot_id == 0x12345678 && e.sequence == i + 1);
        assert(e.generation == s_transactions.generation[SENSOR_RC522]);
        assert(t->sensor == aai16_SensorId_SENSOR_RC522 && t->which_reading == aai16_SensorTelemetry_rc522_tag);
        assert(t->acquired_us == (uint64_t)submitted_us && t->uptime_us == t->acquired_us);
        assert(t->reading.rc522.has_uid && t->reading.rc522.uid.size == length);
        assert(!memcmp(t->reading.rc522.uid.bytes, uid, length));
    }
    assert(mqtt_manager_publish_rc522_event(NULL, 4) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_publish_rc522_event(uid, 5) == ESP_ERR_INVALID_ARG);
    QueueHandle_t saved = s_event_queue; s_event_queue = NULL;
    assert(mqtt_manager_publish_rc522_event(uid, 4) == ESP_ERR_INVALID_STATE);
    s_event_queue = saved;
    assert(stats->invalid == 2 && stats->offline == 1);

    // No data/idle means no publication. Offline presentations retain their
    // original timestamp/sequence, beyond the periodic telemetry's 2 s expiry.
    int calls = enqueue_calls;
    service_event_queue(); assert(enqueue_calls == calls);
    event(MQTT_EVENT_DISCONNECTED);
    uint32_t oldest_seq = stats->attempts + 1;
    for (unsigned i = 0; i < EVENT_QUEUE_SIZE; ++i) assert(mqtt_manager_publish_rc522_event(uid, 4) == ESP_OK);
    assert(mqtt_manager_publish_rc522_event(uid, 4) == ESP_ERR_NO_MEM);
    assert(stats->queue_full == 1 && s_event_offline_buffered == EVENT_QUEUE_SIZE);
    clock_us += 3000000; service_event_queue();
    assert(enqueue_calls == calls && uxQueueMessagesWaiting(s_event_queue) == EVENT_QUEUE_SIZE);
    event(MQTT_EVENT_CONNECTED);
    model_event_outbox = true;
    // Earlier unacknowledged events already hold most of the event allowance, so
    // the test exercises backpressure whatever the encoded event size is. (A
    // Protobuf event is under half the old JSON size: eight of them no longer
    // fill the allowance on their own.)
    outbox_size = EVENT_OUTBOX_BUDGET_BYTES - 300;
    for (unsigned i = 0; i < EVENT_QUEUE_SIZE; ++i) service_event_queue();
    assert(outbox_size > 0 && outbox_size <= EVENT_OUTBOX_BUDGET_BYTES);
    assert(OUTBOX_LIMIT_BYTES - outbox_size >= ACK_OUTBOX_RESERVE_BYTES);
    unsigned waiting = uxQueueMessagesWaiting(s_event_queue);
    assert(waiting > 0 && waiting < EVENT_QUEUE_SIZE && s_event_outbox_waits > 0);
    aai16_Envelope buffered = decode_last_event();
    assert(buffered.sequence == oldest_seq + EVENT_QUEUE_SIZE - waiting - 1);

    // Full event queues/budget cannot take any existing ACK/telemetry slot.
    calls = enqueue_calls;
    assert(queue_test_ack() == ESP_OK);
    service_event_queue(); assert(enqueue_calls == calls); // ACK queued: defer event.
    uint32_t ack_accepted = s_stats[TX_CONFIG_ACK].enqueued;
    pump();
    assert(s_stats[TX_CONFIG_ACK].enqueued == ack_accepted + 1 && !strcmp(last_topic, CONFIG_ACK_TOPIC));
    int publications = publish_calls;
    assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK);
    assert(mqtt_manager_publish_gy511(1, 0, 0, 10, 20, 30) == ESP_OK);
    pump(); assert(publish_calls == publications + 2);
    while (uxQueueMessagesWaiting(s_event_queue)) {
        outbox_size = 0; // Model completed broker receipts freeing the outbox.
        service_event_queue();
    }
    model_event_outbox = false; outbox_size = 0;

    // Local expiry is strictly > 60000 ms, even while disconnected.
    event(MQTT_EVENT_DISCONNECTED);
    assert(mqtt_manager_publish_rc522_event(uid, 7) == ESP_OK);
    clock_us += 60000000; service_event_queue();
    assert(uxQueueMessagesWaiting(s_event_queue) == 1);
    clock_us += 1000; service_event_queue();
    assert(!uxQueueMessagesWaiting(s_event_queue) && stats->stale == 1);
    event(MQTT_EVENT_CONNECTED);
    assert(mqtt_manager_publish_rc522_event(uid, 7) == ESP_OK);
    clock_us += 59000000; outbox_query_delay_us = 2000000;
    calls = enqueue_calls; service_event_queue(); outbox_query_delay_us = 0;
    assert(enqueue_calls == calls && stats->stale == 2); // Expired during SDK lock wait.

    // SDK capacity race/backpressure retries the same local event, never a new seq.
    assert(mqtt_manager_publish_rc522_event(uid, 10) == ESP_OK);
    uint32_t attempts = stats->attempts, accepted = stats->enqueued;
    enqueue_result = -2; service_event_queue();
    char original[TX_PAYLOAD_SIZE]; int original_len = last_payload_len;
    memcpy(original, last_payload, (size_t)original_len); // Binary: never strcpy().
    assert(uxQueueMessagesWaiting(s_event_queue) == 1 && stats->enqueue_failed == 0);
    enqueue_result = 1; service_event_queue();
    assert(last_payload_len == original_len && !memcmp(original, last_payload, (size_t)original_len));
    assert(stats->attempts == attempts && stats->enqueued == accepted + 1);
    assert(stats->outbox_full == 0); // Retained, not mislabeled as a drop.
    enqueue_result = -1;
    assert(mqtt_manager_publish_rc522_event(uid, 4) == ESP_OK);
    service_event_queue();
    assert(stats->enqueue_failed == 1 && !uxQueueMessagesWaiting(s_event_queue));
    enqueue_result = 1;

    // A disconnect during successful enqueue hands ownership to the SDK exactly once.
    disconnect_during_enqueue = true;
    assert(mqtt_manager_publish_rc522_event(uid, 4) == ESP_OK);
    service_event_queue(); disconnect_during_enqueue = false;
    calls = enqueue_calls; service_event_queue();
    assert(!uxQueueMessagesWaiting(s_event_queue) && enqueue_calls == calls);
    event(MQTT_EVENT_CONNECTED);

    // A configuration ACK arriving during an SDK snapshot still gets priority.
    assert(mqtt_manager_publish_rc522_event(uid, 4) == ESP_OK);
    ack_during_query = true; calls = enqueue_calls; service_event_queue();
    assert(enqueue_calls == calls && uxQueueMessagesWaiting(s_tx_queue) == 1);
    pump(); assert(!uxQueueMessagesWaiting(s_event_queue));
    assert(mqtt_manager_publish_rc522_event(uid, 4) == ESP_OK);
    calls = enqueue_calls; pump(); // Existing worker also services events on its idle timeout.
    assert(enqueue_calls == calls + 1 && !uxQueueMessagesWaiting(s_event_queue));

    uint32_t receipts = s_qos1_pubacked, expired = s_outbox_deleted;
    event(MQTT_EVENT_PUBLISHED); event(MQTT_EVENT_DELETED);
    assert(s_qos1_pubacked == receipts + 1 && s_outbox_deleted == expired + 1);
    int queries = outbox_queries; clock_us += 60000000;
    mqtt_manager_log_diagnostics(); assert(outbox_queries == queries);
    assert(stats->attempts == stats->enqueued + stats->queue_full + stats->invalid +
           stats->offline + stats->stale + stats->enqueue_failed);
    puts("PASS: RFID Protobuf/UID4/7/10 incl. embedded NUL, offline FIFO, expiry, drop accounting, ACK reserve/priority, QoS1 handoff");
}

#include "test_radar.c"
#include "test_hcsr04p.c"
#include "test_mhz1911a.c"
#include "test_mhz1911a_config.c"

int main(int argc, char **argv)
{
    if (argc == 3 && !strcmp(argv[1], "--motion-bridge")) {
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        assert(mqtt_manager_init() == ESP_OK); event(MQTT_EVENT_CONNECTED);
        clock_us = (INT64_C(1) << 54) + 5000123; // Exercise exact uint64 decoding.
        motion_result_t r = motion_fixture(!strcmp(argv[2], "b"));
        if (!strcmp(argv[2], "status")) { r.status = MOTION_INIT_FAILED; r.model_ready = false; }
        check_motion_publish(&r);
        assert(fwrite(last_payload, 1, (size_t)last_payload_len, stdout) == (size_t)last_payload_len);
        return 0;
    }
    if (argc == 3 && !strcmp(argv[1], "--rc522-bridge")) {
        // Emit the exact bytes the firmware hands to the SDK for one card
        // presentation. argv[2] is the UID as hex text (8, 14 or 20 digits).
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        uint8_t uid[10] = {0};
        size_t digits = strlen(argv[2]);
        assert(digits % 2 == 0 && digits / 2 <= sizeof(uid));
        for (size_t i = 0; i < digits / 2; ++i) {
            unsigned value;
            assert(sscanf(argv[2] + 2 * i, "%2x", &value) == 1);
            uid[i] = (uint8_t)value;
        }
        assert(mqtt_manager_init() == ESP_OK); event(MQTT_EVENT_CONNECTED);
        clock_us = (INT64_C(1) << 54) + 5000123; // Exercise exact uint64 decoding.
        assert(mqtt_manager_publish_rc522_event(uid, (uint8_t)(digits / 2)) == ESP_OK);
        service_event_queue();
        assert(enqueue_calls == 1 && last_qos == 1 && !strcmp(last_topic, RC522_EVENT_TOPIC));
        assert(fwrite(last_payload, 1, (size_t)last_payload_len, stdout) == (size_t)last_payload_len);
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "--hcsr04p-bridge")) {
        // Emit the exact bytes the real publisher hands to the SDK for one distance
        // reading: acquired 0.5 s before the message is built, configuration generation 7.
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        assert(mqtt_manager_init() == ESP_OK); event(MQTT_EVENT_CONNECTED);
        s_transactions.generation[SENSOR_HCSR04P] = 7;
        clock_us = INT64_C(99885041);
        assert(mqtt_manager_update_hcsr04p_snapshot(&distance_fixture, clock_us - 500000) == ESP_OK);
        distance_due(); service_hcsr04p_snapshot();
        assert(publish_calls == 1 && last_qos == 0 && !strcmp(last_topic, HCSR04P_DATA_TOPIC));
        assert(fwrite(last_payload, 1, (size_t)last_payload_len, stdout) == (size_t)last_payload_len);
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "--mhz1911a-bridge")) {
        // Emit the exact bytes the real publisher hands to the SDK for one CO2 reading:
        // acquired 0.5 s before the message is built, configuration generation 7.
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        assert(mqtt_manager_init() == ESP_OK); event(MQTT_EVENT_CONNECTED);
        s_transactions.generation[SENSOR_MHZ1911A] = 7;
        clock_us = INT64_C(99885041);
        assert(mqtt_manager_update_mhz1911a_snapshot(1433, clock_us - 500000) == ESP_OK);
        service_mhz1911a_snapshot();
        assert(publish_calls == 1 && last_qos == 0 && !strcmp(last_topic, MHZ1911A_DATA_TOPIC));
        assert(fwrite(last_payload, 1, (size_t)last_payload_len, stdout) == (size_t)last_payload_len);
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "--ld2450-bridge")) {
        // Emit the exact bytes the real publisher hands to the SDK for one radar report:
        // slots 1 and 3 present (negative values at both extremes), slot 2 empty,
        // acquired 0.5 s before the message is built, configuration generation 7.
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        assert(mqtt_manager_init() == ESP_OK); event(MQTT_EVENT_CONNECTED);
        s_transactions.generation[SENSOR_LD2450] = 7;
        clock_us = INT64_C(99885041);
        s_radar_next_publish_us = clock_us;
        ld2450_report_t report = radar_fixture(5);
        assert(mqtt_manager_update_ld2450_snapshot(&report, clock_us - 500000) == ESP_OK);
        service_radar_snapshot();
        assert(publish_calls == 1 && last_qos == 0 && !strcmp(last_topic, LD2450_DATA_TOPIC));
        assert(fwrite(last_payload, 1, (size_t)last_payload_len, stdout) == (size_t)last_payload_len);
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "--dht22-bridge")) {
        // Emit the exact bytes the real publisher hands to the SDK for one DHT22 reading
        // (negative temperature, so the float sign survives), generation 7.
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        assert(mqtt_manager_init() == ESP_OK); event(MQTT_EVENT_CONNECTED);
        s_transactions.generation[SENSOR_DHT22] = 7;
        clock_us = (INT64_C(1) << 54) + 5000123; // Exercise exact uint64 decoding.
        assert(mqtt_manager_publish_dht22(-12.3f, 71.8f) == ESP_OK);
        pump();
        assert(publish_calls == 1 && last_qos == 0 && !strcmp(last_topic, DHT22_DATA_TOPIC));
        assert(fwrite(last_payload, 1, (size_t)last_payload_len, stdout) == (size_t)last_payload_len);
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "--config-bridge")) {
#ifdef _WIN32
        _setmode(_fileno(stdin), _O_BINARY); _setmode(_fileno(stdout), _O_BINARY);
#endif
        uint8_t bytes[AAI16_CONFIG_FRAME_MAX + 1];
        size_t n = fread(bytes, 1, sizeof(bytes), stdin);
        assert(mqtt_manager_init() == ESP_OK); event(MQTT_EVENT_CONNECTED);
        command((char *)bytes, (int)n, (int)n, 0, false);
        if (uxQueueMessagesWaiting(s_config_queue)) {
            mqtt_config_update_t u = start_owner();
            // Deterministic owner completion; physical owner scheduling is
            // independently exercised in the sensor_host suite.
            assert(mqtt_manager_publish_config_ack(&u) == ESP_OK);
        }
        pump();
        if (!enqueue_calls) return 2;
        fwrite(last_payload, 1, (size_t)last_payload_len, stdout);
        return 0;
    }
    // Failure injection at every allocating/registration/start/task-init step.
    for (int i = 1; i <= 8; i++) {
        fail_step = i; step = 0;
        assert(mqtt_manager_init() != ESP_OK);
        assert(!s_client && !s_tx_queue && !s_event_queue && !s_mqtt_events && !s_config_queue);
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
    // DHT22 telemetry is binary aai16.Envelope, not JSON; clock_us = 1234000.
    aai16_Envelope dht = decode_last_event();
    assert(dht.sequence == 2 && dht.boot_id == 0x12345678 && dht.generation == 0);
    assert(dht.body.sensor_telemetry.sensor == aai16_SensorId_SENSOR_DHT22);
    assert(dht.body.sensor_telemetry.acquired_us == 1234000 && dht.body.sensor_telemetry.uptime_us == 1234000);
    assert(dht.body.sensor_telemetry.which_reading == aai16_SensorTelemetry_dht22_tag);
    assert(dht.body.sensor_telemetry.reading.dht22.temperature_c == 25.0f);
    assert(dht.body.sensor_telemetry.reading.dht22.humidity_pct == 50.0f);
    cJSON *data;
    assert(last_qos == 0 && s_qos1_pubacked == 0);
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
    // Datasheet limits: -40..80 C and 0..100 %RH. Outside is a failed read, counted invalid.
    {
        uint32_t invalid_before = s_stats[TX_DHT22].invalid, queued_before = s_stats[TX_DHT22].queued;
        assert(mqtt_manager_publish_dht22(25, NAN) == ESP_ERR_INVALID_ARG);
        assert(mqtt_manager_publish_dht22(INFINITY, 50) == ESP_ERR_INVALID_ARG);
        assert(mqtt_manager_publish_dht22(-40.1f, 50) == ESP_ERR_INVALID_ARG);
        assert(mqtt_manager_publish_dht22(80.1f, 50) == ESP_ERR_INVALID_ARG);
        assert(mqtt_manager_publish_dht22(25, -0.1f) == ESP_ERR_INVALID_ARG);
        assert(mqtt_manager_publish_dht22(25, 100.1f) == ESP_ERR_INVALID_ARG);
        assert(s_stats[TX_DHT22].invalid == invalid_before + 6 && s_stats[TX_DHT22].queued == queued_before);
        // The limits themselves are valid readings.
        assert(mqtt_manager_publish_dht22(-40, 0) == ESP_OK);
        assert(mqtt_manager_publish_dht22(80, 100) == ESP_OK);
        pump(); pump();
    }
    assert(mqtt_manager_publish_gy511(INFINITY, 0, 0, 0, 0, 0) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_publish_gy511(FLT_MAX, FLT_MAX, FLT_MAX,
        FLT_MAX, FLT_MAX, FLT_MAX) == ESP_ERR_INVALID_SIZE);

    // Acquisition metadata must survive queueing and a delayed publisher.
    int64_t acquired = clock_us - 100000;
    assert(mqtt_manager_publish_gy511_at(0.123f, -0.456f, 1.001f,
                                       12, -23, 34, acquired, -1250) == ESP_OK);
    clock_us += 100000; pump();
    data = cJSON_Parse(last_payload);
    assert(data && cJSON_GetObjectItem(data, "uptime_ms")->valuedouble == acquired / 1000);
    assert(cJSON_GetObjectItem(data, "jitter_us")->valueint == -1250);
    assert(cJSON_GetObjectItem(cJSON_GetObjectItem(data, "acceleration_g"), "y")->valuedouble == -0.456);
    cJSON_Delete(data);
    assert(last_qos == 0);
    assert(mqtt_manager_publish_gy511_at(0, 0, 1, 0, 0, 0, -1, 0) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_publish_gy511_at(0, 0, 1, 0, 0, 0, clock_us + 1, 0) == ESP_ERR_INVALID_ARG);
    uint32_t gy_stale = s_stats[TX_GY511].stale;
    clock_us += 3000000;
    assert(mqtt_manager_publish_gy511_at(0, 0, 1, 0, 0, 0, clock_us - 2001000, 0) == ESP_OK);
    pump(); assert(s_stats[TX_GY511].stale == gy_stale + 1);

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
    test_config_protocol();
    enqueue_result = -2; assert(queue_test_ack() == ESP_OK); pump();
    assert(s_stats[TX_CONFIG_ACK].outbox_full >= 1);
    enqueue_result = -1; assert(queue_test_ack() == ESP_OK); pump();
    assert(s_stats[TX_CONFIG_ACK].enqueue_failed >= 1);
    enqueue_result = 0;
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
    assert(s_qos1_pubacked == 1 && s_outbox_deleted == 1);

    test_rfid_events();

    // A disable ACK does not retract earlier presentations from the event FIFO.
    const uint8_t card[] = {0x5a, 0x2d, 0xfd, 3};
    assert(mqtt_manager_publish_rc522_event(card, sizeof(card)) == ESP_OK);
    aai16_Envelope disable_frame = config_frame(SENSOR_RC522, true, 0);
    send_frame(&disable_frame);
    mqtt_config_update_t disable = start_owner();
    assert(mqtt_manager_publish_config_ack(&disable) == ESP_OK);
    assert(uxQueueMessagesWaiting(s_event_queue) == 1);
    uint32_t events_enqueued = s_stats[TX_RC522_EVENT].enqueued;
    pump();
    assert(s_stats[TX_RC522_EVENT].enqueued == events_enqueued + 1 && last_qos == 1);
    assert(!strcmp(last_topic, RC522_EVENT_TOPIC));
    assert(!uxQueueMessagesWaiting(s_event_queue));

    test_radar_telemetry();
    test_radar_config();
    test_hcsr04p_telemetry();
    test_hcsr04p_config();
    test_mhz1911a_telemetry();
    test_mhz1911a_config();
    assert(mqtt_manager_publish_bme280(25, 1013.25f, 50, true) == ESP_OK); pump();
    assert(!strcmp(last_topic, BME280_DATA_TOPIC) && last_qos == 0);
    data = cJSON_Parse(last_payload);
    assert(data && cJSON_GetObjectItem(data, "pressure_hpa")->valuedouble == 1013.25);
    cJSON_Delete(data);
    assert(mqtt_manager_publish_max6675(123.25f) == ESP_OK); pump();
    assert(!strcmp(last_topic, MAX6675_DATA_TOPIC) && last_qos == 0);
    data = cJSON_Parse(last_payload);
    assert(data && cJSON_GetObjectItem(data, "temperature_c")->valuedouble == 123.25);
    cJSON_Delete(data);
    test_motion_telemetry();
    for (int i = 0; i < TX_KIND_COUNT; i++) {
        tx_stats_t *s = &s_stats[i];
        assert(s->attempts == s->enqueued + s->publish_ok + s->offline + s->queue_full +
               s->invalid + s->stale + s->outbox_full + s->enqueue_failed + s->publish_failed + s->cancelled);
    }
    {
        uint8_t reply[768] = {1, 0, 255};
        model_event_outbox = false;
        int calls = enqueue_calls;
        assert(!queue_model_reply(false, reply, 769));
        for (int i = 0; i < 4; ++i) assert(queue_model_reply(i == 0, reply, sizeof(reply)));
        assert(!queue_model_reply(false, reply, 1));
        assert(enqueue_calls == calls); // No SDK call on the upload worker.
        outbox_size = 300; service_model_replies(); assert(enqueue_calls == calls);
        event(MQTT_EVENT_DISCONNECTED); outbox_size = 0;
        service_model_replies(); assert(enqueue_calls == calls);
        event(MQTT_EVENT_CONNECTED); enqueue_result = -1;
        service_model_replies(); assert(s_model_reply_count == 4);
        enqueue_result = 1; service_model_replies();
        assert(!strcmp(last_topic, MODEL_STATUS_TOPIC) && last_payload_len == 768 && !memcmp(reply, last_payload, 768));
        assert(s_model_reply_count == 3);
        while (s_model_reply_count) service_model_replies();
        assert(!strcmp(last_topic, MODEL_ACK_TOPIC));
        unsigned received = model_received;
        esp_mqtt_event_t e = {.topic=MODEL_CHUNK_TOPIC, .topic_len=strlen(MODEL_CHUNK_TOPIC),
            .data=(char *)reply, .data_len=10, .total_data_len=10};
        mqtt_event_handler(NULL,NULL,MQTT_EVENT_DATA,&e);
        assert(model_received == received + 1 && model_disconnected > 0);
        puts("PASS: model routing, bounded reply mailbox, QoS1 binary ACK/status, reconnect and configuration outbox reserve");
    }
    puts("PASS: init cleanup, bounded handoff, sequence/JSON, drop accounting, ACKs, configuration validation");
    return 0;
}
