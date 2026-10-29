// Included by test_mqtt.c: real reader, status registry and MQTT manager;
// only acquisition, time, RTOS and SDK calls are deterministic host doubles.
#include "../../main/hcsr04p_reader.c"

static jmp_buf distance_reader_idle;
static int64_t distance_reader_until, distance_previous_start;
static unsigned distance_reads;
static esp_err_t distance_read_result = ESP_OK;
static hcsr04p_reading_t distance_fixture = {.distance_cm = 42.1f, .echo_us = 2456.3f};

const char *esp_err_to_name(esp_err_t error) { return error == ESP_OK ? "ESP_OK" : "test_error"; }
void vTaskDelete(TaskHandle_t task) { assert(false); }
void vTaskDelay(TickType_t ticks)
{
    if (reader_delay_hook) { reader_delay_hook(ticks); return; }
    assert(!critical_depth && ticks > 0 && ticks <= HCSR04P_SAMPLE_MS);
    clock_us += ticks * 1000LL;
    if (clock_us >= distance_reader_until) longjmp(distance_reader_idle, 1);
}
int xTaskNotifyWait(uint32_t entry, uint32_t exit, uint32_t *value, TickType_t wait)
{
    assert(!entry && exit == UINT32_MAX);
    if (wait) vTaskDelay(wait);
    return 0;
}
esp_err_t hcsr04p_sensor_init(void) { return ESP_OK; }
esp_err_t hcsr04p_sensor_read(hcsr04p_reading_t *reading)
{
    assert(!critical_depth);
    if (distance_reads) assert(clock_us - distance_previous_start == 200000);
    distance_previous_start = clock_us;
    distance_reads++;
    clock_us += distance_read_result == ESP_OK ? 4000 : 30000;
    *reading = distance_read_result == ESP_OK ? distance_fixture : (hcsr04p_reading_t){0};
    return distance_read_result;
}

static void run_distance_reader(int64_t duration_us, esp_err_t result)
{
    distance_read_result = result;
    distance_reads = 0;
    distance_reader_until = clock_us + duration_us;
    hcsr04p_reader_start();
    assert(distance_reader);
    if (!setjmp(distance_reader_idle)) distance_reader(NULL);
}

static void distance_due(void)
{
    if (clock_us < s_hcsr04p_next_publish_us) clock_us = s_hcsr04p_next_publish_us;
}

static void check_distance_json(const hcsr04p_reading_t *reading, int64_t acquired_us, uint32_t seq)
{
    assert(!strcmp(last_topic, HCSR04P_DATA_TOPIC) && last_qos == 0);
    cJSON *data = cJSON_Parse(last_payload);
    assert(data && cJSON_GetArraySize(data) == 6);
    assert(!strcmp(cJSON_GetObjectItem(data, "sensor")->valuestring, "hcsr04p"));
    assert(!strcmp(cJSON_GetObjectItem(data, "boot_id")->valuestring, "12345678"));
    assert(cJSON_GetObjectItem(data, "seq")->valuedouble == seq);
    assert(cJSON_GetObjectItem(data, "uptime_ms")->valuedouble == acquired_us / 1000);
    assert(fabs(cJSON_GetObjectItem(data, "distance_cm")->valuedouble - reading->distance_cm) < 0.051);
    assert(fabs(cJSON_GetObjectItem(data, "echo_us")->valuedouble - reading->echo_us) < 0.051);
    cJSON_Delete(data);
}

static void distance_age_during_format(void) { clock_us += HCSR04P_MAX_AGE_US; }
static void distance_disconnect_during_format(void) { event(MQTT_EVENT_DISCONNECTED); }

static void distance_reader_during_publish(void)
{
    publish_hook = NULL;
    int calls = publish_calls, enqueues = enqueue_calls, allocations = allocs;
    uint32_t updates = s_hcsr04p_updates;
    // The actual reader completes 25 acquisitions at 200 ms while the SDK
    // double is stalled for five seconds, replacing the mailbox each time.
    run_distance_reader(5000000, ESP_OK);
    assert(distance_reads == 25 && s_hcsr04p_updates == updates + 25);
    assert(publish_calls == calls && enqueue_calls == enqueues && allocs == allocations);
    assert(!uxQueueMessagesWaiting(s_tx_queue) && !uxQueueMessagesWaiting(s_event_queue));
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_HCSR04P, &status));
    assert(status.state == SENSOR_READY && status.read_successes == 25 && !status.read_failures);
}

static void test_hcsr04p_telemetry(void)
{
    tx_stats_t *stats = &s_stats[TX_HCSR04P];
    event(MQTT_EVENT_CONNECTED);
    publish_result = 0;
    int calls = publish_calls;
    distance_due(); pump();
    assert(publish_calls == calls && stats->attempts == 0); // No fabricated initial sample.

    char payload[HCSR04P_PAYLOAD_SIZE];
    int len = format_hcsr04p(payload, sizeof(payload), &distance_fixture, 25000999, 1);
    assert(len > 0 && !strcmp(payload,
        "{\"sensor\":\"hcsr04p\",\"boot_id\":\"12345678\",\"seq\":1,\"uptime_ms\":25000,\"distance_cm\":42.1,\"echo_us\":2456.3}"));
    hcsr04p_reading_t largest = {.distance_cm = HCSR04P_MAX_CM, .echo_us = HCSR04P_ECHO_TIMEOUT_MS * 1000};
    len = format_hcsr04p(payload, sizeof(payload), &largest, INT64_MAX, UINT32_MAX);
    assert(len > 0 && len < HCSR04P_PAYLOAD_SIZE && len < TX_PAYLOAD_SIZE);
    assert(strstr(payload, "\"seq\":4294967295") && strstr(payload, "\"uptime_ms\":9223372036854775"));
    // Every truncation boundary, exact fit (including NUL), and guard byte.
    for (size_t capacity = 0; capacity <= (size_t)len + 1; ++capacity) {
        unsigned char guarded[HCSR04P_PAYLOAD_SIZE + 1];
        memset(guarded, 0xa5, sizeof(guarded));
        int n = format_hcsr04p((char *)guarded, capacity, &largest, INT64_MAX, UINT32_MAX);
        assert(n == (capacity <= (size_t)len ? -1 : len));
        assert(guarded[capacity] == 0xa5);
        if (capacity) assert(guarded[capacity - 1] == 0);
    }
    uint32_t boot = s_boot_id;
    s_boot_id = 0xabcdef;
    assert(format_hcsr04p(payload, sizeof(payload), &distance_fixture, 0, 0) > 0);
    assert(strstr(payload, "\"boot_id\":\"00abcdef\"") && strstr(payload, "\"uptime_ms\":0"));
    s_boot_id = boot;
    puts("PASS: HC-SR04P exact JSON, boot/sequence/timestamp/units, precision and all buffer boundaries");

    // Five successful measurements per publication; mailbox owns a value copy.
    uint32_t coalesced = s_hcsr04p_coalesced;
    int allocations = allocs, enqueues = enqueue_calls;
    int64_t acquired = 0;
    hcsr04p_reading_t expected = distance_fixture;
    for (unsigned i = 0; i < 5; ++i) {
        clock_us += 200000;
        acquired = clock_us;
        expected.distance_cm = 40.0f + i;
        expected.echo_us = expected.distance_cm / 0.01715f;
        hcsr04p_reading_t local = expected;
        assert(mqtt_manager_update_hcsr04p_snapshot(&local, acquired) == ESP_OK);
        memset(&local, 0, sizeof(local));
        assert(publish_calls == calls && enqueue_calls == enqueues && allocs == allocations);
        pump();
        assert(publish_calls == calls + (i == 4));
    }
    assert(stats->attempts == 1 && s_hcsr04p_coalesced == coalesced + 4);
    check_distance_json(&expected, acquired, 1);
    calls = publish_calls;
    for (unsigned i = 0; i < 3; ++i) { distance_due(); pump(); }
    assert(publish_calls == calls); // Consumed measurement is never repeated.

    // Invalid updates neither replace a valid snapshot nor advance sequence.
    acquired = clock_us;
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, acquired) == ESP_OK);
    uint32_t updates = s_hcsr04p_updates, seq = stats->attempts + 1;
    uint32_t invalid_updates = s_hcsr04p_invalid;
    const hcsr04p_reading_t bad[] = {
        {NAN, 1000}, {INFINITY, 1000}, {-INFINITY, 1000}, {0, 1000}, {1.99f, 1000}, {400.1f, 1000},
        {20, NAN}, {20, INFINITY}, {20, -INFINITY}, {20, 0}, {20, -1}, {20, 30001},
    };
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
        assert(mqtt_manager_update_hcsr04p_snapshot(&bad[i], clock_us) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_hcsr04p_snapshot(NULL, clock_us) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, -1) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, clock_us + 1) == ESP_ERR_INVALID_ARG);
    assert(s_hcsr04p_invalid == invalid_updates + 15 && s_hcsr04p_updates == updates);
    assert(stats->attempts == seq - 1 && s_hcsr04p.acquired_us == acquired);
    distance_due(); pump(); check_distance_json(&expected, acquired, seq);

    // Exactly 2000 ms is stale; 1999.999 ms is still eligible at admission.
    distance_due(); acquired = clock_us - HCSR04P_MAX_AGE_US + 1;
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, acquired) == ESP_OK);
    seq = stats->attempts + 1; pump(); check_distance_json(&expected, acquired, seq);
    distance_due(); calls = publish_calls;
    uint32_t stale = stats->stale;
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, clock_us - HCSR04P_MAX_AGE_US) == ESP_OK);
    pump(); assert(publish_calls == calls && stats->stale == stale + 1 && !s_hcsr04p.pending);
    distance_due(); assert(mqtt_manager_update_hcsr04p_snapshot(&expected, clock_us) == ESP_OK);
    timer_hook = distance_age_during_format; timer_hook_countdown = 2;
    service_hcsr04p_snapshot();
    assert(!timer_hook && publish_calls == calls && stats->stale == stale + 2);

    // Disconnect drops one consumed attempt per interval; reconnect has no history.
    event(MQTT_EVENT_DISCONNECTED);
    uint32_t offline = stats->offline;
    for (unsigned i = 0; i < 3; ++i) {
        distance_due(); assert(mqtt_manager_update_hcsr04p_snapshot(&expected, clock_us) == ESP_OK);
        pump(); assert(!s_hcsr04p.pending);
    }
    assert(stats->offline == offline + 3 && publish_calls == calls);
    event(MQTT_EVENT_CONNECTED); distance_due(); pump(); assert(publish_calls == calls);
    // A snapshot left pending through a publisher stall must expire on reconnect.
    event(MQTT_EVENT_DISCONNECTED);
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, clock_us) == ESP_OK);
    clock_us += 3000000; event(MQTT_EVENT_CONNECTED); pump();
    assert(publish_calls == calls && stats->stale == stale + 3);
    distance_due(); acquired = clock_us;
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, acquired) == ESP_OK);
    seq = stats->attempts + 1; pump(); check_distance_json(&expected, acquired, seq);
    distance_due(); assert(mqtt_manager_update_hcsr04p_snapshot(&expected, clock_us) == ESP_OK);
    timer_hook = distance_disconnect_during_format; timer_hook_countdown = 2;
    calls = publish_calls; service_hcsr04p_snapshot();
    assert(!timer_hook && publish_calls == calls && stats->offline == offline + 4);
    event(MQTT_EVENT_CONNECTED);
    puts("PASS: HC-SR04P 5:1 coalescing, copy ownership, invalid input, freshness and reconnect/drop policy");

    // Shared queue full: ACK + DHT22 + GY511 and RFID still progress first.
    distance_due();
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, clock_us) == ESP_OK);
    assert(queue_test_ack() == ESP_OK);
    assert(mqtt_manager_publish_gy511(1, 2, 3, 4, 5, 6) == ESP_OK);
    for (unsigned i = 2; i < TX_QUEUE_SIZE; ++i) assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK);
    const uint8_t uid[] = {1, 2, 3, 4};
    assert(mqtt_manager_publish_rc522_event(uid, sizeof(uid)) == ESP_OK);
    uint32_t acks = s_stats[TX_CONFIG_ACK].enqueued, rfid = s_stats[TX_RC522_EVENT].enqueued;
    uint32_t gy = s_stats[TX_GY511].publish_ok, dht = s_stats[TX_DHT22].publish_ok;
    uint32_t deferrals = s_hcsr04p_deferred, successes = stats->publish_ok;
    pump();
    assert(stats->publish_ok == successes && s_hcsr04p.pending && s_hcsr04p_deferred == deferrals + 1);
    assert(s_stats[TX_CONFIG_ACK].enqueued == acks + 1 && s_stats[TX_RC522_EVENT].enqueued == rfid + 1);
    assert(s_stats[TX_GY511].publish_ok == gy + 1 && s_stats[TX_DHT22].publish_ok == dht + TX_QUEUE_SIZE - 2);

    // RFID outbox backpressure defers distance without stealing its ACK reserve.
    outbox_size = OUTBOX_LIMIT_BYTES;
    assert(mqtt_manager_publish_rc522_event(uid, sizeof(uid)) == ESP_OK);
    distance_due(); assert(mqtt_manager_update_hcsr04p_snapshot(&expected, clock_us) == ESP_OK);
    seq = stats->attempts; pump();
    assert(stats->attempts == seq && s_hcsr04p.pending && s_hcsr04p_deferred == deferrals + 2);
    outbox_size = 0; pump(); assert(!uxQueueMessagesWaiting(s_event_queue));
    // QoS0 distance uses no outbox capacity, even when the SDK outbox is full.
    outbox_size = OUTBOX_LIMIT_BYTES;
    distance_due(); acquired = clock_us;
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, acquired) == ESP_OK);
    enqueues = enqueue_calls; pump(); check_distance_json(&expected, acquired, seq + 1);
    assert(outbox_size == OUTBOX_LIMIT_BYTES && enqueue_calls == enqueues);
    outbox_size = 0;

    // Independent radar deadline and both mailboxes coexist in one publisher.
    distance_due(); radar_due(); acquired = clock_us;
    ld2450_report_t radar = radar_fixture(3);
    assert(mqtt_manager_update_ld2450_snapshot(&radar, acquired) == ESP_OK);
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, acquired) == ESP_OK);
    uint32_t radar_successes = s_stats[TX_LD2450].publish_ok;
    seq = stats->attempts + 1; pump(); check_distance_json(&expected, acquired, seq);
    assert(s_stats[TX_LD2450].publish_ok == radar_successes + 1);
    assert(!stats->queued && !stats->queue_full && !stats->enqueued && !stats->outbox_full);
    puts("PASS: HC-SR04P ACK/RFID priority, full queues/outbox, DHT22/GY511/LD2450 coexistence");

    // Reader continues at 200 ms during an actual manager SDK-call boundary.
    distance_due(); acquired = clock_us;
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, acquired) == ESP_OK);
    publish_hook = distance_reader_during_publish;
    seq = stats->attempts + 1; service_hcsr04p_snapshot();
    check_distance_json(&expected, acquired, seq);
    assert(stats->max_tx_call_us == 5020000 && s_hcsr04p.pending);
    acquired = s_hcsr04p.acquired_us; seq = stats->attempts + 1;
    calls = publish_calls; pump();
    assert(publish_calls == calls + 1); // Latest only; no five-second catch-up burst.
    check_distance_json(&distance_fixture, acquired, seq);

    // Timeout/invalid acquisitions create no snapshot. A prior success stays
    // eligible until 2 s, and keeps its acquisition time even while degraded.
    distance_due(); acquired = clock_us;
    assert(mqtt_manager_update_hcsr04p_snapshot(&expected, acquired) == ESP_OK);
    updates = s_hcsr04p_updates;
    run_distance_reader(400000, ESP_ERR_TIMEOUT);
    assert(distance_reads == 2 && s_hcsr04p_updates == updates && s_hcsr04p.acquired_us == acquired);
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_HCSR04P, &status) && status.state == SENSOR_DEGRADED);
    assert(status.read_failures == 2 && !status.read_successes);
    seq = stats->attempts + 1; pump(); check_distance_json(&expected, acquired, seq);
    calls = publish_calls; run_distance_reader(400000, ESP_ERR_INVALID_RESPONSE);
    assert(s_hcsr04p_updates == updates && !s_hcsr04p.pending);
    distance_due(); pump(); assert(publish_calls == calls);

    // Transport errors and offline drops leave the real sensor registry healthy.
    run_distance_reader(400000, ESP_OK);
    assert(sensor_status_get(SENSOR_HCSR04P, &status) && status.state == SENSOR_READY);
    uint32_t failed = stats->publish_failed;
    publish_result = -1; distance_due(); pump(); publish_result = 0;
    assert(stats->publish_failed == failed + 1 && !s_hcsr04p.pending);
    sensor_status_t after;
    assert(sensor_status_get(SENSOR_HCSR04P, &after));
    assert(after.state == status.state && after.read_successes == status.read_successes && !after.read_failures);
    event(MQTT_EVENT_DISCONNECTED); run_distance_reader(400000, ESP_OK);
    distance_due(); pump(); event(MQTT_EVENT_CONNECTED);
    assert(sensor_status_get(SENSOR_HCSR04P, &after) && after.state == SENSOR_READY && !after.read_failures);
    calls = publish_calls; distance_due(); pump(); assert(publish_calls == calls);
    distance_due(); assert(mqtt_manager_update_hcsr04p_snapshot(&expected, clock_us) == ESP_OK);
    disconnect_during_publish = true; pump(); disconnect_during_publish = false;
    assert(stats->publish_failed == failed + 2 && !s_hcsr04p.pending);
    event(MQTT_EVENT_CONNECTED);
    assert(!critical_depth && allocs == allocations);

    // Runtime configuration is exercised separately below in Phase 3 tests.
    mqtt_manager_log_diagnostics();
    assert(stats->attempts == stats->publish_ok + stats->publish_failed + stats->offline + stats->stale + stats->invalid);
    puts("PASS: real HC-SR04P reader during blocked SDK, timeout/invalid handoff, sensor/transport health isolation");
}

static void distance_gate_during_format(void)
{
    mqtt_manager_commit_hcsr04p_enabled(false);
    mqtt_manager_commit_hcsr04p_enabled(true);
    assert(mqtt_manager_update_hcsr04p_snapshot(&distance_fixture, clock_us) == ESP_OK);
}
static void distance_disable_in_sdk(void)
{
    publish_hook = NULL;
    mqtt_manager_commit_hcsr04p_enabled(false);
}

static void test_hcsr04p_config(void)
{
    int calls;
    distance_due();
    assert(mqtt_manager_update_hcsr04p_snapshot(&distance_fixture, clock_us) == ESP_OK);
    int64_t deadline = s_hcsr04p_next_publish_us;
    uint32_t epoch = s_hcsr04p_epoch;
    tx_stats_t *stats = &s_stats[TX_HCSR04P];
    uint32_t seq = stats->attempts;
    mqtt_manager_commit_hcsr04p_enabled(true);
    assert(s_hcsr04p.pending && s_hcsr04p_epoch == epoch && s_hcsr04p_next_publish_us == deadline);
    mqtt_manager_commit_hcsr04p_enabled(false);
    assert(!s_hcsr04p.pending && !s_hcsr04p_enabled && s_hcsr04p_epoch == epoch + 1);
    assert(mqtt_manager_update_hcsr04p_snapshot(&distance_fixture, clock_us) == ESP_ERR_INVALID_STATE);
    mqtt_manager_commit_hcsr04p_enabled(false);
    assert(s_hcsr04p_epoch == epoch + 1 && s_hcsr04p_next_publish_us == deadline);
    calls = publish_calls; clock_us += 10000000; pump();
    assert(publish_calls == calls && stats->attempts == seq);
    mqtt_manager_commit_hcsr04p_enabled(true);
    distance_due(); pump(); assert(publish_calls == calls); // No pre-disable replay.
    distance_due();
    assert(mqtt_manager_update_hcsr04p_snapshot(&distance_fixture, clock_us) == ESP_OK);
    uint32_t cancelled = stats->cancelled;
    timer_hook = distance_gate_during_format; timer_hook_countdown = 2;
    service_hcsr04p_snapshot();
    assert(!timer_hook && publish_calls == calls && stats->cancelled == cancelled + 1 && s_hcsr04p.pending);
    int64_t acquired = s_hcsr04p.acquired_us;
    seq = stats->attempts + 1; distance_due(); pump(); check_distance_json(&distance_fixture, acquired, seq);
    distance_due();
    assert(mqtt_manager_update_hcsr04p_snapshot(&distance_fixture, clock_us) == ESP_OK);
    calls = publish_calls; publish_hook = distance_disable_in_sdk; pump();
    assert(publish_calls == calls + 1 && !s_hcsr04p_enabled && !s_hcsr04p.pending);
    mqtt_manager_commit_hcsr04p_enabled(true);
    distance_due(); pump(); assert(publish_calls == calls + 1);
    // Slow acquisition cannot cause 1 Hz repeats or relax the 2 s freshness limit.
    for (unsigned i = 0; i < 3; ++i) {
        distance_due(); acquired = clock_us;
        assert(mqtt_manager_update_hcsr04p_snapshot(&distance_fixture, acquired) == ESP_OK);
        seq = stats->attempts + 1; pump(); check_distance_json(&distance_fixture, acquired, seq);
        calls = publish_calls;
        for (unsigned quiet = 0; quiet < 9; ++quiet) { distance_due(); pump(); }
        assert(publish_calls == calls);
    }
    assert(stats->attempts == stats->publish_ok + stats->publish_failed + stats->offline + stats->stale + stats->invalid + stats->cancelled);
    puts("PASS: HC-SR04P gate/epoch cancellation, idempotence, in-flight boundary, slow samples and fixed MQTT cadence");
}
