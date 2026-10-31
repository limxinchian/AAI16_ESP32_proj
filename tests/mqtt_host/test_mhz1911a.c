// Included by test_mqtt.c: real MQTT manager, CO2 reader and registry; driver,
// RTOS timing and transport are explicit doubles. No device/broker is contacted.
#include "../../main/mhz1911a_reader.c"

static jmp_buf co2_reader_idle;
static int64_t co2_until, co2_started_at, co2_last_start, co2_last_completed;
static unsigned co2_reads, co2_valid;
static esp_err_t co2_read_result;
static bool co2_pump_while_waiting;
static uint16_t co2_fixture = 1433;

// Decode a captured CO2 payload (binary aai16.Envelope, never JSON).
static aai16_Envelope decode_co2(const char *bytes, int length)
{
    aai16_Envelope e;
    assert(length > 0 && length < MHZ1911A_PAYLOAD_SIZE);
    assert(aai16_protocol_decode((const uint8_t *)bytes, (size_t)length, &e) == aai16_ResultCode_RESULT_OK);
    assert(e.has_protocol_version && e.protocol_version == AAI16_PROTOCOL_VERSION);
    assert(e.which_body == aai16_Envelope_sensor_telemetry_tag);
    assert(e.has_boot_id && e.has_sequence && e.has_generation && !e.has_request_id && !e.has_phase);
    const aai16_SensorTelemetry *t = &e.body.sensor_telemetry;
    assert(t->sensor == aai16_SensorId_SENSOR_MHZ1911A && t->which_reading == aai16_SensorTelemetry_mhz1911a_tag);
    assert(t->reading.mhz1911a.has_co2_ppm);
    return e;
}

// The message just handed to the SDK carries this CO2 value, acquisition time and sequence.
static void check_co2_message(uint16_t ppm, int64_t acquired_us, uint32_t seq)
{
    assert(!strcmp(last_topic, "aai16/sensors/mhz1911a/data") && last_qos == 0);
    aai16_Envelope e = decode_co2(last_payload, last_payload_len);
    const aai16_SensorTelemetry *t = &e.body.sensor_telemetry;
    assert(e.boot_id == 0x12345678 && e.sequence == seq);
    assert(e.generation == s_transactions.generation[SENSOR_MHZ1911A]);
    assert(t->acquired_us == (uint64_t)acquired_us);
    assert(t->uptime_us >= t->acquired_us && t->uptime_us <= (uint64_t)clock_us);
    assert(t->reading.mhz1911a.co2_ppm == ppm);
}

static void co2_delay(TickType_t ticks)
{
    assert(!critical_depth && ticks > 0 && ticks <= 60000);
    int64_t wake = clock_us + ticks * 1000LL;
    if (!co2_reads) {
        sensor_status_t status;
        assert(sensor_status_get(SENSOR_MHZ1911A, &status));
        assert(status.state == SENSOR_UNINITIALIZED && !status.read_successes);
    }
    if (co2_pump_while_waiting) pump(); // SDK work overlaps this reader's sleep.
    if (clock_us < wake) clock_us = wake;
    if (clock_us >= co2_until) longjmp(co2_reader_idle, 1);
}
esp_err_t mhz1911a_sensor_init(void) { return ESP_OK; }
esp_err_t mhz1911a_sensor_read(mhz1911a_reading_t *out)
{
    assert(clock_us >= co2_started_at + 60000000);
    if (co2_reads) assert(clock_us - co2_last_start == 5000000);
    co2_last_start = clock_us; ++co2_reads;
    clock_us += co2_read_result == ESP_ERR_TIMEOUT ? 2000000 : 30000;
    co2_last_completed = clock_us;
    if (co2_read_result == ESP_OK) { out->co2_ppm = co2_fixture; ++co2_valid; }
    return co2_read_result;
}
void mhz1911a_sensor_get_diagnostics(mhz1911a_diagnostics_t *out)
{
    *out = (mhz1911a_diagnostics_t){.requests=co2_reads, .valid_frames=co2_valid,
        .timeouts=co2_read_result == ESP_ERR_TIMEOUT ? co2_reads : 0,
        .checksum_failures=co2_read_result == ESP_ERR_INVALID_CRC ? co2_reads : 0};
}
static void run_co2_reader(int64_t duration, esp_err_t result, bool service_publisher)
{
    co2_started_at = clock_us; co2_until = clock_us + duration;
    co2_reads = co2_valid = 0; co2_read_result = result;
    co2_pump_while_waiting = service_publisher;
    // Independent reader scenarios, each with a fresh registry/warm-up. The
    // manager remains alive, preserving boot identity, counters and sequences.
    s_reader_started = false;
    mhz1911a_reader_start(); assert(co2_reader);
    reader_delay_hook = co2_delay;
    if (!setjmp(co2_reader_idle)) co2_reader(NULL);
    reader_delay_hook = NULL;
}

static void co2_age_during_format(void) { clock_us += MHZ1911A_MAX_AGE_US; }
static void co2_disconnect_during_format(void) { event(MQTT_EVENT_DISCONNECTED); }
static void co2_reader_during_publish(void)
{
    publish_hook = NULL;
    int calls = publish_calls, enqueues = enqueue_calls, allocations = allocs;
    uint32_t updates = s_mhz1911a_updates;
    // Model a stalled SDK API mutex while an independent real reader starts,
    // warms up and completes four five-second acquisitions. This is synthetic
    // latency, not a claimed ESP32 transport timeout or hardware measurement.
    run_co2_reader(80000000, ESP_OK, false);
    assert(co2_reads == 4 && s_mhz1911a_updates == updates + 4);
    assert(publish_calls == calls && enqueue_calls == enqueues && allocs == allocations);
    assert(!uxQueueMessagesWaiting(s_tx_queue) && !uxQueueMessagesWaiting(s_event_queue));
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_MHZ1911A, &status));
    assert(status.state == SENSOR_READY && status.read_successes == 4 && !status.read_failures);
}

static int64_t co2_stress_base;
static atomic_bool co2_stress_done;
static void *co2_stress_producer(void *arg)
{
    uintptr_t parity = (uintptr_t)arg;
    for (unsigned i = 1 + parity; i <= 20000; i += 2) {
        esp_err_t err = mqtt_manager_update_mhz1911a_snapshot(400 + i % 4601, co2_stress_base + i);
        assert(err == ESP_OK || err == ESP_ERR_INVALID_STATE);
    }
    return NULL;
}
static void *co2_stress_observer(void *arg)
{
    (void)arg;
    int64_t last = -1;
    do {
        portENTER_CRITICAL(&s_stats_lock);
        if (s_mhz1911a.pending) {
            assert(s_mhz1911a.acquired_us >= last);
            last = s_mhz1911a.acquired_us;
            unsigned index = (unsigned)(last - co2_stress_base);
            assert(s_mhz1911a.co2_ppm == 400 + index % 4601);
        }
        portEXIT_CRITICAL(&s_stats_lock);
    } while (!atomic_load(&co2_stress_done));
    return NULL;
}

static void test_mhz1911a_telemetry(void)
{
    event(MQTT_EVENT_CONNECTED); publish_result = 0;
    assert(!s_radar.pending && !s_hcsr04p.pending);
    tx_stats_t *stats = &s_stats[TX_MHZ1911A];
    int calls = publish_calls, enqueues = enqueue_calls, allocations = allocs;
    pump(); assert(publish_calls == calls && stats->attempts == 0);

    // Binary encoding: decode the exact bytes and test every buffer boundary.
    uint8_t wire[MHZ1911A_PAYLOAD_SIZE + 4];
    int len = encode_mhz1911a(wire, MHZ1911A_PAYLOAD_SIZE, 1433, 65000999, 65501999, 3, 1);
    assert(len > 0 && len < MHZ1911A_PAYLOAD_SIZE);
    aai16_Envelope decoded;
    assert(aai16_protocol_decode(wire, (size_t)len, &decoded) == aai16_ResultCode_RESULT_OK);
    assert(decoded.boot_id == 0x12345678 && decoded.sequence == 1 && decoded.generation == 3);
    assert(decoded.body.sensor_telemetry.acquired_us == 65000999);
    assert(decoded.body.sensor_telemetry.uptime_us == 65501999);
    assert(decoded.body.sensor_telemetry.reading.mhz1911a.co2_ppm == 1433);
    // Largest values: 64-bit times, 32-bit counters and the top of the CO2 range.
    len = encode_mhz1911a(wire, MHZ1911A_PAYLOAD_SIZE, 5000, INT64_MAX, INT64_MAX, UINT32_MAX, UINT32_MAX);
    assert(len > 0 && len < MHZ1911A_PAYLOAD_SIZE);
    const int worst = len;
    for (size_t size = 0; size < (size_t)len; ++size) {
        memset(wire, 0xa5, sizeof(wire));
        assert(encode_mhz1911a(wire, size, 5000, INT64_MAX, INT64_MAX, UINT32_MAX, UINT32_MAX) == -1);
        for (size_t i = size; i < sizeof(wire); ++i) assert(wire[i] == 0xa5);
    }
    memset(wire, 0xa5, sizeof(wire));
    assert(encode_mhz1911a(wire, (size_t)len, 5000, INT64_MAX, INT64_MAX, UINT32_MAX, UINT32_MAX) == len);
    for (size_t i = (size_t)len; i < sizeof(wire); ++i) assert(wire[i] == 0xa5);
    uint32_t boot = s_boot_id; s_boot_id = 0xabcdef;
    len = encode_mhz1911a(wire, MHZ1911A_PAYLOAD_SIZE, 400, 0, 0, 0, 0);
    assert(len > 0 && aai16_protocol_decode(wire, (size_t)len, &decoded) == aai16_ResultCode_RESULT_OK);
    assert(decoded.boot_id == 0xabcdef && decoded.body.sensor_telemetry.acquired_us == 0);
    s_boot_id = boot;
    assert(encode_mhz1911a(wire, MHZ1911A_PAYLOAD_SIZE, 1433, 10, 9, 0, 0) == -1); // uptime < acquired
    assert(encode_mhz1911a(wire, MHZ1911A_PAYLOAD_SIZE, 1433, -1, 9, 0, 0) == -1);
    assert(encode_mhz1911a(wire, MHZ1911A_PAYLOAD_SIZE, 399, 0, 0, 0, 0) == -1);  // codec bounds
    assert(encode_mhz1911a(wire, MHZ1911A_PAYLOAD_SIZE, 5001, 0, 0, 0, 0) == -1);
    printf("PASS: MH-Z1911A Protobuf encode/decode, boot/sequence/generation/timestamp, integer CO2, worst payload=%d/%d bytes and all buffer boundaries\n", worst, MHZ1911A_PAYLOAD_SIZE);

    int64_t acquired = clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired) == ESP_OK);
    assert(publish_calls == calls && enqueue_calls == enqueues && allocs == allocations);
    clock_us += 123000; pump(); check_co2_message(1433, acquired, 1);
    calls = publish_calls;
    for (unsigned i = 0; i < 3; ++i) { clock_us += 5000000; pump(); }
    assert(publish_calls == calls && stats->attempts == 1);
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired) == ESP_ERR_INVALID_STATE);
    assert(mqtt_manager_update_mhz1911a_snapshot(1434, acquired - 1) == ESP_ERR_INVALID_STATE);
    pump(); assert(publish_calls == calls && stats->attempts == 1 && s_mhz1911a_nonnew == 2);

    acquired = clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(400, acquired) == ESP_OK);
    uint32_t updates = s_mhz1911a_updates;
    assert(mqtt_manager_update_mhz1911a_snapshot(399, acquired) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_mhz1911a_snapshot(5001, acquired) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_mhz1911a_snapshot(UINT16_MAX, acquired) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, -1) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired + 1) == ESP_ERR_INVALID_ARG);
    assert(s_mhz1911a_invalid == 5 && s_mhz1911a_updates == updates && s_mhz1911a.co2_ppm == 400);
    pump(); check_co2_message(400, acquired, 2);
    acquired = clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(5000, acquired) == ESP_OK);
    pump(); check_co2_message(5000, acquired, 3);

    // A sample keeps the generation it was acquired under, even when a configuration
    // commit advances the sensor's generation before the sample is published.
    clock_us += 5000000; acquired = clock_us;
    uint32_t generation_before = s_transactions.generation[SENSOR_MHZ1911A];
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired) == ESP_OK);
    s_transactions.generation[SENSOR_MHZ1911A] = generation_before + 1;
    pump();
    aai16_Envelope held = decode_co2(last_payload, last_payload_len);
    assert(held.generation == generation_before && held.sequence == stats->attempts);
    s_transactions.generation[SENSOR_MHZ1911A] = generation_before;

    uint32_t seq = stats->attempts;
    clock_us += 5000000; acquired = clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired) == ESP_OK);
    clock_us += MHZ1911A_MAX_AGE_US - 1; pump(); check_co2_message(1433, acquired, ++seq);
    acquired = clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired) == ESP_OK);
    clock_us += MHZ1911A_MAX_AGE_US; calls = publish_calls; pump();
    assert(publish_calls == calls && stats->stale == 1 && stats->attempts == ++seq);
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, clock_us) == ESP_OK);
    timer_hook = co2_age_during_format; timer_hook_countdown = 2;
    service_mhz1911a_snapshot();
    assert(!timer_hook && stats->stale == 2 && stats->attempts == ++seq && publish_calls == calls);
    puts("PASS: MH-Z1911A immutable timestamp, duplicates/out-of-order/invalid inputs, exact freshness boundary and admission recheck");

    event(MQTT_EVENT_DISCONNECTED);
    for (unsigned i = 0; i < 3; ++i) {
        clock_us += 5000000;
        assert(mqtt_manager_update_mhz1911a_snapshot(1433, clock_us) == ESP_OK);
        pump(); assert(!s_mhz1911a.pending);
    }
    assert(stats->offline == 3 && publish_calls == calls);
    event(MQTT_EVENT_CONNECTED); pump(); assert(publish_calls == calls);
    // An unconsumed fresh sample can survive a short outage, never a history.
    event(MQTT_EVENT_DISCONNECTED); clock_us += 5000000; acquired = clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired) == ESP_OK);
    clock_us += 100000; event(MQTT_EVENT_CONNECTED);
    seq = stats->attempts + 1; pump(); check_co2_message(1433, acquired, seq);
    event(MQTT_EVENT_DISCONNECTED); acquired = clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired) == ESP_OK);
    clock_us += 5000000; event(MQTT_EVENT_CONNECTED); calls = publish_calls;
    pump(); assert(publish_calls == calls && stats->stale == 3);
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, clock_us) == ESP_OK);
    timer_hook = co2_disconnect_during_format; timer_hook_countdown = 2;
    service_mhz1911a_snapshot();
    assert(!timer_hook && publish_calls == calls && stats->offline == 4);
    event(MQTT_EVENT_CONNECTED); clock_us += 5000000;
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, clock_us) == ESP_OK);
    publish_result = -1; pump(); publish_result = 0;
    assert(stats->publish_failed == 1 && !s_mhz1911a.pending);
    calls = publish_calls; pump(); assert(publish_calls == calls);
    clock_us += 5000000;
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, clock_us) == ESP_OK);
    disconnect_during_publish = true; pump(); disconnect_during_publish = false;
    assert(stats->publish_failed == 2 && !s_mhz1911a.pending);
    event(MQTT_EVENT_CONNECTED); calls = publish_calls; pump(); assert(publish_calls == calls);
    puts("PASS: MH-Z1911A offline/reconnect, pending freshness, failed SDK submission and no retry/replay");

    // One mailbox remains bounded even with full queues/outbox; priorities and
    // the five existing telemetry/event paths retain their policies.
    outbox_size = OUTBOX_LIMIT_BYTES;
    assert(queue_test_ack() == ESP_OK);
    assert(mqtt_manager_publish_gy511(1,0,0,10,20,30) == ESP_OK);
    for (unsigned i = 2; i < TX_QUEUE_SIZE; ++i) assert(mqtt_manager_publish_dht22(25,50) == ESP_OK);
    seq = stats->attempts;
    for (unsigned i = 0; i < 1000; ++i) {
        clock_us += 1000;
        assert(mqtt_manager_update_mhz1911a_snapshot(1400 + i % 100, clock_us) == ESP_OK);
        service_mhz1911a_snapshot();
    }
    assert(s_mhz1911a_coalesced == 999 && stats->attempts == seq && allocs == allocations);
    assert(s_mhz1911a_deferred >= 1000 && uxQueueMessagesWaiting(s_tx_queue) == TX_QUEUE_SIZE);
    acquired = clock_us; pump(); check_co2_message(1499, acquired, seq + 1);
    const uint8_t uid[] = {1,2,3,4};
    assert(mqtt_manager_publish_rc522_event(uid, sizeof(uid)) == ESP_OK);
    acquired = clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired) == ESP_OK);
    calls = publish_calls; service_mhz1911a_snapshot(); assert(publish_calls == calls && s_mhz1911a.pending);
    outbox_size = 0; service_event_queue(); assert(!uxQueueMessagesWaiting(s_event_queue));
    ld2450_report_t radar = {0};
    assert(mqtt_manager_update_ld2450_snapshot(&radar, clock_us) == ESP_OK);
    assert(mqtt_manager_update_hcsr04p_snapshot(&distance_fixture, clock_us) == ESP_OK);
    // Make both existing mailboxes due without changing the production policies.
    s_radar_next_publish_us = s_hcsr04p_next_publish_us = clock_us;
    uint32_t radar_ok = s_stats[TX_LD2450].publish_ok, distance_ok = s_stats[TX_HCSR04P].publish_ok;
    seq = stats->attempts + 1; pump(); check_co2_message(1433, acquired, seq);
    assert(s_stats[TX_LD2450].publish_ok == radar_ok + 1 && s_stats[TX_HCSR04P].publish_ok == distance_ok + 1);
    assert(!stats->queued && !stats->enqueued && !stats->queue_full && !stats->outbox_full);
    puts("PASS: MH-Z1911A bounded replacement, ACK/RFID priority and all five existing telemetry paths");

    calls = publish_calls; updates = s_mhz1911a_updates;
    run_co2_reader(59999000, ESP_OK, true);
    assert(!co2_reads && s_mhz1911a_updates == updates && publish_calls == calls);
    run_co2_reader(125000000, ESP_OK, true);
    assert(co2_reads == 13 && co2_valid == 13 && s_mhz1911a_updates == updates + 13 && publish_calls == calls + 13);
    check_co2_message(co2_fixture, co2_last_completed, stats->attempts);
    calls = publish_calls; updates = s_mhz1911a_updates;
    run_co2_reader(75000000, ESP_ERR_TIMEOUT, true);
    assert(co2_reads == 3 && !co2_valid && s_mhz1911a_updates == updates && publish_calls == calls);
    run_co2_reader(75000000, ESP_ERR_INVALID_CRC, true);
    assert(co2_reads == 3 && !co2_valid && s_mhz1911a_updates == updates && publish_calls == calls);
    publish_result = -1; run_co2_reader(75000000, ESP_OK, true); publish_result = 0;
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_MHZ1911A, &status) && status.state == SENSOR_READY && status.read_successes == 3 && !status.read_failures);
    event(MQTT_EVENT_DISCONNECTED); calls = publish_calls;
    run_co2_reader(75000000, ESP_OK, true); event(MQTT_EVENT_CONNECTED);
    assert(co2_reads == 3 && publish_calls == calls && !s_mhz1911a.pending);
    assert(sensor_status_get(SENSOR_MHZ1911A, &status) && status.state == SENSOR_READY && !status.read_failures);
    puts("PASS: real MH-Z1911A reader warm-up, 5000 ms acquisitions, success-only handoff and MQTT/health isolation");

    acquired = clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(1430, acquired) == ESP_OK);
    publish_hook = co2_reader_during_publish;
    seq = stats->attempts + 1; service_mhz1911a_snapshot();
    check_co2_message(1430, acquired, seq); // SDK payload remains its own immutable copy.
    assert(stats->max_tx_call_us >= 80000000 && s_mhz1911a.pending);
    calls = publish_calls; uint32_t stale = stats->stale; pump();
    assert(publish_calls == calls && stats->stale == stale + 1); // Last sample aged during long stall.
    acquired = ++clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(1431, acquired) == ESP_OK);
    seq = stats->attempts + 1; pump(); check_co2_message(1431, acquired, seq);
    puts("PASS: independent real CO2 reader during blocked SDK call, bounded mailbox and immutable in-flight payload");

    // Real parallel producers and an observer exercise the actual mailbox copy
    // under a host mutex implementing the production critical-section contract.
    assert(!s_mhz1911a.pending && !timer_hook);
    co2_stress_base = clock_us + 1; clock_us = co2_stress_base + 20001;
    pthread_t first, second, observer;
    assert(!pthread_create(&observer, NULL, co2_stress_observer, NULL));
    assert(!pthread_create(&first, NULL, co2_stress_producer, (void *)(uintptr_t)0));
    assert(!pthread_create(&second, NULL, co2_stress_producer, (void *)(uintptr_t)1));
    assert(!pthread_join(first, NULL) && !pthread_join(second, NULL));
    atomic_store(&co2_stress_done, true); assert(!pthread_join(observer, NULL));
    acquired = co2_stress_base + 20000;
    assert(s_mhz1911a.acquired_us == acquired && allocs == allocations);
    seq = stats->attempts + 1; pump(); check_co2_message(400 + 20000 % 4601, acquired, seq);
    assert(stats->attempts == stats->publish_ok + stats->publish_failed + stats->offline + stats->stale + stats->invalid);
    puts("PASS: 20000 parallel CO2 updates, coherent ppm/timestamp copies and bounded allocation-free handoff");

    *stats = (tx_stats_t){.attempts=UINT32_MAX, .publish_ok=UINT32_MAX};
    acquired = ++clock_us;
    assert(mqtt_manager_update_mhz1911a_snapshot(1433, acquired) == ESP_OK);
    pump(); check_co2_message(1433, acquired, 0);
    assert(stats->attempts == 0 && stats->publish_ok == 0);
    assert(!critical_depth && allocs == allocations);
    mqtt_manager_log_diagnostics();
    puts("PASS: MH-Z1911A sequence wrap");
}
