// Included by test_mqtt.c to exercise the real manager with its existing SDK
// doubles. No production fault injection and no network or serial access.
static ld2450_report_t radar_fixture(unsigned mask)
{
    ld2450_report_t r = {0};
    const ld2450_target_t targets[] = {
        {true, -782, 1713, -16, 320},
        {true, 3000, -4000, 12, 640},
        {true, -32768, -32768, -32768, 65535},
    };
    for (unsigned i = 0; i < LD2450_TARGETS; ++i) {
        if (mask & (1U << i)) { r.targets[i] = targets[i]; r.count++; }
    }
    return r;
}

static void radar_due(void)
{
    if (clock_us < s_radar_next_publish_us) clock_us = s_radar_next_publish_us;
}

static void check_radar_json(const ld2450_report_t *r, int64_t received_us, uint32_t seq)
{
    assert(!strcmp(last_topic, LD2450_DATA_TOPIC) && last_qos == 0);
    cJSON *data = cJSON_Parse(last_payload);
    assert(data && cJSON_GetArraySize(data) == 6);
    assert(!strcmp(cJSON_GetObjectItem(data, "sensor")->valuestring, "ld2450"));
    assert(!strcmp(cJSON_GetObjectItem(data, "boot_id")->valuestring, "12345678"));
    assert(cJSON_GetObjectItem(data, "uptime_ms")->valuedouble == received_us / 1000);
    assert(cJSON_GetObjectItem(data, "seq")->valuedouble == seq);
    assert(cJSON_GetObjectItem(data, "target_count")->valueint == (int)r->count);
    cJSON *targets = cJSON_GetObjectItem(data, "targets");
    assert(cJSON_IsArray(targets) && cJSON_GetArraySize(targets) == (int)r->count);
    unsigned index = 0;
    const unsigned distances[] = {1883, 5000, 46341};
    for (unsigned i = 0; i < LD2450_TARGETS; ++i) {
        const ld2450_target_t *t = &r->targets[i];
        if (!t->present) continue;
        cJSON *target = cJSON_GetArrayItem(targets, index++);
        assert(target && cJSON_GetArraySize(target) == 6);
        assert(cJSON_GetObjectItem(target, "slot")->valueint == (int)i + 1);
        assert(cJSON_GetObjectItem(target, "x_mm")->valueint == t->x_mm);
        assert(cJSON_GetObjectItem(target, "y_mm")->valueint == t->y_mm);
        assert(cJSON_GetObjectItem(target, "speed_cm_s")->valueint == t->speed_cm_s);
        assert(cJSON_GetObjectItem(target, "resolution_mm")->valueint == t->resolution_mm);
        assert(cJSON_GetObjectItem(target, "distance_mm")->valuedouble == distances[i]);
    }
    cJSON_Delete(data);
}

static int64_t during_publish_received_us;
static void radar_during_publish(void)
{
    // Simulate the UART owner running throughout a five-second SDK stall. All
    // 50 valid batches must be accepted, with no SDK re-entry or queue growth.
    publish_hook = NULL;
    int calls = publish_calls, enqueues = enqueue_calls, allocations = allocs;
    for (unsigned i = 0; i < 50; ++i) {
        clock_us += 100000;
        ld2450_report_t r = radar_fixture(i == 49 ? 4 : 2);
        assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
    }
    during_publish_received_us = clock_us;
    assert(publish_calls == calls && enqueue_calls == enqueues && allocs == allocations);
    assert(!uxQueueMessagesWaiting(s_tx_queue) && !uxQueueMessagesWaiting(s_event_queue));
}

static void test_radar_telemetry(void)
{
    tx_stats_t *stats = &s_stats[TX_LD2450];
    int calls = publish_calls;
    event(MQTT_EVENT_CONNECTED);
    publish_result = 0;
    radar_due(); pump();
    assert(publish_calls == calls && !stats->attempts); // Never initialized data.

    // Every slot combination, including valid no-target data and sparse slots.
    for (unsigned mask = 0; mask < 8; ++mask) {
        radar_due();
        ld2450_report_t expected = radar_fixture(mask), local = expected;
        int64_t acquired = clock_us;
        uint32_t seq = stats->attempts + 1;
        calls = publish_calls;
        int enqueues = enqueue_calls;
        assert(mqtt_manager_update_ld2450_snapshot(&local, acquired) == ESP_OK);
        memset(&local, 0, sizeof(local)); // Mailbox owns a copy, not a pointer.
        assert(publish_calls == calls && enqueue_calls == enqueues);
        assert(!uxQueueMessagesWaiting(s_tx_queue));
        pump();
        assert(publish_calls == calls + 1 && enqueue_calls == enqueues);
        check_radar_json(&expected, acquired, seq);
        radar_due(); pump(); // No new frame: never repeat the consumed snapshot.
        assert(publish_calls == calls + 1);
    }

    // Largest storage-type values, 10-digit seq and full 64-bit timestamp.
    // Existing queue items remain 320 bytes; the radar JSON never enters them.
    ld2450_report_t worst = {.count = 3};
    for (unsigned i = 0; i < 3; ++i) worst.targets[i] = (ld2450_target_t){true, INT16_MIN, INT16_MIN, INT16_MIN, UINT16_MAX};
    struct { char payload[LD2450_PAYLOAD_SIZE]; char guard[4]; } bounded = {.guard = "END"};
    int length = format_radar(bounded.payload, sizeof(bounded.payload), &worst, INT64_MAX, UINT32_MAX);
    assert(length > TX_PAYLOAD_SIZE && length < LD2450_PAYLOAD_SIZE && !strcmp(bounded.guard, "END"));
    cJSON *data = cJSON_Parse(bounded.payload);
    assert(data && cJSON_GetArraySize(cJSON_GetObjectItem(data, "targets")) == 3);
    cJSON_Delete(data);
    assert(format_radar(bounded.payload, 20, &worst, INT64_MAX, UINT32_MAX) == -1);
    assert(!strcmp(bounded.guard, "END"));
    printf("PASS: radar JSON 0/1/2/3 targets, signed units/slots, worst payload=%d/512 bytes\n", length);

    // 10 Hz input is coalesced to <=1 Hz publication, with a fresh timestamp.
    radar_due();
    ld2450_report_t expected = radar_fixture(5);
    calls = publish_calls;
    uint32_t coalesced = s_radar_coalesced;
    for (unsigned i = 0; i < 10; ++i) {
        ld2450_report_t r = radar_fixture(i == 9 ? 5 : 1);
        assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
        if (i < 9) clock_us += 100000;
    }
    int64_t acquired = clock_us;
    uint32_t seq = stats->attempts + 1;
    assert(s_radar_coalesced == coalesced + 9);
    pump();
    assert(publish_calls == calls + 1);
    check_radar_json(&expected, acquired, seq);
    assert(mqtt_manager_update_ld2450_snapshot(&expected, clock_us) == ESP_OK);
    clock_us = s_radar_next_publish_us - 1; pump();
    assert(publish_calls == calls + 1); // No early send, even with fresh frames.
    radar_due(); pump(); assert(publish_calls == calls + 2);

    // Reject malformed handoffs without replacing the previous valid report.
    radar_due(); acquired = clock_us;
    assert(mqtt_manager_update_ld2450_snapshot(&expected, acquired) == ESP_OK);
    ld2450_report_t bad = expected; bad.count = 4;
    assert(mqtt_manager_update_ld2450_snapshot(&bad, acquired) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_ld2450_snapshot(NULL, acquired) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_ld2450_snapshot(&expected, -1) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_update_ld2450_snapshot(&expected, acquired + 1) == ESP_ERR_INVALID_ARG);
    assert(s_radar_invalid == 4);
    seq = stats->attempts + 1; pump(); check_radar_json(&expected, acquired, seq);

    // Exact two-second boundary: silence does not turn old data into a new read.
    radar_due();
    assert(mqtt_manager_update_ld2450_snapshot(&expected, clock_us) == ESP_OK);
    clock_us += LD2450_MAX_AGE_US;
    calls = publish_calls; pump();
    assert(publish_calls == calls && stats->stale == 1);
    radar_due(); pump(); assert(stats->stale == 1);

    // Offline acquisition stays bounded. Reconnect sends only a recent latest
    // report, never a backlog, and never an expired pre-disconnect report.
    event(MQTT_EVENT_DISCONNECTED);
    for (unsigned period = 0; period < 4; ++period) {
        radar_due();
        for (unsigned i = 0; i < 100; ++i)
            assert(mqtt_manager_update_ld2450_snapshot(&expected, clock_us) == ESP_OK);
        pump();
    }
    assert(stats->offline == 4 && publish_calls == calls);
    assert(!uxQueueMessagesWaiting(s_tx_queue) && !uxQueueMessagesWaiting(s_event_queue));
    radar_due(); acquired = clock_us;
    assert(mqtt_manager_update_ld2450_snapshot(&expected, acquired) == ESP_OK);
    event(MQTT_EVENT_CONNECTED);
    seq = stats->attempts + 1; pump(); check_radar_json(&expected, acquired, seq);
    event(MQTT_EVENT_DISCONNECTED); radar_due();
    assert(mqtt_manager_update_ld2450_snapshot(&expected, clock_us) == ESP_OK);
    clock_us += LD2450_MAX_AGE_US;
    event(MQTT_EVENT_CONNECTED); calls = publish_calls; pump();
    assert(publish_calls == calls && stats->stale == 2);

    // Full shared queue: radar neither takes a slot nor drops ACKs/events.
    radar_due();
    assert(queue_test_ack() == ESP_OK);
    for (unsigned i = 1; i < TX_QUEUE_SIZE; ++i) assert(mqtt_manager_publish_dht22(25, 50) == ESP_OK);
    assert(mqtt_manager_update_ld2450_snapshot(&expected, clock_us) == ESP_OK);
    uint32_t deferrals = s_radar_deferred, radar_ok = stats->publish_ok;
    uint32_t acks = s_stats[TX_CONFIG_ACK].enqueued, rfid = s_stats[TX_RC522_EVENT].enqueued;
    const uint8_t uid[] = {1, 2, 3, 4};
    assert(mqtt_manager_publish_rc522_event(uid, 4) == ESP_OK);
    pump();
    assert(s_radar_deferred == deferrals + 1 && stats->publish_ok == radar_ok && s_radar.pending);
    assert(s_stats[TX_CONFIG_ACK].enqueued == acks + 1 && s_stats[TX_RC522_EVENT].enqueued == rfid + 1);
    assert(!uxQueueMessagesWaiting(s_tx_queue) && !uxQueueMessagesWaiting(s_event_queue));
    radar_due(); acquired = clock_us;
    assert(mqtt_manager_update_ld2450_snapshot(&expected, acquired) == ESP_OK);
    seq = stats->attempts + 1; pump(); check_radar_json(&expected, acquired, seq);
    assert(stats->queued == 0 && stats->queue_full == 0 && stats->enqueued == 0);

    // All ACK outbox capacity remains usable; QoS 0 radar creates no entry.
    outbox_size = OUTBOX_LIMIT_BYTES;
    radar_due(); assert(mqtt_manager_update_ld2450_snapshot(&expected, clock_us) == ESP_OK);
    int enqueues = enqueue_calls;
    pump(); assert(outbox_size == OUTBOX_LIMIT_BYTES && enqueue_calls == enqueues);
    outbox_size = 0;

    // A stalled publication cannot retain the snapshot spinlock or its pointer.
    radar_due(); acquired = clock_us;
    assert(mqtt_manager_update_ld2450_snapshot(&expected, acquired) == ESP_OK);
    publish_hook = radar_during_publish;
    seq = stats->attempts + 1; service_radar_snapshot();
    check_radar_json(&expected, acquired, seq);
    assert(s_radar.pending && stats->max_tx_call_us == 5020000);
    expected = radar_fixture(4); seq = stats->attempts + 1;
    calls = publish_calls; pump();
    assert(publish_calls == calls + 1); // One latest sample, no catch-up burst.
    check_radar_json(&expected, during_publish_received_us, seq);

    // Transport errors are MQTT counters only, and failed samples are not retried.
    radar_due(); assert(mqtt_manager_update_ld2450_snapshot(&expected, clock_us) == ESP_OK);
    publish_result = -1; pump();
    assert(stats->publish_failed == 1 && !s_radar.pending);
    publish_result = 0; calls = publish_calls; radar_due(); pump(); assert(publish_calls == calls);
    radar_due(); assert(mqtt_manager_update_ld2450_snapshot(&expected, clock_us) == ESP_OK);
    disconnect_during_publish = true; pump(); disconnect_during_publish = false;
    assert(stats->publish_failed == 2 && !s_radar.pending);
    event(MQTT_EVENT_CONNECTED);
    assert(stats->attempts == stats->publish_ok + stats->publish_failed + stats->offline + stats->stale);
    assert(!stats->outbox_full && !stats->invalid && !critical_depth);
    puts("PASS: radar latest snapshot, 1 Hz, stale/invalid/offline/full queue, ACK/RFID isolation, blocked SDK, QoS0 counters");
}

static void radar_interval(uint32_t ms)
{
    int calls = publish_calls, enqueues = enqueue_calls;
    assert(mqtt_manager_commit_ld2450_config(&(mqtt_config_update_t){
        .sensor=SENSOR_LD2450, .operation=MQTT_CONFIG_INTERVAL, .interval_ms=ms}) == ESP_OK);
    assert(publish_calls == calls && enqueue_calls == enqueues);
}

static void radar_gate(bool enabled)
{
    int calls = publish_calls, enqueues = enqueue_calls;
    assert(mqtt_manager_commit_ld2450_config(&(mqtt_config_update_t){
        .sensor=SENSOR_LD2450, .operation=MQTT_CONFIG_ENABLED, .enabled=enabled}) == ESP_OK);
    assert(publish_calls == calls && enqueue_calls == enqueues);
}

static void radar_gate_during_format(void)
{
    radar_gate(false); radar_gate(true);
    ld2450_report_t fresh_idle = {0};
    assert(mqtt_manager_update_ld2450_snapshot(&fresh_idle, clock_us) == ESP_OK);
}
static void radar_interval_during_format(void) { radar_interval(10000); }
static void radar_age_during_format(void) { clock_us += LD2450_MAX_AGE_US; }
static void radar_gate_during_sdk(void) { publish_hook = NULL; radar_gate(false); }

static void test_radar_config(void)
{
    assert(s_radar_enabled && s_radar_interval_ms == 1000);
    assert(mqtt_manager_commit_ld2450_config(NULL) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_commit_ld2450_config(&(mqtt_config_update_t){.sensor=SENSOR_RC522, .interval_ms=500}) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_commit_ld2450_config(&(mqtt_config_update_t){.sensor=SENSOR_LD2450, .interval_ms=499}) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_commit_ld2450_config(&(mqtt_config_update_t){.sensor=SENSOR_LD2450, .interval_ms=10001}) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_commit_ld2450_config(&(mqtt_config_update_t){.sensor=SENSOR_LD2450, .operation=99}) == ESP_ERR_INVALID_ARG);

    ld2450_report_t r = radar_fixture(1);
    tx_stats_t *stats = &s_stats[TX_LD2450];
    radar_interval(500);
    assert(s_radar_next_publish_us == clock_us + 500000 && publisher_wait_ticks() == 500);
    assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
    int calls = publish_calls;
    clock_us += 499000; pump();
    assert(publish_calls == calls && last_worker_wait == 1);
    clock_us += 1000; pump(); assert(publish_calls == calls + 1 && last_worker_wait <= 500);
    for (unsigned i = 0; i < 3; ++i) {
        radar_due();
        int64_t due = clock_us;
        assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
        pump();
        assert(s_radar_next_publish_us == due + 500000 && publish_calls == calls + 2 + (int)i);
    }
    radar_interval(501); assert(publisher_wait_ticks() == 501);
    radar_interval(10000); assert(publisher_wait_ticks() == 1000);
    assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
    uint32_t stale = stats->stale;
    radar_due(); calls = publish_calls; pump();
    assert(publish_calls == calls && stats->stale == stale + 1); // Long period cannot relax freshness.
    clock_us = s_radar_next_publish_us - 100000;
    int64_t acquired = clock_us;
    assert(mqtt_manager_update_ld2450_snapshot(&r, acquired) == ESP_OK);
    radar_due(); uint32_t seq = stats->attempts + 1; pump(); check_radar_json(&r, acquired, seq);

    radar_interval(2000);
    assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
    radar_gate(false);
    assert(!s_radar.pending && !s_radar_enabled && publisher_wait_ticks() == 1000);
    assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_ERR_INVALID_STATE);
    assert(mqtt_manager_commit_ld2450_config(&(mqtt_config_update_t){.sensor=SENSOR_LD2450, .interval_ms=500}) == ESP_ERR_INVALID_STATE);
    calls = publish_calls; clock_us += 10000000; pump(); assert(publish_calls == calls);
    radar_gate(false); assert(s_radar_interval_ms == 2000);
    radar_gate(true);
    assert(!s_radar.pending && s_radar_next_publish_us == clock_us + 2000000);
    int64_t deadline = s_radar_next_publish_us;
    clock_us += 500000; radar_gate(true); assert(s_radar_next_publish_us == deadline);
    radar_due(); pump(); assert(publish_calls == calls); // No pre-enable snapshot replay.
    ld2450_report_t idle = {0};
    clock_us = s_radar_next_publish_us - 100000; acquired = clock_us;
    assert(mqtt_manager_update_ld2450_snapshot(&idle, acquired) == ESP_OK);
    radar_due(); seq = stats->attempts + 1; pump(); check_radar_json(&idle, acquired, seq);

    // Disable/re-enable between copy and SDK admission invalidates even a
    // copied report. The new generation's snapshot must remain pending.
    radar_due(); assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
    timer_hook = radar_gate_during_format; timer_hook_countdown = 2;
    calls = publish_calls; uint32_t cancelled = stats->cancelled;
    service_radar_snapshot();
    assert(!timer_hook && publish_calls == calls && stats->cancelled == cancelled + 1 && s_radar.pending);
    // Advance with new frames so the retained 2 s period has fresh idle data.
    clock_us = s_radar_next_publish_us - 100000; acquired = clock_us;
    assert(mqtt_manager_update_ld2450_snapshot(&idle, acquired) == ESP_OK);
    radar_due(); seq = stats->attempts + 1; pump(); check_radar_json(&idle, acquired, seq);
    radar_due(); assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
    timer_hook = radar_interval_during_format; timer_hook_countdown = 2;
    calls = publish_calls; service_radar_snapshot();
    assert(!timer_hook && publish_calls == calls && stats->cancelled == cancelled + 2);
    assert(s_radar_interval_ms == 10000 && s_radar_next_publish_us == clock_us + 10000000);
    radar_due(); assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
    timer_hook = radar_age_during_format; timer_hook_countdown = 2;
    stale = stats->stale; service_radar_snapshot();
    assert(!timer_hook && publish_calls == calls && stats->stale == stale + 1);

    // An already admitted send may complete after disable. No lock is held
    // during SDK calls and the control owner need not wait for that transport.
    radar_due(); assert(mqtt_manager_update_ld2450_snapshot(&r, clock_us) == ESP_OK);
    publish_hook = radar_gate_during_sdk; service_radar_snapshot();
    assert(!s_radar_enabled && !s_radar.pending && publish_calls == calls + 1);
    clock_us += 20000000; pump(); assert(publish_calls == calls + 1);
    radar_gate(true); radar_interval(1000);
    assert(stats->attempts == stats->publish_ok + stats->publish_failed + stats->offline + stats->stale + stats->cancelled);
    puts("PASS: radar publication 500..10000 ms, bounded worker wait, pause/resume, freshness and in-flight control races");
}
