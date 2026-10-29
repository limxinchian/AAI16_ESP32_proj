// Real production MQTT adapter + nanopb codec, with the SDK payload captured.
static motion_result_t motion_fixture(bool model_b)
{
    motion_result_t r = {
        .model_identity = {.family = model_b ? 2 : 1, .source = model_b ? 2 : 1, .bank = 0},
        .model_epoch = model_b ? 7 : 0, .status = MOTION_PREDICTION,
        .model_ready = true, .class_id = model_b ? 0 : 1,
        .output_int8 = {-128, 0}, .logits = {-1.25f, 2.5f},
        .boot_id = mqtt_manager_boot_id(), .first_sequence = UINT32_MAX - 7, .last_sequence = 11,
        .generation = 2, .interval_ms = 100, .uptime_us = clock_us,
        .window_start_us = clock_us - 1900000, .window_end_us = clock_us,
        .inference_us = 123, .clipped = 3, .clipped_total = 27,
        .queue_drops = 4, .window_resets = 5, .invoke_failures = 6, .arena_used_bytes = 12000,
    };
    strcpy(r.model_identity.version, model_b ? "aai16-motion-model-b-0.1.0" : "aai16-motion-model-a-0.1.0");
    // Deliberately different legacy display string: wire identity comes only
    // from the immutable identity captured with this result, not this string.
    strcpy(r.model_version, "legacy-display-must-not-be-serialized");
    for (unsigned i = 0; i < 32; ++i) r.model_identity.sha256[i] = i;
    return r;
}

static aai16_Envelope check_motion_publish(const motion_result_t *r)
{
    int before = publish_calls, allocation_count = allocs;
    assert(mqtt_manager_publish_motion(r) == ESP_OK);
    assert(publish_calls == before && allocs == allocation_count); // Only a queue copy on producer.
    pump();
    assert(publish_calls == before + 1); // Exactly one MQTT message, no JSON/side-topic duplicate.
    assert(!strcmp(last_topic, MOTION_DATA_TOPIC) && last_qos == 0);
    assert(last_payload_len > 0 && last_payload_len < TX_PAYLOAD_SIZE);
    assert((uint8_t)last_payload[0] == 0x08 && (uint8_t)last_payload[1] == 1);
    assert(memchr(last_payload, 0, last_payload_len) != NULL);
    assert(strlen(last_payload) < (size_t)last_payload_len); // strlen would truncate the real frame.
    assert(cJSON_Parse(last_payload) == NULL);
    aai16_Envelope e;
    assert(aai16_protocol_decode((uint8_t *)last_payload, last_payload_len, &e) == CR(OK));
    assert(e.which_body == aai16_Envelope_ml_prediction_tag);
    assert(e.boot_id == r->boot_id && e.generation == r->generation);
    assert(!e.has_request_id && !e.has_phase && !e.has_sequence);
    const aai16_MlPrediction *p = &e.body.ml_prediction;
    assert(p->model_epoch == r->model_epoch && p->status == (aai16_MotionStatus)(r->status + 1));
    assert(p->model_ready == r->model_ready && p->interval_ms == r->interval_ms && p->uptime_us == (uint64_t)r->uptime_us);
    assert(p->model.family == r->model_identity.family && p->model.source == r->model_identity.source);
    assert(p->model.has_bank == (r->model_identity.source == 2));
    assert(!p->model.has_bank || p->model.bank == r->model_identity.bank);
    assert(!strcmp(p->model.version, r->model_identity.version));
    assert(p->model.sha256.size == 32 && !memcmp(p->model.sha256.bytes, r->model_identity.sha256, 32));
    assert(p->has_clipped_total && p->clipped_total == r->clipped_total);
    assert(p->has_queue_drops && p->queue_drops == r->queue_drops);
    assert(p->has_window_resets && p->window_resets == r->window_resets);
    assert(p->has_invoke_failures && p->invoke_failures == r->invoke_failures);
    assert(p->arena_used_bytes == r->arena_used_bytes && p->arena_reserved_bytes == 16384);
    assert(p->has_backend && p->backend == TEST_EXPECTED_BACKEND);
    assert(p->backend == motion_inference_backend() && p->has_trial && !p->trial);
    if (r->status == MOTION_PREDICTION) {
        assert(p->classification == (r->class_id ? aai16_MotionClass_CLASS_MOVING : aai16_MotionClass_CLASS_REST));
        assert(p->has_first_sequence && p->first_sequence == r->first_sequence);
        assert(p->has_last_sequence && p->last_sequence == r->last_sequence);
        assert(p->window_start_us == (uint64_t)r->window_start_us && p->window_end_us == (uint64_t)r->window_end_us);
        assert(p->has_inference_us && p->inference_us == (uint64_t)r->inference_us);
        assert(p->has_clipped && p->clipped == r->clipped);
        assert(p->output_int8.size == 2 && !memcmp(p->output_int8.bytes, r->output_int8, 2));
        assert(p->logits_count == 2 && !memcmp(p->logits, r->logits, sizeof(r->logits)));
    } else {
        assert(p->classification == aai16_MotionClass_CLASS_UNKNOWN);
        assert(!p->has_output_int8 && !p->logits_count && !p->has_inference_us && !p->has_clipped);
        assert(!p->has_first_sequence && !p->has_last_sequence && !p->has_window_start_us && !p->has_window_end_us);
    }
    return e;
}

static void test_motion_telemetry(void)
{
    if (clock_us < 2000000) clock_us = 5000123;
    motion_result_t r = motion_fixture(false);
    check_motion_publish(&r);
    r = motion_fixture(true);
    aai16_Envelope e = check_motion_publish(&r);
    for (int status = MOTION_COLLECTING; status <= MOTION_SENSOR_UNAVAILABLE; ++status) {
        if (status == MOTION_PREDICTION) continue;
        r = motion_fixture(true); r.status = status;
        r.model_ready = status != MOTION_INIT_FAILED && status != MOTION_INVOKE_FAILED;
        if (status == MOTION_WAIT_100MS) r.interval_ms = 500;
        // Stale per-prediction fields intentionally left populated: statuses
        // must carry UNKNOWN and omit those fields rather than leaking them.
        check_motion_publish(&r);
    }
    // Fixed bounds, maximum scalar widths, exact-fit/short-buffer behavior.
    int64_t saved_clock = clock_us;
    clock_us = (INT64_C(1) << 54) + 5000123;
    r = motion_fixture(true);
    r.boot_id = r.generation = r.model_epoch = UINT32_MAX;
    r.clipped = 60; r.clipped_total = r.queue_drops = r.window_resets = r.invoke_failures = UINT32_MAX;
    r.inference_us = r.uptime_us; r.arena_used_bytes = 16384;
    memset(r.model_identity.version, 'v', 39); r.model_identity.version[39] = 0;
    e = check_motion_publish(&r);
    uint8_t guarded[TX_PAYLOAD_SIZE + 2]; size_t size = 0, ignored = 99;
    memset(guarded, 0xA5, sizeof(guarded));
    assert(aai16_protocol_encode(&e, guarded + 1, sizeof(guarded) - 2, &size) == CR(OK));
    assert(aai16_protocol_encode(&e, guarded + 1, size, &ignored) == CR(OK) && ignored == size);
    assert(guarded[0] == 0xA5 && guarded[size + 1] == 0xA5);
    assert(aai16_protocol_encode(&e, guarded + 1, size - 1, &ignored) == CR(TOO_LARGE) && !ignored);
    aai16_Envelope invalid;
    assert(aai16_protocol_decode((uint8_t *)last_payload, last_payload_len - 1, &invalid) != CR(OK));
    assert(aai16_protocol_decode((uint8_t *)last_payload, last_payload_len + 1, &invalid) != CR(OK));
    clock_us = saved_clock;

    int before = publish_calls;
    assert(mqtt_manager_publish_motion(NULL) == ESP_ERR_INVALID_ARG);
#define REJECT_MOTION(change) do { r = motion_fixture(true); change; \
    assert(mqtt_manager_publish_motion(&r) == ESP_ERR_INVALID_ARG); } while (0)
    REJECT_MOTION(r.status = -1);
    REJECT_MOTION(r.status = MOTION_SENSOR_UNAVAILABLE + 1);
    REJECT_MOTION(r.model_ready = false);
    REJECT_MOTION(r.class_id = -1);
    REJECT_MOTION(r.class_id = 2);
    REJECT_MOTION(r.model_identity.family = 0);
    REJECT_MOTION(r.model_identity.source = 0);
    REJECT_MOTION(r.model_identity.bank = 2);
    REJECT_MOTION(r.model_identity.version[0] = 0);
    REJECT_MOTION(strcpy(r.model_identity.version, "bad/version"));
    REJECT_MOTION(memset(r.model_identity.version, 'v', sizeof(r.model_identity.version)));
    REJECT_MOTION(r.interval_ms = 500);
    REJECT_MOTION(r.uptime_us = -1);
    REJECT_MOTION(r.uptime_us = clock_us + 1);
    REJECT_MOTION(r.window_start_us = -1);
    REJECT_MOTION(r.window_start_us = r.window_end_us + 1);
    REJECT_MOTION(r.window_end_us = r.uptime_us + 1);
    REJECT_MOTION(r.inference_us = -1);
    REJECT_MOTION(r.inference_us = r.uptime_us + 1);
    REJECT_MOTION(r.last_sequence++);
    REJECT_MOTION(r.clipped = 61);
    REJECT_MOTION(r.logits[0] = NAN);
    REJECT_MOTION(r.logits[1] = INFINITY);
    REJECT_MOTION(r.arena_used_bytes = 16385);
#if SIZE_MAX > UINT32_MAX
    REJECT_MOTION(r.arena_used_bytes = (size_t)UINT32_MAX + 1);
#endif
    REJECT_MOTION(r.status = MOTION_WAIT_100MS; r.interval_ms = 0);
#undef REJECT_MOTION
    assert(!uxQueueMessagesWaiting(s_tx_queue) && publish_calls == before);

    r = motion_fixture(false);
    event(MQTT_EVENT_DISCONNECTED);
    assert(mqtt_manager_publish_motion(&r) == ESP_ERR_INVALID_STATE);
    event(MQTT_EVENT_CONNECTED); pump();
    assert(publish_calls == before); // Rejected/offline samples have no side channel.
    r.uptime_us = clock_us;
    for (unsigned i = 0; i < TX_QUEUE_SIZE; ++i) assert(mqtt_manager_publish_motion(&r) == ESP_OK);
    assert(mqtt_manager_publish_motion(&r) == ESP_ERR_NO_MEM);
    pump();
    assert(publish_calls == before + TX_QUEUE_SIZE);
    before = publish_calls; r.uptime_us = clock_us;
    assert(mqtt_manager_publish_motion(&r) == ESP_OK);
    clock_us += (TELEMETRY_MAX_AGE_MS + 1) * 1000;
    pump(); assert(publish_calls == before); // Same freshness policy as other telemetry.
    r.uptime_us = clock_us;
    assert(mqtt_manager_publish_motion(&r) == ESP_OK);
    event(MQTT_EVENT_DISCONNECTED); pump(); event(MQTT_EVENT_CONNECTED); pump();
    assert(publish_calls == before); // Disconnected publisher drops without outbox replay.
    publish_result = -1; r.uptime_us = clock_us;
    uint32_t failed = s_stats[TX_MOTION].publish_failed;
    assert(mqtt_manager_publish_motion(&r) == ESP_OK); pump();
    assert(s_stats[TX_MOTION].publish_failed == failed + 1);
    publish_result = 0;
    puts("PASS: ML binary-only data topic, exact lengths, all fields/statuses, bounds/invalid inputs, QoS0, offline/full/stale/failure accounting");
}
