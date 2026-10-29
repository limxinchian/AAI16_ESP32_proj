// Real MQTT callback -> bounded FIFO -> sidecar -> owner validation -> binary
// ACK serializer. Platform MQTT/RTOS are doubles; protobuf code is production.
#include "pb_encode.h"
static uint64_t next_request = 100;
static aai16_Envelope config_frame(sensor_id_t sensor, bool gate, uint32_t value)
{
    aai16_Envelope e = aai16_Envelope_init_zero;
    e.has_protocol_version = e.has_request_id = e.has_phase = true;
    e.protocol_version = 1; e.request_id = ++next_request; e.phase = CP(VALIDATE);
    e.which_body = aai16_Envelope_sensor_config_tag;
    aai16_SensorConfig *c = &e.body.sensor_config;
    c->has_sensor = true; c->sensor = config_sensor_to_wire(sensor);
    c->has_enabled = gate; c->has_interval_ms = !gate;
    if (gate) c->enabled = value != 0; else c->interval_ms = value;
    return e;
}
static size_t raw_frame(const aai16_Envelope *e, uint8_t *bytes)
{
    pb_ostream_t out = pb_ostream_from_buffer(bytes, AAI16_CONFIG_FRAME_MAX);
    assert(pb_encode(&out, aai16_Envelope_fields, e));
    return out.bytes_written;
}
static void send_frame(const aai16_Envelope *e)
{
    uint8_t bytes[AAI16_CONFIG_FRAME_MAX]; size_t n = raw_frame(e, bytes);
    command((const char *)bytes, (int)n, (int)n, 0, false);
}
static aai16_Envelope ack_frame(void)
{
    aai16_Envelope e;
    assert(last_qos == 1 && !strcmp(last_topic, CONFIG_ACK_TOPIC));
    assert(aai16_protocol_decode((uint8_t *)last_payload, last_payload_len, &e) == CR(OK));
    assert(e.which_body == aai16_Envelope_config_ack_tag && e.boot_id == s_boot_id);
    return e;
}
static void assert_ack(uint64_t request, aai16_Phase phase, aai16_ResultCode result, uint32_t generation)
{
    pump(); aai16_Envelope e = ack_frame();
    assert(e.request_id == request && e.phase == phase && e.body.config_ack.result == result && e.generation == generation);
}
static mqtt_config_update_t start_owner(void)
{
    mqtt_config_update_t u, received;
    assert(mqtt_manager_peek_config_update(&u));
    assert(mqtt_manager_stage_owner_config(&u));
    assert(!mqtt_manager_stage_owner_config(&u)); // Occupied sidecar cannot be overwritten.
    mqtt_manager_cancel_owner_config(u.sensor); // Failed notification leaves FIFO intact.
    assert(mqtt_manager_stage_owner_config(&u));
    assert(mqtt_manager_take_config_update(&received));
    assert(mqtt_manager_take_owner_config(u.sensor, &received));
    assert(received.request_id == u.request_id && received.operation == u.operation);
    assert(!mqtt_manager_take_owner_config(u.sensor, &u));
    assert(mqtt_manager_begin_owner_config(&received));
    return received;
}
// Transport-priority tests need one genuinely admitted/owner-started receipt.
static esp_err_t queue_test_ack(void)
{
    aai16_Envelope e = config_frame(SENSOR_DHT22, false, 5000);
    send_frame(&e); mqtt_config_update_t u = start_owner();
    return mqtt_manager_publish_config_ack(&u);
}
static void rejected_frame(aai16_Envelope *e, aai16_ResultCode result)
{
    unsigned before = uxQueueMessagesWaiting(s_config_queue);
    send_frame(e); pump(); aai16_Envelope a = ack_frame();
    assert(a.request_id == e->request_id && a.phase == CP(VALIDATE) && a.body.config_ack.result == result);
    assert(uxQueueMessagesWaiting(s_config_queue) == before);
}
static void test_config_protocol(void)
{
    int allocations = allocs;
    // Forged/local queue-admission success is prohibited.
    assert(mqtt_manager_publish_config_ack(&(mqtt_config_update_t){.sensor=SENSOR_GY511}) == ESP_ERR_INVALID_STATE);
    for (unsigned i = 0; i < sizeof(s_config_rules)/sizeof(*s_config_rules); ++i) {
        const config_rule_t *rule = &s_config_rules[i];
        uint32_t values[] = {0, 1, rule->min_interval_ms, rule->min_interval_ms + 1, rule->max_interval_ms};
        for (unsigned n = 0; n < 5; ++n) {
            aai16_Envelope e = config_frame(rule->id, n < 2, values[n]);
            uint32_t generation = s_transactions.generation[rule->id];
            unsigned attempts = s_stats[TX_CONFIG_ACK].attempts;
            send_frame(&e); send_frame(&e); // Pending QoS duplicate: one queued command, no ACK.
            assert(uxQueueMessagesWaiting(s_config_queue) == 1 && s_stats[TX_CONFIG_ACK].attempts == attempts);
            mqtt_config_update_t pending; assert(mqtt_manager_peek_config_update(&pending));
            assert(mqtt_manager_publish_config_ack(&pending) == ESP_ERR_INVALID_STATE);
            mqtt_config_update_t u = start_owner();
            assert(u.generation == generation + 1 && u.request_id == e.request_id);
            assert(n < 2 ? u.enabled == (bool)values[n] : u.interval_ms == values[n]);
            assert(mqtt_manager_publish_config_ack(&u) == ESP_OK);
            assert_ack(e.request_id, CP(APPLY), CR(OK), generation + 1);
            aai16_Envelope ack = ack_frame();
            assert(ack.body.config_ack.requested.has_enabled == (n < 2));
            if (n < 2) assert(ack.body.config_ack.requested.enabled == (bool)values[n]);
            send_frame(&e); assert_ack(e.request_id, CP(APPLY), CR(OK), generation + 1);
            assert(!uxQueueMessagesWaiting(s_config_queue));
            assert(s_transactions.generation[rule->id] == generation + 1);
            // Same ID, changed operation/value is a conflict, not another APPLY.
            e.body.sensor_config.has_enabled = true; e.body.sensor_config.has_interval_ms = false;
            e.body.sensor_config.enabled = n == 0;
            rejected_frame(&e, CR(BUSY));
        }
        aai16_Envelope e = config_frame(rule->id, false, rule->min_interval_ms - 1);
        rejected_frame(&e, CR(OUT_OF_RANGE));
        e.request_id = ++next_request; e.body.sensor_config.interval_ms = rule->max_interval_ms + 1;
        rejected_frame(&e, CR(OUT_OF_RANGE));
    }
    aai16_Envelope e = config_frame(SENSOR_GY511, true, 0);
    e.body.sensor_config.has_enabled = false; rejected_frame(&e, CR(MISSING_FIELD));
    e.body.sensor_config.has_enabled = true; e.body.sensor_config.has_interval_ms = true;
    rejected_frame(&e, CR(AMBIGUOUS_OPERATION));
    e = config_frame(SENSOR_GY511, true, 0); e.body.sensor_config.has_sensor = false;
    rejected_frame(&e, CR(MISSING_FIELD));
    e = config_frame(SENSOR_GY511, true, 0); e.protocol_version = 2;
    rejected_frame(&e, CR(UNSUPPORTED_VERSION));
    e = config_frame(SENSOR_GY511, true, 0); e.has_generation = true;
    rejected_frame(&e, CR(NOT_SUPPORTED));
    e = config_frame(SENSOR_GY511, true, 0); e.has_boot_id = true; e.boot_id = s_boot_id + 1;
    rejected_frame(&e, CR(STALE_GENERATION));
    e = config_frame(SENSOR_GY511, true, 0); e.phase = CP(APPLY);
    rejected_frame(&e, CR(OUT_OF_RANGE));

    // Expected generation is checked by the owner after preceding commands.
    e = config_frame(SENSOR_GY511, true, 0);
    uint32_t generation = s_transactions.generation[SENSOR_GY511];
    e.body.sensor_config.has_expected_generation = true; e.body.sensor_config.expected_generation = generation;
    send_frame(&e); aai16_Envelope stale = e; stale.request_id = ++next_request; send_frame(&stale);
    mqtt_config_update_t u = start_owner(); assert(mqtt_manager_publish_config_ack(&u) == ESP_OK); pump();
    assert(mqtt_manager_take_config_update(&u)); assert(!mqtt_manager_begin_owner_config(&u));
    assert_ack(stale.request_id, CP(APPLY), CR(STALE_GENERATION), generation + 1);
    assert(s_transactions.generation[SENSOR_GY511] == generation + 1);
    // Owner-specific failure never increments generation and is replayable.
    e = config_frame(SENSOR_GY511, false, 100); send_frame(&e); u = start_owner();
    assert(mqtt_manager_reject_config(&u, "sensor_disabled") == ESP_OK);
    assert_ack(e.request_id, CP(APPLY), CR(SENSOR_DISABLED), generation + 1);
    send_frame(&e); assert_ack(e.request_id, CP(APPLY), CR(SENSOR_DISABLED), generation + 1);

    // Fragment boundaries include omitted topic on continuations (ESP-MQTT API).
    e = config_frame(SENSOR_GY511, true, 0);
    uint8_t bytes[AAI16_CONFIG_FRAME_MAX + 1]; size_t len = raw_frame(&e, bytes);
    for (size_t cut = 1; cut < len; ++cut) {
        e.request_id = ++next_request; len = raw_frame(&e, bytes);
        command((char *)bytes, (int)cut, (int)len, 0, false);
        assert(!uxQueueMessagesWaiting(s_config_queue));
        esp_mqtt_event_t fragment = {.data=(char *)bytes+cut, .data_len=(int)(len-cut),
            .total_data_len=(int)len, .current_data_offset=(int)cut};
        mqtt_event_handler(NULL, NULL, MQTT_EVENT_DATA, &fragment);
        u = start_owner(); assert(mqtt_manager_publish_config_ack(&u) == ESP_OK); pump();
    }
    // Bad offsets/lengths, timeout, retained, disconnect and replacement messages.
    e=config_frame(SENSOR_GY511,true,0); len=raw_frame(&e,bytes);
    command(NULL,0,(int)len,0,false);
    assert(s_fragments.active && !s_fragments.received);
    esp_mqtt_event_t topic_only_followup = {.data=(char *)bytes,.data_len=(int)len,.total_data_len=(int)len};
    mqtt_event_handler(NULL,NULL,MQTT_EVENT_DATA,&topic_only_followup);
    u=start_owner(); assert(mqtt_manager_publish_config_ack(&u)==ESP_OK); pump();
    unsigned invalid = s_fragments.rejected, expired = s_fragments.expired;
    command((char *)bytes, 2, (int)len, 0, false);
    command((char *)bytes+2, 2, (int)len, 3, false);
    assert(!s_fragments.active && s_fragments.rejected == invalid + 1);
    command((char *)bytes, 2, (int)len, 0, false);
    clock_us += CONFIG_FRAGMENT_TIMEOUT_US; mqtt_manager_peek_config_update(&u);
    assert(!s_fragments.active && s_fragments.expired == expired + 1);
    command((char *)bytes+2, (int)len-2, (int)len, 2, false);
    command((char *)bytes, (int)len, (int)len, 0, true);
    command((char *)bytes, 1, 129, 0, false);
    command((char *)bytes, 2, 1, 0, false);
    command((char *)bytes, 2, (int)len, -1, false);
    command((char *)bytes, 2, (int)len, 0, false); event(MQTT_EVENT_DISCONNECTED);
    event(MQTT_EVENT_CONNECTED); command((char *)bytes+2, (int)len-2, (int)len, 2, false);
    for (unsigned variant = 0; variant < 7; ++variant) {
        command((char *)bytes, 2, (int)len, 0, false);
        esp_mqtt_event_t bad_fragment = {.data=(char *)bytes+2, .data_len=(int)len-2,
            .total_data_len=(int)len, .current_data_offset=2};
        switch (variant) {
        case 0: bad_fragment.msg_id = 1; break;
        case 1: bad_fragment.total_data_len++; break;
        case 2: bad_fragment.data_len = -1; break;
        case 3: bad_fragment.data = NULL; break;
        case 4: bad_fragment.retain = true; break;
        case 5: bad_fragment.topic = "other"; bad_fragment.topic_len = 5; break;
        case 6: bad_fragment.data_len++; break;
        }
        mqtt_event_handler(NULL,NULL,MQTT_EVENT_DATA,&bad_fragment);
        assert(!s_fragments.active && !uxQueueMessagesWaiting(s_config_queue));
    }
    command((char *)bytes, 2, (int)len, 0, false);
    e=config_frame(SENSOR_GY511,true,1); send_frame(&e); // Abandons older partial input.
    u=start_owner(); assert(u.request_id==e.request_id);
    assert(mqtt_manager_publish_config_ack(&u)==ESP_OK); pump();
    assert(!uxQueueMessagesWaiting(s_config_queue));
    // Unknown field, duplicate, invalid bool, truncation, JSON: fail closed, no ID invented.
    unsigned attempts = s_stats[TX_CONFIG_ACK].attempts;
    const uint8_t bad[][6] = {{0x52,4,8,2,16,2},{8,1,8,1},{0x52,4,8,2,40,1}};
    for (unsigned i=0;i<3;++i) command((char *)bad[i], 6, 6, 0, false);
    command("{\"enabled\":false}", 17, 17, 0, false);
    for (size_t cut=1;cut<len;++cut) command((char *)bytes,(int)cut,(int)cut,0,false);
    assert(!uxQueueMessagesWaiting(s_config_queue));
    // Some truncated frames are structurally valid headers and receive semantic failure ACKs.
    assert(s_stats[TX_CONFIG_ACK].attempts >= attempts); pump();

    // Four-entry FIFO remains bounded; failure cached at VALIDATE, never mutates settings.
    aai16_Envelope requests[5];
    for (unsigned i=0;i<5;++i) { requests[i]=config_frame(SENSOR_GY511,true,i%2); send_frame(&requests[i]); }
    assert_ack(requests[4].request_id, CP(VALIDATE), CR(QUEUE_FULL), s_transactions.generation[SENSOR_GY511]);
    for (unsigned i=0;i<4;++i) {
        u = start_owner(); assert(u.request_id==requests[i].request_id);
        assert(mqtt_manager_publish_config_ack(&u)==ESP_OK); pump();
    }
    // Failed outgoing ACK: retry recovers the receipt without reapplying.
    e=config_frame(SENSOR_GY511,true,1); send_frame(&e); u=start_owner();
    event(MQTT_EVENT_DISCONNECTED); assert(mqtt_manager_publish_config_ack(&u)==ESP_ERR_INVALID_STATE);
    generation=s_transactions.generation[SENSOR_GY511]; event(MQTT_EVENT_CONNECTED); send_frame(&e);
    assert_ack(e.request_id,CP(APPLY),CR(OK),generation);
    // Evicted ID cannot become a fresh command.
    aai16_Envelope old=e;
    for(unsigned i=0;i<CONFIG_HISTORY_SIZE;++i) { assert(queue_test_ack()==ESP_OK); pump(); }
    rejected_frame(&old,CR(STALE_GENERATION));
    // Pending history entries cannot be evicted, and generations never wrap.
    config_transactions_t bounded = {0}; config_receipt_t reply;
    for (unsigned i=0;i<CONFIG_HISTORY_SIZE;++i) {
        e=config_frame(SENSOR_GY511,true,1);
        assert(config_admit(&bounded,&e,s_boot_id,&u,&reply)==1);
    }
    e=config_frame(SENSOR_GY511,true,1);
    assert(config_admit(&bounded,&e,s_boot_id,&u,&reply)==-1 && reply.result==CR(BUSY));
    bounded=(config_transactions_t){0};
    assert(config_admit(&bounded,&e,s_boot_id,&u,&reply)==1);
    bounded.generation[SENSOR_GY511]=UINT32_MAX;
    assert(!config_begin(&bounded,&u,&reply) && reply.result==CR(STALE_GENERATION));
    assert(bounded.generation[SENSOR_GY511]==UINT32_MAX);
    assert(allocs==allocations);
    printf("PASS: Protobuf live config, all sensors, owner ACK, bounds, fragments, generations, duplicates; state=%zu rx=%zu update=%zu\n",
        sizeof(s_transactions),sizeof(s_fragments),sizeof(mqtt_config_update_t));
}
