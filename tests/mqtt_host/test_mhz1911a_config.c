// Included after the Phase 2 tests. Uses the production parser, mailbox,
// admission logic, ACK formatting and FIFO with deterministic transport doubles.
static void co2_disable_before_admission(void) { mqtt_manager_commit_mhz1911a_config(false); }
static void co2_reschedule_before_admission(void) { mqtt_manager_commit_mhz1911a_config(true); }
static void co2_toggle_before_admission(void)
{
    mqtt_manager_commit_mhz1911a_config(false);
    mqtt_manager_commit_mhz1911a_config(true);
    assert(mqtt_manager_update_mhz1911a_snapshot(1600, ++clock_us) == ESP_OK);
}
static void co2_disable_inside_sdk(void)
{
    publish_hook = NULL;
    mqtt_manager_commit_mhz1911a_config(false);
    assert(mqtt_manager_update_mhz1911a_snapshot(1601, ++clock_us) == ESP_ERR_INVALID_STATE);
}
static void *co2_stress_config(void *arg)
{
    (void)arg;
    for (unsigned i = 0; i < 10000; ++i) mqtt_manager_commit_mhz1911a_config(i % 2);
    return NULL;
}

static void test_mhz1911a_config(void)
{
    tx_stats_t *stats = &s_stats[TX_MHZ1911A];
    int calls = publish_calls, allocations = allocs;
    unsigned attempts = stats->attempts;
    assert(mqtt_manager_update_mhz1911a_snapshot(1500, ++clock_us) == ESP_OK);
    mqtt_manager_commit_mhz1911a_config(false);
    assert(!s_mhz1911a.pending && !s_mhz1911a_enabled);
    assert(mqtt_manager_update_mhz1911a_snapshot(1501, ++clock_us) == ESP_ERR_INVALID_STATE);
    pump(); assert(publish_calls == calls && stats->attempts == attempts);
    event(MQTT_EVENT_DISCONNECTED); mqtt_manager_commit_mhz1911a_config(true);
    event(MQTT_EVENT_CONNECTED); pump(); assert(publish_calls == calls);
    assert(mqtt_manager_update_mhz1911a_snapshot(1502, ++clock_us) == ESP_OK);
    mqtt_manager_commit_mhz1911a_config(true); // Interval change also clears pending data.
    pump(); assert(publish_calls == calls && stats->attempts == attempts);

    void (*hooks[])(void) = {co2_disable_before_admission, co2_reschedule_before_admission, co2_toggle_before_admission};
    for (unsigned i = 0; i < 3; ++i) {
        mqtt_manager_commit_mhz1911a_config(true);
        assert(mqtt_manager_update_mhz1911a_snapshot(1550, ++clock_us) == ESP_OK);
        uint32_t cancelled = stats->cancelled;
        timer_hook = hooks[i]; timer_hook_countdown = 2; // After copy, before SDK admission.
        service_mhz1911a_snapshot();
        assert(!timer_hook && stats->cancelled == cancelled + 1 && publish_calls == calls);
        if (i == 2) {
            int64_t fresh = clock_us;
            assert(s_mhz1911a.pending);
            service_mhz1911a_snapshot(); ++calls;
            assert(publish_calls == calls); check_co2_json(1600, fresh, stats->attempts);
        } else assert(!s_mhz1911a.pending);
    }
    assert(mqtt_manager_update_mhz1911a_snapshot(1700, ++clock_us) == ESP_OK);
    int64_t submitted = clock_us;
    publish_hook = co2_disable_inside_sdk;
    service_mhz1911a_snapshot(); ++calls;
    assert(publish_calls == calls && !s_mhz1911a_enabled && !s_mhz1911a.pending);
    check_co2_json(1700, submitted, stats->attempts); // Already admitted cannot be recalled.
    pump(); assert(publish_calls == calls);
    mqtt_manager_commit_mhz1911a_config(true);
    event(MQTT_EVENT_DISCONNECTED);
    assert(mqtt_manager_update_mhz1911a_snapshot(1750, ++clock_us) == ESP_OK);
    mqtt_manager_commit_mhz1911a_config(false);
    event(MQTT_EVENT_CONNECTED); mqtt_manager_commit_mhz1911a_config(true);
    pump(); assert(publish_calls == calls);
    assert(mqtt_manager_update_mhz1911a_snapshot(1800, ++clock_us) == ESP_OK);
    submitted = clock_us; pump(); ++calls;
    assert(publish_calls == calls); check_co2_json(1800, submitted, stats->attempts);
    assert(allocs == allocations);
    puts("PASS: CO2 disable/interval cancellation before admission, disable-enable epoch, in-flight SDK boundary and reconnect freshness");

    // Parallel config and producer threads use the same real critical-section
    // boundaries as firmware; time is immutable during the threaded interval.
    co2_stress_base = clock_us + 1; clock_us = co2_stress_base + 20001;
    pthread_t producer, config;
    assert(!pthread_create(&producer, NULL, co2_stress_producer, (void *)(uintptr_t)0));
    assert(!pthread_create(&config, NULL, co2_stress_config, NULL));
    assert(!pthread_join(producer, NULL) && !pthread_join(config, NULL));
    mqtt_manager_commit_mhz1911a_config(false);
    assert(!s_mhz1911a.pending && !s_mhz1911a_enabled);
    pump(); assert(publish_calls == calls);
    mqtt_manager_commit_mhz1911a_config(true);
    assert(mqtt_manager_update_mhz1911a_snapshot(1900, ++clock_us) == ESP_OK);
    pump();
    assert(allocs == allocations && !critical_depth);
    assert(stats->attempts == stats->publish_ok + stats->publish_failed + stats->offline + stats->stale + stats->invalid + stats->cancelled);
    puts("PASS: 10000 concurrent CO2 controls/producer handoffs, bounded mailbox, no post-disable sample and conserved attempt accounting");
}
