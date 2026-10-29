// Included by test_runtime.c: real dispatch, five readers and status registry
// with deterministic cooperative scheduling; SDK/driver calls are doubles.
static void test_hcsr04p_config(const char *scenario)
{
    ultra_active = radar_active = rc_active = true;
    rc_scenario = "rc_default";
    ultra_init_failure = !strcmp(scenario, "ultra_cfg_init");
    ultra_task_failure = !strcmp(scenario, "ultra_cfg_task");
    ultra_missing = !strcmp(scenario, "ultra_cfg_missing") || !strcmp(scenario, "ultra_cfg_recovery");
    ultra_recover_on_enable = !strcmp(scenario, "ultra_cfg_recovery");
    preempt_on_notify = !strcmp(scenario, "ultra_cfg_preempt");
    bool unavailable = ultra_init_failure || ultra_task_failure;
    bool paused = !strcmp(scenario, "ultra_cfg_paused");
    bool burst = !strcmp(scenario, "ultra_cfg_burst") || preempt_on_notify;
    bool during_read = !strcmp(scenario, "ultra_cfg_read");
    bool intervals = !strcmp(scenario, "ultra_cfg_interval");
    bool rounding = !strcmp(scenario, "ultra_cfg_rounding");
    bool shorten = !strcmp(scenario, "ultra_cfg_shorten");
    if (!strcmp(scenario, "ultra_cfg_wrap")) base_us = ((int64_t)UINT32_MAX - 30) * 10000;
    if (!strcmp(scenario, "ultra_cfg_uninitialized")) ultra_first_delay_us = 4000000;
    end_us = intervals ? 18200000 : paused ? 12200000 : 8200000;
    test_now_us = base_us;

    // Other sensors receive real owner-committed commands throughout this test.
    input(2350000, SENSOR_DHT22, 5000);
    input(2350000, SENSOR_GY511, 1000);
    input(2350000, SENSOR_RC522, 500);
    input(5500000, SENSOR_LD2450, 2000);
    if (unavailable || ultra_first_delay_us) {
        gate_input(0, SENSOR_HCSR04P, false); inputs[input_count - 1].rejection = "sensor_unavailable";
        input(ultra_first_delay_us ? 4500000 : 2750000, SENSOR_HCSR04P, 500);
        if (unavailable) inputs[input_count - 1].rejection = "sensor_unavailable";
        gate_input(ultra_first_delay_us ? 6000000 : 4500000, SENSOR_HCSR04P, true);
        if (unavailable) inputs[input_count - 1].rejection = "sensor_unavailable";
    } else if (intervals || rounding || shorten) {
        input(2300000, SENSOR_HCSR04P, shorten ? 10000 : rounding ? 101 : 500);
        input(4500000, SENSOR_HCSR04P, 100);
        input(6000000, SENSOR_HCSR04P, intervals ? 10000 : 500);
    } else if (burst) {
        gate_input(3000000, SENSOR_HCSR04P, false);
        input(3000000, SENSOR_HCSR04P, 500); inputs[input_count - 1].rejection = "sensor_disabled";
        gate_input(3000000, SENSOR_HCSR04P, true);
        input(3000000, SENSOR_HCSR04P, 100);
    } else if (during_read) {
        // After app_main's existing 2 s DHT startup delay, it polls at 100 ms.
        // The reader starts ECHO first, then yields to the dispatcher.
        gate_input(2200000, SENSOR_HCSR04P, false);
        gate_input(2500000, SENSOR_HCSR04P, true);
        gate_input(2550000, SENSOR_HCSR04P, true);
    } else {
        input(2300000, SENSOR_HCSR04P, 500);
        gate_input(2600000, SENSOR_HCSR04P, false);
        input(3000000, SENSOR_HCSR04P, 100); inputs[input_count - 1].rejection = "sensor_disabled";
        gate_input(3400000, SENSOR_HCSR04P, false);
        if (!paused) {
            gate_input(4500000, SENSOR_HCSR04P, true);
            gate_input(4700000, SENSOR_HCSR04P, true);
            gate_input(7000000, SENSOR_HCSR04P, false);
            gate_input(7500000, SENSOR_HCSR04P, true);
        }
    }
    // Stable ordering for equal-time commands; preserve FIFO order within bursts.
    for (unsigned i = 1; i < input_count; ++i) {
        input_t held = inputs[i]; unsigned j = i;
        while (j && inputs[j - 1].at_us > held.at_us) { inputs[j] = inputs[j - 1]; --j; }
        inputs[j] = held;
    }
    simulate(); verify_commands();
    assert(ultra_inits == (ultra_task_failure ? 0U : 1U));
    assert(dht_count >= 2 && gy_count >= 10 && rc_count >= 20 && radar_snapshots >= 50);
    for (sensor_id_t id = SENSOR_DHT22; id <= SENSOR_LD2450; ++id) {
        sensor_status_t status;
        assert(sensor_status_get(id, &status) && status.state == SENSOR_READY && !status.read_failures);
    }
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_HCSR04P, &status));
    assert(ultra_snapshots == status.read_successes);
    if (unavailable) {
        assert(!ultra_reads && !ultra_snapshots && status.state == SENSOR_UNAVAILABLE);
        assert(hcsr04p_reader_task_handle() == NULL);
    } else {
        assert(hcsr04p_reader_task_handle() != NULL && ultra_reads > 2);
        bool enabled = true;
        int64_t period = 200000, due = ultra_first_delay_us;
        unsigned cursor = 0;
        for (unsigned i = 0; i < ultra_reads; ++i) {
            // ACK read_count establishes the exact ordering at equal timestamps:
            // a read already started when notified must finish before commitment.
            while (cursor < ack_count) {
                const ack_t *ack = &acks[cursor];
                if (ack->update.sensor != SENSOR_HCSR04P || !ack->applied) { cursor++; continue; }
                if (ack->read_count > i) break;
                if (ack->update.operation == MQTT_CONFIG_INTERVAL) {
                    period = ack->update.interval_ms * 1000LL;
                    due = ack->at_us + period;
                } else if (enabled != ack->update.enabled) {
                    enabled = ack->update.enabled;
                    due = ack->at_us + period;
                }
                cursor++;
            }
            assert(enabled && ultra_times[i] >= due && ultra_times[i] < due + 10000);
            if (i) assert(ultra_times[i] - ultra_times[i - 1] >= HCSR04P_MIN_INTERVAL_MS * 1000LL);
            due = ultra_times[i] + period;
        }
        if (paused) {
            assert(status.state == SENSOR_DISABLED && ultra_health_saved && !ultra_enabled);
            assert(status.read_successes == ultra_disabled_health.read_successes &&
                   status.read_failures == ultra_disabled_health.read_failures &&
                   status.last_success_us == ultra_disabled_health.last_success_us);
            assert(test_logs_contain("enabled=false overruns_total="));
        } else {
            assert(status.state == (ultra_missing ? SENSOR_DEGRADED : SENSOR_READY));
        }
        if (ultra_recover_on_enable) assert(status.read_failures > 0 && status.read_successes > 0 && !status.consecutive_failures);
        if (during_read) {
            assert(ultra_active_notifications > 0);
            assert(ultra_times[11] == 2200000);
            assert(acks[0].read_count == 12 && acks[0].at_us == 2240000);
        }
        if (burst && !preempt_on_notify) assert(notify_busy + test_config_sidecar_busy > 0);
        if (intervals) assert(ultra_times[ultra_reads - 1] >= 16000000);
    }
    printf("PASS: HC-SR04P owner configuration, deadlines, gate/history and five-sensor isolation %s\n", scenario);
}
