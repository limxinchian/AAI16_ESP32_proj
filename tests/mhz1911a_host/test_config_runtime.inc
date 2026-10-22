// Real dispatcher, notifications, reader scheduling and registry under the
// six-task scheduler. Transport admission is separately tested in mqtt_host.
static void co2_rejected_interval(int64_t at_us)
{
    input(at_us, SENSOR_MHZ1911A, 60000);
    inputs[input_count - 1].rejection = "sensor_disabled";
}

static void test_co2_controls(const char *scenario)
{
    co2_controls = co2_active = ultra_active = radar_active = rc_active = true;
    rc_scenario = "rc_default";
    end_us = 125000000;
    bool interval = !strcmp(scenario, "co2_config_interval");
    bool maximum = !strcmp(scenario, "co2_config_max");
    bool idle = !strcmp(scenario, "co2_config_idle");
    bool inflight = !strcmp(scenario, "co2_config_inflight");
    bool inflight_success = !strcmp(scenario, "co2_config_inflight_success");
    bool burst = !strcmp(scenario, "co2_config_burst");
    bool warmup = !strcmp(scenario, "co2_config_warmup");
    bool near_warmup = !strcmp(scenario, "co2_config_warmup_end");
    bool paused = !strcmp(scenario, "co2_config_paused");
    bool recovery = !strcmp(scenario, "co2_config_recovery");
    bool init = !strcmp(scenario, "co2_config_init");
    bool task = !strcmp(scenario, "co2_config_task");
    preempt_on_notify = !strcmp(scenario, "co2_config_preempt");
    if (interval) {
        input(63000000, SENSOR_MHZ1911A, 10000);
        input(95000000, SENSOR_MHZ1911A, 5000);
    } else if (maximum) {
        input(10000000, SENSOR_MHZ1911A, 60000);
    } else if (idle || preempt_on_notify) {
        input(5000000, SENSOR_MHZ1911A, 10000);
        gate_input(70500000, SENSOR_MHZ1911A, false);
        gate_input(71000000, SENSOR_MHZ1911A, false);
        co2_rejected_interval(72000000);
        gate_input(80000000, SENSOR_MHZ1911A, true);
        gate_input(81000000, SENSOR_MHZ1911A, true);
    } else if (inflight || burst || inflight_success) {
        co2_fail_first = !inflight_success; // Notification arrives inside a bounded read.
        co2_slow_success = inflight_success;
        if (burst) input(60100000, SENSOR_MHZ1911A, 10000);
        gate_input(60200000, SENSOR_MHZ1911A, false);
        co2_rejected_interval(60300000);
        gate_input(burst ? 60400000 : 64000000, SENSOR_MHZ1911A, true);
    } else if (warmup || near_warmup) {
        gate_input(warmup ? 10000000 : 55000000, SENSOR_MHZ1911A, false);
        co2_rejected_interval(warmup ? 20000000 : 56000000);
        gate_input(warmup ? 30000000 : 58000000, SENSOR_MHZ1911A, true);
    } else if (paused) {
        gate_input(10000000, SENSOR_MHZ1911A, false);
        co2_rejected_interval(121000000);
    } else if (recovery) {
        co2_recovery = true;
        gate_input(78000000, SENSOR_MHZ1911A, false);
        gate_input(80000000, SENSOR_MHZ1911A, true);
    } else {
        assert(init || task);
        co2_init_failure = init; co2_task_failure = task;
        gate_input(0, SENSOR_MHZ1911A, true);
        inputs[input_count - 1].rejection = "sensor_unavailable";
        input(10000000, SENSOR_MHZ1911A, 10000);
        inputs[input_count - 1].rejection = "sensor_unavailable";
        gate_input(20000000, SENSOR_MHZ1911A, false);
        inputs[input_count - 1].rejection = "sensor_unavailable";
    }
    // Exercise the other five owners throughout pause, long reads and changes.
    simulate(); verify_commands();
    assert(co2_inits == (task ? 0U : 1U) && co2_handoffs == co2_ok);
    assert(dht_count == 62 && gy_count > 240 && rc_count > 620 && ultra_reads > 620);
    assert(radar_valid > 1000 && radar_snapshots == radar_valid && !radar_deinits);
    for (sensor_id_t id = SENSOR_DHT22; id <= SENSOR_HCSR04P; ++id) {
        sensor_status_t s;
        assert(sensor_status_get(id, &s) && s.state == SENSOR_READY && !s.read_failures);
        assert(s.last_success_us >= 124000000);
    }
    sensor_status_t s;
    assert(sensor_status_get(SENSOR_MHZ1911A, &s));
    assert(s.read_successes == co2_ok && s.read_failures == co2_failed);
    if (init || task) {
        assert(!co2_reads && !co2_commits && s.state == SENSOR_UNAVAILABLE);
        assert(!mhz1911a_reader_task_handle());
    } else if (paused) {
        assert(!co2_reads && s.state == SENSOR_DISABLED && co2_commits == 1);
        assert(test_logs_contain("period=5000 ms enabled=false"));
    } else {
        assert(mhz1911a_reader_task_handle() == &tasks[5] && co2_reads && s.state == SENSOR_READY);
        for (unsigned i = 1; i < co2_reads; ++i) assert(co2_starts[i] - co2_starts[i-1] >= 5000000);
        if (interval) {
            assert(co2_starts[0] == 60000000);
            assert(co2_starts[1] == acks[0].at_us + 10000000);
            assert(co2_starts[2] - co2_starts[1] == 10000000);
            assert(co2_starts[4] == acks[1].at_us + 5000000);
        }
        if (maximum) assert(co2_reads == 1 && co2_starts[0] == acks[0].at_us + 60000000);
        if (idle || preempt_on_notify) {
            assert(co2_starts[0] == 60000000 && co2_starts[1] == 70000000);
            assert(co2_starts[2] == acks[4].at_us + 10000000);
            for (unsigned i = 3; i < co2_reads; ++i) assert(co2_starts[i] - co2_starts[i-1] == 10000000);
            assert(co2_commits == 3); // Idempotent gates don't reset schedule/mailbox.
        }
        if (inflight || burst || inflight_success) {
            assert(acks[0].at_us >= (inflight_success ? 60500000 : 62000000));
            assert(co2_failed == (inflight_success ? 0U : 1U));
            assert(co2_starts[0] == 60000000);
            assert(co2_starts[1] == acks[ack_count-1].at_us + (burst ? 10000000 : 5000000));
            assert(acks[ack_count-1].state == SENSOR_UNINITIALIZED);
            if (burst) assert(notify_busy > 0);
        }
        if (warmup) assert(co2_starts[0] == 60000000);
        if (near_warmup) assert(co2_starts[0] == acks[2].at_us + 5000000 && co2_starts[0] >= 63000000);
        if (recovery) {
            assert(co2_failed == 3 && !s.consecutive_failures);
            assert(acks[0].state == SENSOR_DISABLED && acks[1].state == SENSOR_UNINITIALIZED);
            assert(co2_starts[4] == acks[1].at_us + 5000000);
        }
    }
    printf("PASS: six-sensor runtime control/ACK/scheduling %s\n", scenario);
}
