// All six production reader loops and the real status registry. UART is an
// explicit acquisition double here; real dual-UART drivers run in test_driver.
static bool co2_missing, co2_recovery, co2_init_failure, co2_slow;
static unsigned co2_inits, co2_reads, co2_ok, co2_failed;
static unsigned co2_handoffs;
static int64_t co2_last_handoff = -1;
static int64_t co2_starts[32];
static bool co2_gate = true, co2_read_active, co2_saved;
static bool co2_controls, co2_fail_first, co2_slow_success;
static unsigned co2_commits;
static sensor_status_t co2_paused_health;

void mqtt_manager_commit_mhz1911a_config(bool enabled)
{
    assert(task_index() == 5 && !co2_read_active);
    co2_gate = enabled; ++co2_commits;
}
static void co2_check_ack(const mqtt_config_update_t *update, const sensor_status_t *s)
{
    assert(!co2_read_active);
    if (update->operation != MQTT_CONFIG_ENABLED) { assert(co2_commits); return; }
    assert(co2_gate == update->enabled);
    if (co2_saved) {
        assert(s->read_successes == co2_paused_health.read_successes &&
               s->read_failures == co2_paused_health.read_failures &&
               s->consecutive_failures == co2_paused_health.consecutive_failures &&
               s->last_error == co2_paused_health.last_error &&
               s->last_success_us == co2_paused_health.last_success_us &&
               s->initialized == co2_paused_health.initialized);
        if (update->enabled) assert(s->state == SENSOR_UNINITIALIZED);
    }
    co2_saved = !update->enabled;
    if (co2_saved) co2_paused_health = *s;
}

esp_err_t mhz1911a_sensor_init(void)
{
    assert(task_index() == 5); ++co2_inits;
    return !co2_active || co2_init_failure ? ESP_ERR_NO_MEM : ESP_OK;
}
esp_err_t mhz1911a_sensor_read(mhz1911a_reading_t *out)
{
    assert(task_index() == 5 && co2_reads < 32 && test_now_us >= base_us + 60000000);
    assert(co2_gate && !co2_read_active);
    co2_read_active = true;
    sensor_status_t before;
    assert(sensor_status_get(SENSOR_MHZ1911A, &before));
    assert(before.read_successes == co2_ok && before.read_failures == co2_failed);
    if (!co2_reads) assert(before.state == SENSOR_UNINITIALIZED && !before.initialized);
    if (!co2_controls && co2_recovery && co2_reads == 4) assert(before.state == SENSOR_UNAVAILABLE && before.consecutive_failures == 3);
    if (!co2_controls && co2_recovery && co2_reads == 5) assert(before.state == SENSOR_READY && !before.consecutive_failures && before.read_failures == 3);
    co2_starts[co2_reads++] = test_now_us - base_us;
    bool fail = co2_missing || (co2_fail_first && co2_reads == 1) || (co2_recovery && co2_reads >= 2 && co2_reads <= 4);
    // Bounded blocking UART latency yields, letting all other owners proceed.
    vTaskDelay(pdMS_TO_TICKS(co2_slow && co2_reads == 1 ? 6500 : fail ? 2000 : co2_slow_success ? 500 : 30));
    co2_read_active = false;
    if (fail) { ++co2_failed; return ESP_ERR_TIMEOUT; }
    ++co2_ok; out->co2_ppm = 1770; return ESP_OK;
}
void mhz1911a_sensor_get_diagnostics(mhz1911a_diagnostics_t *out)
{
    *out = (mhz1911a_diagnostics_t){.requests=co2_reads, .valid_frames=co2_ok, .timeouts=co2_failed};
}

esp_err_t mqtt_manager_update_mhz1911a_snapshot(uint16_t ppm, int64_t acquired_us)
{
    assert(task_index() == 5 && ppm == 1770 && acquired_us == test_now_us);
    assert(co2_gate && !co2_read_active);
    assert(acquired_us > co2_last_handoff && acquired_us >= base_us + 60000000);
    assert(++co2_handoffs == co2_ok);
    co2_last_handoff = acquired_us;
    // Deliberately reject handoffs: transport failure cannot change any reader's
    // status, timing or other-sensor configuration handling.
    return ESP_ERR_INVALID_STATE;
}

static void test_co2_isolation(const char *scenario)
{
    co2_active = ultra_active = radar_active = rc_active = true;
    rc_scenario = "rc_default";
    co2_missing = !strcmp(scenario, "co2_missing");
    co2_recovery = !strcmp(scenario, "co2_recovery");
    co2_init_failure = !strcmp(scenario, "co2_init");
    co2_task_failure = !strcmp(scenario, "co2_task");
    co2_slow = !strcmp(scenario, "co2_slow");
    end_us = 125000000;
    input(2300000, SENSOR_GY511, 1000);
    input(60500000, SENSOR_GY511, 500); // During warm-up completion/first UART wait.
    input(70500000, SENSOR_GY511, 1000); // During a disconnected read in recovery scenario.
    simulate(); verify_commands();
    assert(wifi_starts == 1 && mqtt_starts == 1);
    assert(acks[1].at_us >= 60500000 && acks[1].at_us < 61000000);
    assert(acks[2].at_us >= 70500000 && acks[2].at_us < 71000000);
    assert(dht_count == 62 && gy_count > 120 && rc_count > 620 && ultra_reads > 620);
    assert(dht_tx == dht_count && gy_tx == gy_count);
    assert(radar_valid > 1000 && radar_snapshots == radar_valid && !radar_deinits);
    for (unsigned i = 1; i < ultra_reads; ++i) assert(ultra_times[i] - ultra_times[i-1] == 200000);
    for (sensor_id_t id = SENSOR_DHT22; id <= SENSOR_HCSR04P; ++id) {
        sensor_status_t s;
        assert(sensor_status_get(id, &s) && s.state == SENSOR_READY && !s.read_failures);
        assert(s.last_success_us >= 124000000);
    }
    sensor_status_t s;
    assert(sensor_status_get(SENSOR_MHZ1911A, &s));
    assert(!sensor_status_get((sensor_id_t)SENSOR_STATUS_CAPACITY, &s));
    assert(co2_inits == (co2_task_failure ? 0U : 1U));
    assert(co2_handoffs == co2_ok);
    if (co2_task_failure || co2_init_failure) {
        assert(!co2_reads && s.state == SENSOR_UNAVAILABLE);
        assert(co2_task_failure || tasks[5].finished);
    } else {
        assert(!tasks[5].finished && co2_starts[0] == 60000000);
        assert(co2_reads == (co2_slow ? 12U : 13U));
        for (unsigned i = 1; i < co2_reads; ++i) assert(co2_starts[i] - co2_starts[i-1] >= 5000000);
        assert(s.read_successes == co2_ok && s.read_failures == co2_failed);
        if (co2_missing) assert(s.state == SENSOR_UNAVAILABLE && !s.initialized && s.consecutive_failures == 13);
        else assert(s.state == SENSOR_READY && s.initialized && !s.consecutive_failures);
        if (co2_recovery) assert(co2_failed == 3 && co2_ok == 10);
        if (co2_slow) assert(co2_starts[1] == 71500000);
    }
    printf("PASS: 125-second six-sensor/config isolation %s\n", scenario);
}
