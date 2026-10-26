#include "mhz_platform.h"
#include "mhz1911a_sensor.h"
#include "sensor_status.h"
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "../../main/mhz1911a_reader.c"

static jmp_buf stopped;
static const char *scenario;
static int64_t base, stop_at = 125000000, starts[32];
static unsigned creates, inits, reads, valid, failed, timeouts, crc;
static unsigned handoffs;
static int64_t last_handoff = -1;
static bool wake_delayed;
static void (*entry)(void *);
static bool is(const char *name) { return !strcmp(scenario, name); }
static sensor_status_t status(void)
{
    sensor_status_t s; assert(sensor_status_get(SENSOR_MHZ1911A, &s)); return s;
}
void vTaskDelay(TickType_t ticks)
{
    assert(ticks && ticks <= 6000);
    if (!reads) {
        sensor_status_t s = status();
        assert(s.state == SENSOR_UNINITIALIZED && !s.initialized && !s.read_successes && !s.read_failures);
    }
    int64_t next = test_now_us + ticks * 10000LL;
    if (is("late_wake") && reads == 1 && !wake_delayed) { next += 7000000; wake_delayed = true; }
    if (next >= base + stop_at) { test_now_us = base + stop_at; longjmp(stopped, 1); }
    test_now_us = next;
}
void vTaskDelete(TaskHandle_t task) { assert(!task); longjmp(stopped, 2); }
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task) { assert(!task); return 1232; }
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, unsigned stack, void *arg, unsigned priority, TaskHandle_t *handle)
{
    assert(!strcmp(name, "mhz1911a_reader") && stack == 4096 && priority == 3 && !arg && handle);
    ++creates; entry = fn;
    if (is("task")) return pdFALSE;
    *handle = &creates; return pdPASS;
}
BaseType_t xTaskNotifyWait(uint32_t entry_bits, uint32_t exit_bits, uint32_t *value, TickType_t wait)
{
    assert(!entry_bits && exit_bits == UINT32_MAX);
    if (wait) vTaskDelay(wait);
    return pdFALSE;
}
void mqtt_manager_commit_mhz1911a_config(bool enabled) { assert(false); }
esp_err_t mqtt_manager_publish_config_ack(const mqtt_config_update_t *update) { assert(false); return ESP_FAIL; }
esp_err_t mqtt_manager_reject_config(const mqtt_config_update_t *update, const char *reason) { assert(false); return ESP_FAIL; }
esp_err_t mhz1911a_sensor_init(void) { ++inits; return is("init") ? ESP_ERR_NO_MEM : ESP_OK; }
esp_err_t mhz1911a_sensor_read(mhz1911a_reading_t *out)
{
    assert(test_now_us >= base + 60000000 && reads < 32 && inits == 1);
    sensor_status_t s = status();
    assert(s.read_successes == valid && s.read_failures == failed);
    if (!reads) assert(s.state == SENSOR_UNINITIALIZED && !s.initialized);
    if (is("recovery") && reads == 4) assert(s.state == SENSOR_UNAVAILABLE && s.consecutive_failures == 3);
    if (is("startup_recovery") && reads == 2) assert(s.state == SENSOR_UNAVAILABLE && !s.initialized);
    starts[reads++] = test_now_us - base;
    bool fail = is("missing") || (is("startup_recovery") && reads <= 2) ||
                ((is("degraded") || is("unavailable") || is("recovery")) && reads >= 2 && reads <= 4) ||
                (is("checksum") && reads == 1);
    esp_err_t err = fail ? is("checksum") ? ESP_ERR_INVALID_CRC : ESP_ERR_TIMEOUT : ESP_OK;
    test_now_us += err == ESP_ERR_TIMEOUT ? 2000000 : 30000;
    if (is("overrun") && reads == 1) test_now_us += 6500000;
    if (err == ESP_OK) { out->co2_ppm = 1770; ++valid; }
    else { ++failed; timeouts += err == ESP_ERR_TIMEOUT; crc += err == ESP_ERR_INVALID_CRC; }
    return err;
}
void mhz1911a_sensor_get_diagnostics(mhz1911a_diagnostics_t *out)
{ *out = (mhz1911a_diagnostics_t){.requests=reads, .valid_frames=valid, .timeouts=timeouts, .checksum_failures=crc}; }

esp_err_t mqtt_manager_update_mhz1911a_snapshot(uint16_t ppm, int64_t acquired_us)
{
    assert(ppm == 1770 && acquired_us == test_now_us && acquired_us > last_handoff);
    assert(test_now_us >= base + 60000000 && status().state == SENSOR_READY);
    assert(++handoffs == valid); // Exactly one handoff per success, none per error.
    last_handoff = acquired_us;
    return is("mqtt_failed") ? ESP_ERR_INVALID_STATE : ESP_OK;
}

static void probe_guards(void)
{
    // The opt-in probe path cannot revive failed init/tasks or disabled slots.
    sensor_id_t spare = (sensor_id_t)6;
    sensor_status_t s;
    sensor_status_configure(spare, "probe", false);
    sensor_status_record_probe(spare, ESP_OK, 0, 3);
    assert(sensor_status_get(spare, &s) && s.state == SENSOR_DISABLED && !s.read_successes);
    sensor_status_configure(spare, "probe", true);
    sensor_status_record_init(spare, ESP_FAIL);
    assert(sensor_status_set_probe_enabled(spare, false) == ESP_ERR_INVALID_STATE);
    assert(sensor_status_set_probe_enabled(spare, true) == ESP_ERR_INVALID_STATE);
    sensor_status_record_probe(spare, ESP_OK, 0, 3);
    assert(sensor_status_get(spare, &s) && s.state == SENSOR_UNAVAILABLE && !s.read_successes);
    sensor_status_configure(spare, "probe", true);
    sensor_status_record_unavailable(spare, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
    assert(sensor_status_set_probe_enabled(spare, true) == ESP_ERR_INVALID_STATE);
    sensor_status_record_probe(spare, ESP_OK, 0, 3);
    assert(sensor_status_get(spare, &s) && s.error_stage == SENSOR_STAGE_TASK && !s.read_successes);
    sensor_status_configure(spare, "probe", true);
    sensor_status_record_probe(spare, ESP_OK, 0, 0);
    assert(sensor_status_get(spare, &s) && s.state == SENSOR_UNINITIALIZED);
    sensor_status_record_probe((sensor_id_t)-1, ESP_OK, 0, 3);
    assert(sensor_status_set_probe_enabled((sensor_id_t)-1, true) == ESP_ERR_INVALID_ARG);
    assert(sensor_status_set_probe_enabled((sensor_id_t)7, true) == ESP_ERR_INVALID_STATE);
    assert(sensor_status_set_probe_enabled(spare, false) == ESP_OK);
    assert(sensor_status_set_probe_enabled(spare, true) == ESP_OK);
    assert(sensor_status_get(spare, &s) && s.state == SENSOR_UNINITIALIZED && !s.initialized);
}
int main(int argc, char **argv)
{
    assert(argc == 2); scenario = argv[1]; probe_guards();
    if (is("warmup")) stop_at = 59990000;
    if (is("degraded")) stop_at = 75000000;
    if (is("unavailable")) stop_at = 80000000;
    if (is("recovery")) stop_at = 85000000;
    if (is("overrun")) stop_at = 135000000;
    if (is("wrap")) base = (UINT32_MAX - 1000LL) * 10000;
    test_now_us = base;
    mhz1911a_reader_start(); mhz1911a_reader_start();
    assert(creates == 1);
    assert(!mhz1911a_reader_task_handle()); // Init hasn't run; don't notify a potentially dying task.
    if (!setjmp(stopped) && !is("task")) entry(NULL);
    sensor_status_t s = status();
    assert(s.read_successes == valid && s.read_failures == failed);
    assert(handoffs == valid);
    if (is("task") || is("init")) {
        assert(!reads && s.state == SENSOR_UNAVAILABLE && !s.initialized);
        assert(s.error_stage == (is("task") ? SENSOR_STAGE_TASK : SENSOR_STAGE_INIT));
        assert(inits == (is("task") ? 0U : 1U));
        assert(!mhz1911a_reader_task_handle());
    } else if (is("warmup")) {
        assert(!reads && s.state == SENSOR_UNINITIALIZED && !s.initialized);
        assert(!test_error_logs && test_log_calls == 2);
    } else {
        assert(starts[0] == 60000000);
        for (unsigned i = 1; i < reads; ++i) assert(starts[i] - starts[i-1] >= 5000000);
        if (is("degraded")) assert(s.state == SENSOR_DEGRADED && s.initialized && failed == 2);
        else if (is("unavailable")) assert(s.state == SENSOR_UNAVAILABLE && s.initialized && failed == 3 && s.consecutive_failures == 3);
        else if (is("missing")) assert(s.state == SENSOR_UNAVAILABLE && !s.initialized && failed == 13 && s.last_success_us == -1);
        else assert(s.state == SENSOR_READY && s.initialized && !s.consecutive_failures && s.last_success_us >= base + starts[reads-1]);
        if (is("healthy") || is("wrap") || is("mqtt_failed")) {
            assert(reads == 13 && !failed && test_error_logs == 0);
            for (unsigned i = 0; i < reads; ++i) assert(starts[i] == 60000000 + i * 5000000LL);
            assert(test_logs_contain("co2=1770 ppm read=30.0 ms"));
            assert(test_logs_contain("ok_total=13 failed_total=0 consecutive=0 period=5000 ms"));
        }
        if (is("recovery")) assert(valid == 2 && failed == 3 && test_recovery_logs == 1);
        if (is("startup_recovery")) assert(valid == 11 && failed == 2 && test_recovery_logs == 1);
        if (is("checksum")) assert(crc == 1 && test_logs_contain("checksum_failed_total=1"));
        if (is("overrun")) assert(starts[1] == 71530000 && test_logs_contain("overruns_total=1"));
        if (is("late_wake")) assert(starts[1] == 72000000 && starts[2] == 77000000 && test_logs_contain("max_late=7000.0 ms"));
        assert(test_error_logs <= 2 && test_log_calls <= valid + 12);
    }
    mhz1911a_reader_start(); assert(creates == 1);
    assert(!test_logs_contain("hex=") && !test_logs_contain("provisional_experimental"));
    printf("PASS: MH-Z1911A reader/status %s\n", scenario);
    return 0;
}
