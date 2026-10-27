#include "platform.h"
#include "sensor_status.h"
#include <assert.h>
#include <string.h>

int main(void)
{
    const char *names[] = {"dht22", "gy511", "rc522", "ld2450", "hcsr04p", "mhz1911a", "bme280", "max6675"};
    for (sensor_id_t id = SENSOR_DHT22; id < SENSOR_STATUS_CAPACITY; ++id) {
        sensor_status_configure(id, names[id], true);
        sensor_status_record_init(id, ESP_OK);
        unsigned errors = test_error_logs, recoveries = test_recovery_logs;
        sensor_status_record_read(id, ESP_ERR_TIMEOUT, 0);
        assert(test_error_logs == errors + 1 && test_last_level == 'W');
        assert(strstr(test_last_log, names[id]) && strstr(test_last_log, "| ERROR |"));
        // A disconnected sensor continues acquisition and counting indefinitely.
        for (int n = 1; n < 30; ++n) sensor_status_record_read(id, ESP_ERR_TIMEOUT, n * 2000000LL);
        assert(test_error_logs == errors + 1);
        sensor_status_t s;
        assert(sensor_status_get(id, &s) && s.state == SENSOR_DEGRADED);
        assert(s.read_failures == 30 && s.consecutive_failures == 30 && s.last_success_us == -1);
        // A different persistent error and recovery each produce one record.
        sensor_status_record_read(id, ESP_ERR_INVALID_RESPONSE, 60000000);
        assert(test_error_logs == errors + 2);
        sensor_status_record_read(id, ESP_OK, 60000001);
        assert(test_recovery_logs == recoveries + 1 && test_last_level == 'I');
        assert(strstr(test_last_log, "recovery=READY"));
        assert(sensor_status_get(id, &s) && s.state == SENSOR_READY);
        assert(s.read_failures == 31 && s.consecutive_failures == 0);
        // Fast flapping must not flood the console or lose any real counters.
        for (int n = 0; n < 100; ++n) {
            sensor_status_record_read(id, n % 2 ? ESP_OK : ESP_ERR_TIMEOUT, 61000000 + n * 1000);
        }
        assert(test_error_logs == errors + 2 && test_recovery_logs == recoveries + 1);
        assert(sensor_status_get(id, &s) && s.read_failures == 81 && s.read_successes == 51);
        sensor_status_record_read(id, ESP_ERR_TIMEOUT, 70000000);
        sensor_status_record_read(id, ESP_OK, 70000001);
        assert(test_error_logs == errors + 3 && test_recovery_logs == recoveries + 2);
        // Disable/enable retains history and cannot manufacture a recovery.
        sensor_status_set_enabled(id, false);
        sensor_status_record_read(id, ESP_ERR_TIMEOUT, 80000000);
        assert(sensor_status_get(id, &s) && s.state == SENSOR_DISABLED && s.read_failures == 82);
        sensor_status_set_enabled(id, true);
        assert(sensor_status_get(id, &s) && s.state == SENSOR_READY && s.read_failures == 82);
    }
    // A transition coalesced during flapping must be reported once it persists.
    sensor_status_record_read(SENSOR_DHT22, ESP_ERR_TIMEOUT, 70000002);
    unsigned errors = test_error_logs;
    sensor_status_record_read(SENSOR_DHT22, ESP_ERR_TIMEOUT, 80000000);
    assert(test_error_logs == errors + 1);
    test_now_us = 80000000;
    sensor_status_log_summary();
    assert(test_summary_logs == 1 + 2 * SENSOR_STATUS_CAPACITY);
    assert(test_logs_contain("HEALTH SUMMARY | counters=cumulative"));
    assert(test_logs_contain("state=READY ok=52 failed=82 consecutive=0"));
    assert(test_logs_contain("transitions_coalesced_total=100"));
    sensor_status_log_summary();
    assert(test_summary_logs == 1 + 2 * SENSOR_STATUS_CAPACITY);
    test_now_us += 60000000;
    sensor_status_log_summary();
    assert(test_summary_logs == 2 * (1 + 2 * SENSOR_STATUS_CAPACITY));
    puts("PASS: eight-sensor error/recovery severity, persistent faults, bounded flapping, counters and health cadence");
}
