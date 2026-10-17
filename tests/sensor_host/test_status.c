#include "platform.h"
#include "sensor_status.h"
#include <assert.h>
#include <string.h>

int main(void)
{
    sensor_status_t status;
    assert(!sensor_status_get(SENSOR_DHT22, &status));
    assert(!sensor_status_can_sample(SENSOR_DHT22));
    assert(sensor_status_set_enabled(SENSOR_DHT22, true) == ESP_ERR_INVALID_STATE);
    assert(sensor_status_set_enabled((sensor_id_t)-1, false) == ESP_ERR_INVALID_ARG);
    assert(sensor_status_configure((sensor_id_t)-1, "bad", true) == ESP_ERR_INVALID_ARG);
    assert(sensor_status_configure(SENSOR_STATUS_CAPACITY, "bad", true) == ESP_ERR_INVALID_ARG);
    assert(sensor_status_configure(SENSOR_DHT22, NULL, true) == ESP_ERR_INVALID_ARG);
    assert(!sensor_status_get(SENSOR_DHT22, NULL));

    assert(sensor_status_configure(SENSOR_DHT22, "dht22", true) == ESP_OK);
    assert(sensor_status_get(SENSOR_DHT22, &status));
    assert(status.state == SENSOR_UNINITIALIZED && !status.initialized);
    assert(status.last_success_us == -1);
    sensor_status_record_read(SENSOR_DHT22, ESP_OK, 0); // Cannot read before init.
    sensor_status_record_init(SENSOR_DHT22, ESP_ERR_NO_MEM);
    assert(sensor_status_get(SENSOR_DHT22, &status));
    assert(status.state == SENSOR_UNAVAILABLE && status.error_stage == SENSOR_STAGE_INIT);
    assert(status.last_error == ESP_ERR_NO_MEM && !status.initialized);
    assert(status.read_failures == 0 && status.read_successes == 0);
    sensor_status_record_init(SENSOR_DHT22, ESP_OK); // No implicit recovery.
    assert(!sensor_status_can_sample(SENSOR_DHT22));
    assert(sensor_status_set_enabled(SENSOR_DHT22, true) == ESP_ERR_INVALID_STATE);
    assert(sensor_status_set_enabled(SENSOR_DHT22, false) == ESP_ERR_INVALID_STATE);
    assert(sensor_status_get(SENSOR_DHT22, &status) && status.state == SENSOR_UNAVAILABLE);

    assert(sensor_status_configure(SENSOR_GY511, "gy511", false) == ESP_OK);
    sensor_status_record_init(SENSOR_GY511, ESP_OK);
    sensor_status_record_unavailable(SENSOR_GY511, SENSOR_STAGE_DEPENDENCY, ESP_FAIL);
    sensor_status_record_read(SENSOR_GY511, ESP_FAIL, 100);
    assert(sensor_status_get(SENSOR_GY511, &status));
    assert(status.state == SENSOR_DISABLED && !status.initialized);
    assert(status.error_stage == SENSOR_STAGE_NONE && status.read_failures == 0);
    assert(!sensor_status_can_sample(SENSOR_GY511));
    assert(sensor_status_set_enabled(SENSOR_GY511, true) == ESP_ERR_INVALID_STATE);

    // Independent enabled slot, including valid success at time zero.
    sensor_id_t other = (sensor_id_t)2;
    assert(sensor_status_configure(other, "other", true) == ESP_OK);
    sensor_status_record_init(other, ESP_OK);
    sensor_status_record_read(other, ESP_OK, 0);
    sensor_status_record_read(other, ESP_ERR_TIMEOUT, 100);
    sensor_status_record_read(other, ESP_ERR_INVALID_CRC, 200);
    assert(sensor_status_get(other, &status));
    assert(status.state == SENSOR_DEGRADED && status.initialized);
    assert(status.error_stage == SENSOR_STAGE_READ && status.last_error == ESP_ERR_INVALID_CRC);
    assert(status.read_successes == 1 && status.read_failures == 2);
    assert(status.consecutive_failures == 2 && status.last_success_us == 0);
    assert(sensor_status_can_sample(other));
    assert(sensor_status_set_enabled(other, false) == ESP_OK);
    assert(sensor_status_set_enabled(other, false) == ESP_OK); // Idempotent pause.
    sensor_status_record_read(other, ESP_OK, 250); // No fake success while disabled.
    assert(sensor_status_get(other, &status) && status.state == SENSOR_DISABLED);
    assert(!sensor_status_can_sample(other));
    assert(status.read_successes == 1 && status.read_failures == 2 && status.last_success_us == 0);
    assert(status.consecutive_failures == 2 && status.last_error == ESP_ERR_INVALID_CRC);
    assert(sensor_status_set_enabled(other, true) == ESP_OK);
    assert(sensor_status_set_enabled(other, true) == ESP_OK);
    assert(sensor_status_get(other, &status) && status.state == SENSOR_DEGRADED);
    assert(status.error_stage == SENSOR_STAGE_READ && status.consecutive_failures == 2);
    sensor_status_record_read(other, ESP_OK, 300);
    assert(sensor_status_get(other, &status));
    assert(status.state == SENSOR_READY && status.error_stage == SENSOR_STAGE_NONE);
    assert(status.last_error == ESP_OK && status.consecutive_failures == 0);
    assert(status.read_successes == 2 && status.read_failures == 2 && status.last_success_us == 300);
    assert(sensor_status_set_enabled(other, false) == ESP_OK);
    assert(sensor_status_set_enabled(other, true) == ESP_OK);
    assert(sensor_status_get(other, &status) && status.state == SENSOR_READY);
    assert(status.read_successes == 2 && status.read_failures == 2 && status.last_success_us == 300);
    assert(sensor_status_get(SENSOR_DHT22, &status) && status.state == SENSOR_UNAVAILABLE);

    sensor_status_record_unavailable(other, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
    sensor_status_record_read(other, ESP_OK, 400); // Cannot revive an unavailable task.
    assert(sensor_status_get(other, &status));
    assert(status.state == SENSOR_UNAVAILABLE && status.initialized);
    assert(status.error_stage == SENSOR_STAGE_TASK && status.last_success_us == 300);
    assert(sensor_status_set_enabled(other, true) == ESP_ERR_INVALID_STATE);
    assert(sensor_status_set_enabled(other, false) == ESP_ERR_INVALID_STATE);
    assert(sensor_status_get(other, &status) && status.state == SENSOR_UNAVAILABLE);

    for (int id = 3; id < SENSOR_STATUS_CAPACITY; ++id) {
        assert(sensor_status_configure((sensor_id_t)id, "future", false) == ESP_OK);
    }
    sensor_status_log_summary();
    assert(test_log_calls == 8 && strstr(test_last_log, "last_ok_ms=never"));
    for (test_now_us = 100000; test_now_us < 60000000; test_now_us += 100000) {
        sensor_status_log_summary();
    }
    assert(test_log_calls == 8);
    sensor_status_log_summary();
    assert(test_log_calls == 16);
    puts("PASS: eight status slots, disabled/unavailable, init/read/task transitions, timestamp, bounded reporting");
    return 0;
}
