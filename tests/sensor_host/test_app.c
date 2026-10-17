#include "platform.h"
#include <assert.h>
#include <setjmp.h>
#include <string.h>

static unsigned applied_logs;
static int test_printf(const char *format, ...)
{
    if (strstr(format, "interval applied")) applied_logs++;
    return 0;
}
// Compile the real application, replacing only platform/driver/network APIs.
#define printf test_printf
#include "../../main/main.c"
#undef printf

static jmp_buf stopped;
static int64_t stop_at_us = 12000000;
static esp_err_t dht_init_error, gy_init_error, bus_init_error;
static bool task_failure, command_flood, read_failures, gy_overrun;
static unsigned wifi_starts, mqtt_starts, bus_starts, scans, gy_inits, gy_deinits;
static unsigned dht_inits, dht_reads, gy_reads, dht_publications, gy_publications;
static unsigned diagnostics_calls, commands_taken, applied_acks, rejected_acks;
static uint32_t ack_interval;
static int64_t command_at_us = -1;
static int64_t dht_times[16], gy_times[16];
static void (*created_reader)(void *);

static void advance_to(int64_t time_us)
{
    assert(time_us > test_now_us); // Detect spin/catch-up loops in the exercised paths.
    test_now_us = time_us;
    if (test_now_us >= stop_at_us) longjmp(stopped, 1);
}
void vTaskDelay(TickType_t ticks) { advance_to(test_now_us + ticks * 10000LL); }
void vTaskDelayUntil(TickType_t *last_wake, TickType_t period)
{
    *last_wake += period;
    advance_to(*last_wake * 10000LL);
}
TickType_t xTaskGetTickCount(void) { return (TickType_t)(test_now_us / 10000); }
BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, eNotifyAction action)
{
    assert(false); // These original regression scenarios only send DHT22 commands.
    return pdFALSE;
}
BaseType_t xTaskNotifyWait(uint32_t clear_entry, uint32_t clear_exit,
                           uint32_t *value, TickType_t wait)
{
    assert(clear_entry == 0 && clear_exit == UINT32_MAX);
    if (wait) vTaskDelay(wait);
    return pdFALSE;
}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task) { return 1024; }
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, unsigned stack,
                      void *arg, unsigned priority, TaskHandle_t *handle)
{
    assert(!strcmp(name, "gy511_reader") && stack == 4096 && priority == 4);
    assert(handle && !arg);
    if (task_failure) return pdFALSE;
    *handle = (TaskHandle_t)1;
    created_reader = fn;
    return pdPASS;
}
esp_err_t wifi_manager_start(void) { wifi_starts++; return ESP_OK; }
esp_err_t mqtt_manager_init(void) { mqtt_starts++; return ESP_OK; }
esp_err_t i2c_bus_manager_init(void) { bus_starts++; return bus_init_error; }
void i2c_bus_manager_scan(void) { scans++; }
esp_err_t gy511_sensor_init(void) { gy_inits++; return gy_init_error; }
esp_err_t gy511_sensor_deinit(void) { gy_deinits++; return ESP_OK; }
esp_err_t dht22_init(gpio_num_t pin) { assert(pin == GPIO_NUM_6); dht_inits++; return dht_init_error; }
esp_err_t dht22_read(dht22_data_t *data)
{
    assert(dht_init_error == ESP_OK && dht_reads < 16);
    dht_times[dht_reads++] = test_now_us;
    if (read_failures && dht_reads <= 2) return ESP_ERR_INVALID_CRC;
    data->temperature_c = 25; data->humidity_pct = 50;
    return ESP_OK;
}
esp_err_t gy511_sensor_read(gy511_data_t *data)
{
    assert(gy_reads < 16);
    gy_times[gy_reads++] = test_now_us;
    if (gy_overrun && gy_reads == 1) test_now_us += 700000;
    if (read_failures && (gy_reads <= 2 || gy_reads == 5)) return ESP_ERR_TIMEOUT;
    *data = (gy511_data_t){.accel_x_g = 1, .mag_x_ut = 10};
    return ESP_OK;
}
esp_err_t mqtt_manager_publish_dht22(float temperature, float humidity)
{
    assert(temperature == 25 && humidity == 50); dht_publications++; return ESP_OK;
}
esp_err_t mqtt_manager_publish_gy511(float ax, float ay, float az, float mx, float my, float mz)
{
    assert(ax == 1 && mx == 10); gy_publications++; return ESP_OK;
}
bool mqtt_manager_peek_interval_update(mqtt_interval_update_t *update)
{
    if (command_flood || (command_at_us >= 0 && test_now_us >= command_at_us && !commands_taken)) {
        *update = (mqtt_interval_update_t){SENSOR_DHT22, 5000}; return true;
    }
    return false;
}
bool mqtt_manager_take_interval_update(mqtt_interval_update_t *update)
{
    if (!mqtt_manager_peek_interval_update(update)) return false;
    commands_taken++; return true;
}
esp_err_t mqtt_manager_publish_config_ack(sensor_id_t sensor, uint32_t interval)
{
    assert(sensor == SENSOR_DHT22);
    applied_acks++; ack_interval = interval; return ESP_OK;
}
esp_err_t mqtt_manager_reject_config(const mqtt_interval_update_t *update, const char *reason)
{
    assert(update->sensor == SENSOR_DHT22 && update->interval_ms == 5000);
    assert(!strcmp(reason, "sensor_unavailable"));
    rejected_acks++; return ESP_OK;
}
void mqtt_manager_log_diagnostics(void) { diagnostics_calls++; }

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *scenario = argv[1];
    bool worker_only = !strcmp(scenario, "gy_reads") || !strcmp(scenario, "gy_overrun");
    if (!strcmp(scenario, "dht_init") || !strcmp(scenario, "both_init") ||
        !strcmp(scenario, "unavailable_flood")) {
        dht_init_error = ESP_ERR_NO_MEM;
        command_at_us = 2500000;
        stop_at_us = 125000000; // Cross two reporting deadlines while unavailable.
    }
    if (!strcmp(scenario, "gy_init") || !strcmp(scenario, "both_init")) gy_init_error = ESP_FAIL;
    if (!strcmp(scenario, "bus_init")) bus_init_error = ESP_ERR_NO_MEM;
    if (!strcmp(scenario, "gy_task")) task_failure = true;
    if (!strcmp(scenario, "unavailable_flood") || !strcmp(scenario, "healthy_flood")) command_flood = true;
    if (!strcmp(scenario, "dht_reads") || !strcmp(scenario, "gy_reads")) read_failures = true;
    if (!strcmp(scenario, "healthy_config")) { command_at_us = 2500000; stop_at_us = 13000000; }
    if (!strcmp(scenario, "gy_overrun")) gy_overrun = true;

    if (worker_only) {
        sensor_status_configure(SENSOR_GY511, "gy511", true);
        sensor_status_record_init(SENSOR_GY511, ESP_OK);
        stop_at_us = 2200000;
    }
    if (!setjmp(stopped)) {
        if (worker_only) gy511_reader_task(NULL);
        else app_main();
        assert(false); // No sensor error may return from app_main.
    }

    sensor_status_t dht, gy;
    assert(sensor_status_get(SENSOR_GY511, &gy));
    if (worker_only) {
        if (gy_overrun) {
            assert(gy_times[0] == 100000 && gy_times[1] == 1300000 && gy_times[2] == 1800000);
            assert(gy_reads == 3 && gy_publications == 3);
        } else {
            assert(gy_reads == 5 && gy_publications == 2);
            for (unsigned i = 0; i < gy_reads; ++i) assert(gy_times[i] == 100000 + i * 500000LL);
            assert(gy.state == SENSOR_DEGRADED && gy.error_stage == SENSOR_STAGE_READ);
            assert(gy.read_failures == 3 && gy.read_successes == 2 && gy.consecutive_failures == 1);
            assert(gy.last_success_us == 1600000);
        }
    } else {
        assert(wifi_starts == 1 && mqtt_starts == 1 && bus_starts == 1 && dht_inits == 1);
        assert(sensor_status_get(SENSOR_DHT22, &dht));
        assert(gy_inits == (bus_init_error == ESP_OK ? 1U : 0U));
        assert(scans == gy_inits);
        if (bus_init_error || gy_init_error || task_failure) {
            assert(!created_reader && gy.state == SENSOR_UNAVAILABLE);
            assert(gy.error_stage == (bus_init_error ? SENSOR_STAGE_DEPENDENCY :
                                     gy_init_error ? SENSOR_STAGE_INIT : SENSOR_STAGE_TASK));
            assert(gy.initialized == task_failure && gy.read_successes == 0 && gy.read_failures == 0);
            assert(gy_deinits == (task_failure ? 1U : 0U));
        } else {
            assert(created_reader == gy511_reader_task && gy.state == SENSOR_READY);
        }
        if (dht_init_error) {
            assert(!dht_reads && !dht_publications && !applied_acks && !applied_logs);
            assert(dht.state == SENSOR_UNAVAILABLE && dht.error_stage == SENSOR_STAGE_INIT);
            assert(!dht.initialized && !dht.read_failures && dht.last_success_us == -1);
            assert(diagnostics_calls == 1230 && test_log_calls == 6);
            assert(commands_taken == (command_flood ? 4 * diagnostics_calls : 1));
            assert(rejected_acks == commands_taken);
        } else {
            assert(dht.state == SENSOR_READY && dht.error_stage == SENSOR_STAGE_NONE);
            assert(diagnostics_calls == dht_reads && dht.read_successes == dht_publications);
            assert(dht.read_failures == (read_failures ? 2U : 0U));
            assert(dht.consecutive_failures == 0 && !rejected_acks);
            if (command_at_us >= 0) {
                assert(applied_acks == 1 && ack_interval == 5000 && applied_logs == 1);
                assert(dht_reads == 3 && dht_times[0] == 2000000 &&
                       dht_times[1] == 7000000 && dht_times[2] == 12000000);
            } else if (command_flood) {
                assert(applied_acks == commands_taken && dht_reads == 1 && dht_times[0] == 7000000);
            } else {
                assert(dht_reads == 5 && !applied_acks);
                for (unsigned i = 0; i < dht_reads; ++i) assert(dht_times[i] == 2000000 + i * 2000000LL);
            }
        }
    }
    printf("PASS: real application scenario %s\n", scenario);
    return 0;
}
