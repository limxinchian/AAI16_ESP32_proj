#include "platform.h"
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "../../main/rc522_reader.c"

static jmp_buf stopped;
static const char *scenario;
static void (*created)(void *);
static unsigned polls, init_calls, deleted;
static int64_t times[64], stop_us = 2600000;

esp_err_t spi_bus_manager_init(void) { return !strcmp(scenario, "spi_init") ? ESP_ERR_NO_MEM : ESP_OK; }
esp_err_t rc522_sensor_init(void) { init_calls++; return !strcmp(scenario, "init") ? ESP_ERR_INVALID_RESPONSE : ESP_OK; }
BaseType_t xTaskCreate(void (*entry)(void *), const char *name, unsigned stack,
                      void *arg, unsigned priority, TaskHandle_t *handle)
{
    assert(!strcmp(name, "rc522_reader") && stack == 4096 && priority == 3 && !handle);
    if (!strcmp(scenario, "task")) return pdFALSE;
    created = entry; return pdPASS;
}
void vTaskDelete(TaskHandle_t task) { assert(!task); deleted++; longjmp(stopped, 1); }
TickType_t xTaskGetTickCount(void) { return test_now_us / 10000; }
void vTaskDelayUntil(TickType_t *wake, TickType_t period)
{
    assert(period == pdMS_TO_TICKS(200));
    *wake += period;
    assert(*wake * 10000LL > test_now_us);
    test_now_us = *wake * 10000LL;
    if (test_now_us >= stop_us) longjmp(stopped, 1);
}
esp_err_t rc522_sensor_poll(rc522_uid_t *uid, bool *present)
{
    assert(polls < 64 && sensor_status_can_sample(SENSOR_RC522));
    times[polls] = test_now_us;
    unsigned sample = polls++;
    *present = false;
    if (!strcmp(scenario, "reads")) return ESP_ERR_TIMEOUT;
    if (!strcmp(scenario, "resume") && sample < 2) return ESP_ERR_INVALID_CRC;
    if (!strcmp(scenario, "idle") || (!strcmp(scenario, "remove") && sample >= 3 && sample <= 5) ||
        (!strcmp(scenario, "resume") && sample < 7)) return ESP_OK;
    if (!strcmp(scenario, "overrun") && sample == 0) test_now_us += 700000;
    *present = true;
    *uid = (rc522_uid_t){.length = 10, .bytes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}};
    if (!strcmp(scenario, "change") && sample >= 5) uid->bytes[0] = 0xff;
    return ESP_OK;
}

int main(int argc, char **argv)
{
    assert(argc == 2); scenario = argv[1];
    rc522_reader_start();
    if (created && !setjmp(stopped)) created(NULL);
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_RC522, &status));
    if (!strcmp(scenario, "spi_init") || !strcmp(scenario, "task")) {
        assert(!created && !init_calls && status.state == SENSOR_UNAVAILABLE);
        assert(status.error_stage == (!strcmp(scenario, "spi_init") ? SENSOR_STAGE_DEPENDENCY : SENSOR_STAGE_TASK));
    } else if (!strcmp(scenario, "init")) {
        assert(deleted == 1 && !polls && status.state == SENSOR_UNAVAILABLE && status.error_stage == SENSOR_STAGE_INIT);
    } else {
        assert(init_calls == 1 && !deleted);
        if (!strcmp(scenario, "idle")) {
            assert(status.state == SENSOR_READY && !status.read_failures && !status.read_successes);
            assert(status.last_success_us == -1 && test_log_calls == 1);
        } else if (!strcmp(scenario, "reads")) {
            assert(status.state == SENSOR_DEGRADED && status.read_failures == polls && !status.read_successes);
            assert(test_log_calls == 2); // Startup + one error; not a warning every 200 ms.
        } else {
            assert(status.state == SENSOR_READY && status.last_success_us >= 0);
            unsigned expected_logs = (!strcmp(scenario, "remove") || !strcmp(scenario, "change") ||
                                      !strcmp(scenario, "resume")) ? 3 : 2;
            assert(test_log_calls == expected_logs && strstr(test_last_log, "UID (10 bytes):"));
            assert(status.read_failures == (!strcmp(scenario, "resume") ? 2U : 0U));
            assert(status.consecutive_failures == 0);
        }
        if (!strcmp(scenario, "overrun")) assert(times[0] == 0 && times[1] == 900000 && times[2] == 1100000);
        else for (unsigned i = 0; i < polls; ++i) assert(times[i] == i * 200000LL);
    }
    printf("PASS: RC522 reader scenario %s\n", scenario);
    return 0;
}
