#include "platform.h"
#include <assert.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#include "../../main/hcsr04p_reader.c"

static jmp_buf stopped;
static void (*reader)(void *);
static bool init_fail, task_fail, missing, invalid, resume, overrun, jitter, alternating;
static unsigned reads, inits, delays;
static int64_t start_us, stop_us, times[128];

BaseType_t xTaskCreate(void (*fn)(void *), const char *name, unsigned stack,
    void *arg, unsigned priority, TaskHandle_t *handle)
{
    assert(!strcmp(name, "hcsr04p_reader") && stack == 4096 && priority == 3 && !arg && !handle);
    if (task_fail) return pdFALSE;
    reader = fn; return pdPASS;
}
void vTaskDelete(TaskHandle_t task) { assert(!task && init_fail); longjmp(stopped, 1); }
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task) { assert(!task); return 2000; }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks > 0 && ticks <= 20); // Never spin or wait for seconds.
    delays++;
    test_now_us += ticks * 10000LL;
    if (jitter && delays == 1) test_now_us += 70000;
    if (test_now_us >= stop_us) longjmp(stopped, 1);
}
esp_err_t hcsr04p_sensor_init(void) { inits++; return init_fail ? ESP_ERR_NO_MEM : ESP_OK; }
esp_err_t hcsr04p_sensor_read(hcsr04p_reading_t *out)
{
    assert(reads < 128 && inits == 1);
    sensor_status_t s;
    assert(sensor_status_get(SENSOR_HCSR04P, &s) && s.initialized);
    if (!reads) assert(s.last_success_us == -1 && !s.read_successes && !s.read_failures);
    if (resume && reads == 3) assert(s.state == SENSOR_DEGRADED && s.consecutive_failures == 3);
    times[reads++] = test_now_us - start_us;
    test_now_us += overrun && reads == 1 ? 350000 : 40000;
    if (missing || (resume && reads <= 3) || (alternating && reads % 2)) return ESP_ERR_TIMEOUT;
    if (invalid) return ESP_ERR_INVALID_RESPONSE;
    *out = (hcsr04p_reading_t){.distance_cm = 17.15f, .echo_us = 1000};
    return ESP_OK;
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    init_fail = !strcmp(argv[1], "init"); task_fail = !strcmp(argv[1], "task");
    missing = !strcmp(argv[1], "missing"); invalid = !strcmp(argv[1], "invalid");
    resume = !strcmp(argv[1], "resume"); overrun = !strcmp(argv[1], "overrun");
    jitter = !strcmp(argv[1], "jitter"); alternating = !strcmp(argv[1], "alternating");
    start_us = !strcmp(argv[1], "wrap") ? ((int64_t)UINT32_MAX - 30) * 10000 : 0;
    test_now_us = start_us;
    // Stop after the first summary but before a later reading can replace the
    // shared log double's last record, including the shifted overrun schedule.
    stop_us = start_us + (overrun ? 10200000 : 10450000);
    hcsr04p_reader_start();
    if (!setjmp(stopped) && reader) reader(NULL);
    sensor_status_t s;
    assert(sensor_status_get(SENSOR_HCSR04P, &s));
    if (init_fail || task_fail) {
        assert(!reads && s.state == SENSOR_UNAVAILABLE && !s.initialized);
        assert(inits == (task_fail ? 0U : 1U) && test_log_calls == 1);
        assert(s.error_stage == (task_fail ? SENSOR_STAGE_TASK : SENSOR_STAGE_INIT));
    } else {
        assert(reads >= 50 && inits == 1 && delays >= reads - 1);
        // Startup + <=11 DATA + two HEALTH lines + <=4 flapping transitions.
        assert(test_log_calls <= 18 && test_logs_contain("stack_min=2000"));
        assert(test_logs_contain(overrun ? "overruns_total=1" : "overruns_total=0"));
        assert(test_logs_contain(jitter || overrun ? "jitter_late_total=1" : "jitter_late_total=0"));
        for (unsigned i = 1; i < reads; ++i) {
            assert(times[i] - times[i-1] == (i == 1 && overrun ? 550000 : i == 1 && jitter ? 270000 : 200000));
        }
        if (missing || invalid) {
            assert(test_error_logs == 1 && test_recovery_logs == 0);
            assert(s.state == SENSOR_DEGRADED && s.consecutive_failures == reads);
            assert(!s.read_successes && s.last_success_us == -1);
            assert(test_logs_contain(missing ? "invalid_total=0" : "timeouts_total=0"));
        } else if (!alternating) {
            assert(test_error_logs == (resume ? 1U : 0U));
            assert(test_recovery_logs == (resume ? 1U : 0U));
            assert(s.state == SENSOR_READY && !s.consecutive_failures && s.read_successes >= 48);
            assert(s.read_failures == (resume ? 3U : 0U));
        }
    }
    printf("PASS: HC-SR04P real reader %s\n", argv[1]);
    return 0;
}
