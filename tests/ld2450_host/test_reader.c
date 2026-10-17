#include "platform.h"
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "../../main/ld2450_reader.c"

static jmp_buf stopped;
static const char *scenario;
static void (*created)(void *);
static unsigned receives, valid, inits, deinits, deleted, delays;
static int64_t stop_us=22200000;
static bool is(const char *name) { return !strcmp(scenario,name); }
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, unsigned stack,
                      void *arg, unsigned priority, TaskHandle_t *handle)
{
    assert(!strcmp(name,"ld2450_reader") && stack==4096 && priority==3 && !arg && !handle);
    if (is("task")) return pdFALSE;
    created=fn; return pdPASS;
}
void vTaskDelete(TaskHandle_t task) { assert(!task); deleted++; longjmp(stopped,1); }
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task) { assert(!task); return 2048; }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks==1); delays++; test_now_us+=ticks*10000LL;
    if (test_now_us>=stop_us) longjmp(stopped,1);
}
esp_err_t ld2450_sensor_init(void) { inits++; return is("init") ? ESP_ERR_NO_MEM : ESP_OK; }
esp_err_t ld2450_sensor_deinit(void) { deinits++; return is("cleanup") ? ESP_FAIL : ESP_OK; }
esp_err_t ld2450_sensor_receive(ld2450_report_t *r, unsigned *frames)
{
    receives++;
    // Assert UART setup never prematurely marked the unproven radar healthy.
    sensor_status_t s; assert(sensor_status_get(SENSOR_LD2450,&s));
    if (!valid) assert(s.state==SENSOR_UNINITIALIZED && !s.initialized && s.last_success_us==-1);
    test_now_us+=100000;
    if (is("resume") && test_now_us>5000000 && valid==10) {
        assert(s.state==SENSOR_DEGRADED && s.read_failures>0);
    }
    *frames=0;
    if (is("missing") || is("cleanup")) return ESP_OK;
    if (is("noise")) return ESP_ERR_INVALID_RESPONSE;
    if ((is("silence") || is("resume")) && receives>10 && (!is("resume") || test_now_us<5000000)) return ESP_OK;
    if (is("uart_errors") && receives>10) return ESP_FAIL;
    *r=(ld2450_report_t){0};
    if (!is("idle") && (!is("changes") || receives%2)) {
        r->count=1; r->targets[0]=(ld2450_target_t){true,-782,1713,-16,320};
    }
    *frames=1; valid++; return ESP_OK;
}
void ld2450_sensor_get_diagnostics(ld2450_diagnostics_t *d)
{ *d=(ld2450_diagnostics_t){.frames=valid,.rx_bytes=valid*30,.discarded_bytes=is("noise")?receives:0}; }

int main(int argc,char **argv)
{
    assert(argc==2); scenario=argv[1];
    ld2450_report_t example = {.count=1, .targets={{true,-782,1713,-16,320}}};
    log_report(&example);
    assert(strstr(test_last_log,"x=-782 mm y=1713 mm distance=1883 mm speed=-16 cm/s resolution=320 mm"));
    test_log_calls=0;
    // Radar errors must not change other registry slots or their counters.
    for (sensor_id_t id=SENSOR_DHT22; id<=SENSOR_RC522; ++id) {
        sensor_status_configure(id,"existing",true); sensor_status_record_init(id,ESP_OK);
        sensor_status_record_read(id,ESP_OK,123);
    }
    ld2450_reader_start();
    if (created && !setjmp(stopped)) created(NULL);
    sensor_status_t s; assert(sensor_status_get(SENSOR_LD2450,&s));
    if (is("task")) {
        assert(!inits && !created && s.state==SENSOR_UNAVAILABLE && s.error_stage==SENSOR_STAGE_TASK);
    } else if (is("init")) {
        assert(inits==1 && !receives && !deinits && deleted==1 && s.state==SENSOR_UNAVAILABLE);
        assert(s.error_stage==SENSOR_STAGE_INIT && !s.initialized);
    } else if (is("missing") || is("noise") || is("cleanup")) {
        assert(inits==1 && deinits==1 && deleted==1 && s.state==SENSOR_UNAVAILABLE && !s.initialized);
        assert(test_now_us>=5000000 && test_now_us<5110000 && s.last_success_us==-1 && !s.read_successes);
        assert(s.last_error==ESP_ERR_TIMEOUT && test_log_calls<=5);
    } else {
        assert(inits==1 && !deinits && !deleted && receives==delays && receives>200);
        assert(s.initialized && s.read_successes==valid && s.last_success_us>=0);
        if (is("silence") || is("uart_errors")) {
            assert(s.state==SENSOR_DEGRADED && s.read_failures>0 && s.last_success_us<1200000);
            assert(test_log_calls<12); // Repeated errors do not flood serial.
        } else {
            assert(s.state==SENSOR_READY && !s.consecutive_failures);
            assert(s.read_failures==(is("resume")?1U:0U));
            if (is("changes")) assert(test_log_calls<50); // 10 Hz changes coalesced <=1 group/s.
            else assert(test_log_calls<12); // Stable targets only startup + 10s summaries.
        }
    }
    for (sensor_id_t id=SENSOR_DHT22; id<=SENSOR_RC522; ++id) {
        assert(sensor_status_get(id,&s) && s.state==SENSOR_READY && s.read_successes==1);
        assert(!s.read_failures && s.last_success_us==123);
    }
    printf("PASS: LD2450 reader %s\n",scenario);
}
