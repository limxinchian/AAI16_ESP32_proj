#include "platform.h"
#include <assert.h>
#include <math.h>
#include <string.h>
// Unlike the shared registry double, esp_timer permits reads inside a short
// critical section/ISR on target. Keep this exception local to the driver.
static int64_t capture_time(void) { return test_now_us; }
#define esp_timer_get_time capture_time
#include "../../components/hcsr04p_sensor/hcsr04p_sensor.c"
#undef esp_timer_get_time

static unsigned step, fail_step, triggers, spins, waits, sends;
static bool queued, high_echo, high_trig, cleanup_fail, trigger_fail;
static bool queue_live, timer_live, channel_live, timer_enabled, timer_started, channel_enabled;
static int64_t pretrigger_delay_us;
static uint32_t resolution = 80000000, pulse_ticks = 80000, rise_ticks = 1234;
static echo_event_t queued_event;
static mcpwm_capture_event_callbacks_t callbacks;
typedef enum { VALID, MISSING, RISE_ONLY, LATE, FALL_ONLY, DOUBLE_RISE } mode_t;
static mode_t mode;
static esp_err_t init_step(void) { return ++step == fail_step ? ESP_FAIL : ESP_OK; }

QueueHandle_t xQueueCreate(unsigned count, unsigned size)
{
    assert(count == 1 && size == sizeof(echo_event_t) && !queue_live);
    if (init_step() != ESP_OK) return NULL;
    queue_live = true; return (void *)1;
}
void vQueueDelete(QueueHandle_t queue) { assert(queue_live && !channel_live); queue_live = false; }
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *event, BaseType_t *wake)
{
    assert(queue == (void *)1 && queue_live && !queued);
    queued_event = *(const echo_event_t *)event; queued = true; sends++; *wake = pdTRUE;
    return pdTRUE;
}
static void edge(int type, uint32_t ticks)
{
    mcpwm_capture_event_data_t event = {.cap_edge = type, .cap_value = ticks};
    callbacks.on_cap(s_channel, &event, NULL);
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait)
{
    assert(queue == (void *)1 && queue_live && (wait == 0 || wait == 4));
    if (!wait && pretrigger_delay_us) {
        test_now_us += pretrigger_delay_us;
        pretrigger_delay_us = 0;
    }
    if (wait && !queued) {
        waits++;
        int64_t wait_start = test_now_us;
        if (mode != MISSING) {
            test_now_us = s_last_trigger_us + 200;
            edge(mode == FALL_ONLY ? MCPWM_CAP_EDGE_NEG : MCPWM_CAP_EDGE_POS, rise_ticks);
            if (mode == DOUBLE_RISE) edge(MCPWM_CAP_EDGE_POS, rise_ticks + 100);
            if (mode == VALID || mode == LATE) {
                test_now_us += mode == LATE ? 31000 : (int64_t)pulse_ticks * 1000000 / resolution;
                edge(MCPWM_CAP_EDGE_NEG, rise_ticks + pulse_ticks);
                // Extra edges cannot overwrite the first completed result.
                edge(MCPWM_CAP_EDGE_POS, 7);
                edge(MCPWM_CAP_EDGE_NEG, 8);
            }
        }
        if (!queued) test_now_us = wait_start + wait * 10000;
    }
    if (!queued) return pdFALSE;
    *(echo_event_t *)out = queued_event; queued = false; return pdTRUE;
}
esp_err_t gpio_config(const gpio_config_t *c)
{ assert(c->pin_bit_mask == (1ULL << 4) && c->mode == GPIO_MODE_OUTPUT); return init_step(); }
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level)
{
    assert(pin == 4);
    if (!s_ready) return init_step();
    if (trigger_fail && level) return ESP_FAIL;
    if (level) { assert(!high_trig); triggers++; }
    high_trig = level != 0;
    return ESP_OK;
}
void esp_rom_delay_us(uint32_t us) { assert(us == 12); spins++; test_now_us += us; }
int gpio_get_level(gpio_num_t pin) { assert(pin == 5); return high_echo; }
esp_err_t gpio_reset_pin(gpio_num_t pin) { assert(pin == 4); return ESP_OK; }
esp_err_t mcpwm_new_capture_timer(const mcpwm_capture_timer_config_t *c, mcpwm_cap_timer_handle_t *out)
{
    assert(c->group_id == 0 && c->clk_src == MCPWM_CAPTURE_CLK_SRC_APB && !c->resolution_hz);
    if (init_step() != ESP_OK) return ESP_FAIL;
    timer_live = true; *out = (void *)2; return ESP_OK;
}
esp_err_t mcpwm_new_capture_channel(mcpwm_cap_timer_handle_t timer,
    const mcpwm_capture_channel_config_t *c, mcpwm_cap_channel_handle_t *out)
{
    assert(timer_live && timer == (void *)2 && c->gpio_num == 5 && c->prescale == 1);
    assert(c->flags.pos_edge && c->flags.neg_edge && c->flags.pull_down);
    if (init_step() != ESP_OK) return ESP_FAIL;
    channel_live = true; *out = (void *)3; return ESP_OK;
}
esp_err_t mcpwm_capture_channel_register_event_callbacks(mcpwm_cap_channel_handle_t channel,
    const mcpwm_capture_event_callbacks_t *c, void *context)
{ assert(channel_live && channel == (void *)3 && !context); callbacks = *c; return init_step(); }
esp_err_t mcpwm_capture_timer_get_resolution(mcpwm_cap_timer_handle_t timer, uint32_t *hz)
{ assert(timer_live); *hz = resolution; return init_step(); }
esp_err_t mcpwm_capture_timer_enable(mcpwm_cap_timer_handle_t timer)
{ if (init_step() != ESP_OK) return ESP_FAIL; timer_enabled = true; return ESP_OK; }
esp_err_t mcpwm_capture_timer_start(mcpwm_cap_timer_handle_t timer)
{ assert(timer_enabled); if (init_step() != ESP_OK) return ESP_FAIL; timer_started = true; return ESP_OK; }
esp_err_t mcpwm_capture_channel_enable(mcpwm_cap_channel_handle_t channel)
{ assert(timer_started); if (init_step() != ESP_OK) return ESP_FAIL; channel_enabled = true; return ESP_OK; }
esp_err_t mcpwm_capture_channel_disable(mcpwm_cap_channel_handle_t channel)
{ assert(channel_enabled); channel_enabled = false; return ESP_OK; }
esp_err_t mcpwm_capture_timer_stop(mcpwm_cap_timer_handle_t timer)
{ assert(timer_started); if (cleanup_fail) return ESP_FAIL; timer_started = false; return ESP_OK; }
esp_err_t mcpwm_capture_timer_disable(mcpwm_cap_timer_handle_t timer)
{ assert(timer_enabled && !timer_started); timer_enabled = false; return ESP_OK; }
esp_err_t mcpwm_del_capture_channel(mcpwm_cap_channel_handle_t channel)
{ assert(channel_live && !channel_enabled); channel_live = false; return ESP_OK; }
esp_err_t mcpwm_del_capture_timer(mcpwm_cap_timer_handle_t timer)
{ assert(timer_live && !channel_live && !timer_enabled); timer_live = false; return ESP_OK; }

static void reset(void)
{
    cleanup_fail = false;
    assert(release_resources() == ESP_OK);
    s_attempted = s_ready = s_armed = s_rising = false;
    s_last_trigger_us = -1;
    step = fail_step = triggers = spins = waits = sends = 0;
    queued = high_echo = high_trig = trigger_fail = false;
    test_now_us = 1000; resolution = 80000000; mode = VALID;
    pulse_ticks = 80000; rise_ticks = 1234;
}
static void read_case(mode_t input, uint32_t ticks, esp_err_t expected)
{
    mode = input; pulse_ticks = ticks; test_now_us += 200000;
    hcsr04p_reading_t out = {.distance_cm = 999, .echo_us = 999};
    int64_t start = test_now_us;
    unsigned old_sends = sends, old_spins = spins;
    assert(hcsr04p_sensor_read(&out) == expected);
    assert(!s_armed && !queued && !high_trig);
    assert(test_now_us - start <= 40012 && spins == old_spins + 1);
    assert(sends - old_sends <= 1);
    if (expected == ESP_OK) {
        assert(fabs(out.echo_us - (double)ticks * 1000000 / resolution) < 0.01);
        assert(fabsf(out.distance_cm - out.echo_us * 0.01715f) < 0.001);
    } else assert(out.echo_us == 0 && out.distance_cm == 0);
}
int main(void)
{
    hcsr04p_reading_t out;
    assert(hcsr04p_sensor_read(NULL) == ESP_ERR_INVALID_ARG);
    assert(hcsr04p_sensor_read(&out) == ESP_ERR_INVALID_STATE);
    // Fail every allocation/configuration/enable step and verify no leaked
    // resources, no trigger, no reads and no implicit initialization retry.
    for (unsigned i = 1; i <= 10; ++i) {
        reset(); fail_step = i;
        assert(hcsr04p_sensor_init() != ESP_OK);
        assert(!s_ready && !queue_live && !timer_live && !channel_live && !s_gpio_owned);
        assert(!timer_enabled && !timer_started && !channel_enabled && !triggers);
        unsigned calls = step;
        assert(hcsr04p_sensor_init() == ESP_ERR_INVALID_STATE && step == calls);
        assert(hcsr04p_sensor_read(&out) == ESP_ERR_INVALID_STATE);
    }
    reset(); resolution = 0;
    assert(hcsr04p_sensor_init() == ESP_ERR_INVALID_STATE && !queue_live && !timer_live);
    reset(); fail_step = 10; cleanup_fail = true;
    assert(hcsr04p_sensor_init() == ESP_FAIL && queue_live && timer_live && channel_live);
    assert(hcsr04p_sensor_init() == ESP_ERR_INVALID_STATE); // Retain unsafe-to-free resources.
    reset(); assert(hcsr04p_sensor_init() == ESP_OK);
    unsigned calls = step;
    assert(hcsr04p_sensor_init() == ESP_OK && step == calls && !triggers);
    // Exactly 2 and 400 cm at a synthetic resolution, plus one tick outside.
    assert(decode_echo(4, 34300, &out) == ESP_OK && out.distance_cm == 2);
    assert(decode_echo(800, 34300, &out) == ESP_OK && out.distance_cm == 400);
    assert(decode_echo(3, 34300, &out) == ESP_ERR_INVALID_RESPONSE);
    assert(decode_echo(801, 34300, &out) == ESP_ERR_INVALID_RESPONSE);
    assert(decode_echo(0, resolution, &out) == ESP_ERR_INVALID_RESPONSE);
    assert(decode_echo(UINT32_MAX, resolution, &out) == ESP_ERR_INVALID_RESPONSE);
    assert(decode_echo(4, 0, &out) == ESP_ERR_INVALID_RESPONSE);
    // APB-resolution boundary ticks: 2 cm = 9329.446..., 400 = 1865889.212...
    read_case(VALID, 9330, ESP_OK);
    read_case(VALID, 9329, ESP_ERR_INVALID_RESPONSE);
    read_case(VALID, 1865889, ESP_OK);
    read_case(VALID, 1865890, ESP_ERR_INVALID_RESPONSE);
    read_case(VALID, 80000, ESP_OK); // 1000 us = 17.15 cm.
    read_case(VALID, 0, ESP_ERR_INVALID_RESPONSE);
    read_case(VALID, 1, ESP_ERR_INVALID_RESPONSE);
    rise_ticks = UINT32_MAX - 100;
    read_case(VALID, 80000, ESP_OK); // Hardware counter wraps during ECHO.
    read_case(MISSING, 0, ESP_ERR_TIMEOUT);
    // Idle/late callbacks are discarded before the next request is armed.
    unsigned old_sends = sends;
    edge(MCPWM_CAP_EDGE_NEG, 99); edge(MCPWM_CAP_EDGE_POS, 100);
    assert(sends == old_sends && !queued);
    read_case(RISE_ONLY, 0, ESP_ERR_TIMEOUT);
    read_case(LATE, 80000, ESP_ERR_TIMEOUT); // Valid width, response beyond 30 ms.
    read_case(FALL_ONLY, 80000, ESP_ERR_INVALID_RESPONSE);
    read_case(DOUBLE_RISE, 80000, ESP_ERR_INVALID_RESPONSE);
    read_case(VALID, 80000, ESP_OK); // Normal acquisition resumes without reinit.
    assert(step == calls);
    assert(hcsr04p_sensor_read(&out) == ESP_ERR_INVALID_STATE); // <60 ms, no trigger.
    test_now_us = s_last_trigger_us + 60000; mode = VALID;
    assert(hcsr04p_sensor_read(&out) == ESP_OK); // Exact trigger cycle limit.
    test_now_us += 200000; high_echo = true;
    unsigned old_triggers = triggers;
    assert(hcsr04p_sensor_read(&out) == ESP_ERR_INVALID_RESPONSE && triggers == old_triggers);
    high_echo = false; trigger_fail = true;
    assert(hcsr04p_sensor_read(&out) == ESP_FAIL && !s_armed && !high_trig);
    reset(); resolution = 40000000;
    assert(hcsr04p_sensor_init() == ESP_OK);
    read_case(VALID, 40000, ESP_OK); // Uses queried resolution, not hardcoded 80 MHz.
    test_now_us += 200000;
    int64_t request_start = test_now_us;
    pretrigger_delay_us = 150000;
    assert(hcsr04p_sensor_read(&out) == ESP_OK);
    assert(s_last_trigger_us == request_start + 150000);
    test_now_us = request_start + 200000;
    old_triggers = triggers;
    assert(hcsr04p_sensor_read(&out) == ESP_ERR_INVALID_STATE && triggers == old_triggers);
    reset();
    puts("PASS: HC-SR04P real driver, range/units, capture rollover, edge validation, timeout/stale echo, trigger cycle and startup cleanup");
    return 0;
}
