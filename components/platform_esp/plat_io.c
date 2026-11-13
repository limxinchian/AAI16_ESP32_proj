/* GPIO / PWM outputs driven by rule actions. Pin policy is enforced by the rule engine
 * (cm_pin_allowed + not owned by a running sensor) before these are called. NOT hardware-verified. */
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "platform_esp.h"

static const char *TAG = "plat_io";

static void io_set(void *c, uint32_t pin, bool level)
{
    (void)c;
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level((gpio_num_t)pin, level);
}

static void io_toggle(void *c, uint32_t pin)
{
    (void)c;
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level((gpio_num_t)pin, !gpio_get_level((gpio_num_t)pin));
}

/* A fixed pool of one-shot timers, each with its own state slot: no per-pulse allocation. */
#define PULSE_POOL 4
static struct {
    esp_timer_handle_t timer;
    uint32_t pin;
    bool restore_level;
} s_pulse[PULSE_POOL];

static void pulse_end(void *arg)
{
    const typeof(s_pulse[0]) *slot = arg;
    gpio_set_level((gpio_num_t)slot->pin, slot->restore_level);
}

static void io_pulse(void *c, uint32_t pin, bool level, uint32_t ms)
{
    for (int i = 0; i < PULSE_POOL; i++) {
        if (!s_pulse[i].timer) {
            const esp_timer_create_args_t a = {.callback = pulse_end, .arg = &s_pulse[i], .dispatch_method = ESP_TIMER_TASK,
                                               .name = "pulse", .skip_unhandled_events = true};
            if (esp_timer_create(&a, &s_pulse[i].timer) != ESP_OK) { s_pulse[i].timer = NULL; continue; }
        }
        if (esp_timer_is_active(s_pulse[i].timer)) continue;
        s_pulse[i].pin = pin;
        s_pulse[i].restore_level = !level;
        io_set(c, pin, level);
        esp_timer_start_once(s_pulse[i].timer, (uint64_t)ms * 1000u);
        return;
    }
    ESP_LOGW(TAG, "pulse on pin %u dropped: all pulse timers busy", (unsigned)pin);
}

#define PWM_CHANNELS 8
static int16_t s_pwm_pin[PWM_CHANNELS]; /* pin + 1; 0 = free */
static bool s_pwm_timer_ready;

static void io_pwm(void *c, uint32_t pin, uint32_t permille)
{
    (void)c;
    if (!s_pwm_timer_ready) {
        const ledc_timer_config_t tc = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
                                        .timer_num = LEDC_TIMER_0, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK};
        if (ledc_timer_config(&tc) != ESP_OK) { ESP_LOGE(TAG, "ledc timer config failed"); return; }
        s_pwm_timer_ready = true;
    }
    int ch = -1;
    for (int i = 0; i < PWM_CHANNELS; i++) {
        if (s_pwm_pin[i] == (int16_t)(pin + 1)) { ch = i; break; }
    }
    if (ch < 0) {
        for (int i = 0; i < PWM_CHANNELS; i++) {
            if (!s_pwm_pin[i]) { ch = i; break; }
        }
        if (ch < 0) { ESP_LOGE(TAG, "no free PWM channel"); return; }
        const ledc_channel_config_t cc = {.gpio_num = (int)pin, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = (ledc_channel_t)ch,
                                          .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0};
        if (ledc_channel_config(&cc) != ESP_OK) return;
        s_pwm_pin[ch] = (int16_t)(pin + 1);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, permille * 1023u / 1000u);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}

void plat_io_fill(node_platform_t *pf)
{
    pf->gpio_set = io_set;
    pf->gpio_toggle = io_toggle;
    pf->gpio_pulse = io_pulse;
    pf->pwm = io_pwm;
}
