#include "hcsr04p_sensor.h"

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif

typedef struct {
    uint32_t ticks;
    esp_err_t error;
} echo_event_t;

static mcpwm_cap_timer_handle_t s_timer;
static mcpwm_cap_channel_handle_t s_channel;
static QueueHandle_t s_events;
static uint32_t s_resolution_hz;
static bool s_attempted, s_ready, s_gpio_owned;
static bool s_timer_enabled, s_timer_started, s_channel_enabled;
static int64_t s_last_trigger_us = -1;
// Shared with the capture ISR. Queue send and disarm share this lock so a late
// callback cannot enqueue an old result after the next read drains the queue.
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_armed, s_rising;
static uint32_t s_rise_ticks;
static int64_t s_deadline_us;

static bool echo_captured(mcpwm_cap_channel_handle_t channel,
                          const mcpwm_capture_event_data_t *event, void *context)
{
    (void)channel;
    (void)context;
    BaseType_t wake = pdFALSE;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_lock);
    if (s_armed) {
        echo_event_t result = {.error = ESP_OK};
        bool done = true;
        if (now > s_deadline_us) {
            result.error = ESP_ERR_TIMEOUT;
        } else if (event->cap_edge == MCPWM_CAP_EDGE_POS && !s_rising) {
            s_rising = true;
            s_rise_ticks = event->cap_value;
            done = false;
        } else if (event->cap_edge == MCPWM_CAP_EDGE_NEG && s_rising) {
            // Unsigned subtraction also handles capture counter rollover.
            result.ticks = event->cap_value - s_rise_ticks;
        } else {
            result.error = ESP_ERR_INVALID_RESPONSE;
        }
        if (done) {
            s_armed = false;
            // Exactly one completion per armed read; no logging/float work here.
            (void)xQueueSendFromISR(s_events, &result, &wake);
        }
    }
    portEXIT_CRITICAL_ISR(&s_lock);
    return wake == pdTRUE;
}

static esp_err_t release_resources(void)
{
    // Used only on failed startup, while never armed. Stop at a cleanup error
    // rather than freeing memory an interrupt/peripheral may still reference.
    esp_err_t err;
    if (s_channel_enabled) {
        if ((err = mcpwm_capture_channel_disable(s_channel)) != ESP_OK) return err;
        s_channel_enabled = false;
    }
    if (s_timer_started) {
        if ((err = mcpwm_capture_timer_stop(s_timer)) != ESP_OK) return err;
        s_timer_started = false;
    }
    if (s_timer_enabled) {
        if ((err = mcpwm_capture_timer_disable(s_timer)) != ESP_OK) return err;
        s_timer_enabled = false;
    }
    if (s_channel) {
        if ((err = mcpwm_del_capture_channel(s_channel)) != ESP_OK) return err;
        s_channel = NULL;
    }
    if (s_timer) {
        if ((err = mcpwm_del_capture_timer(s_timer)) != ESP_OK) return err;
        s_timer = NULL;
    }
    if (s_events) {
        vQueueDelete(s_events);
        s_events = NULL;
    }
    if (s_gpio_owned) {
        if ((err = gpio_reset_pin(HCSR04P_TRIG_GPIO)) != ESP_OK) return err;
        s_gpio_owned = false;
    }
    return ESP_OK;
}

esp_err_t hcsr04p_sensor_init(void)
{
    if (s_ready) return ESP_OK;
    if (s_attempted) return ESP_ERR_INVALID_STATE;
    s_attempted = true;
    esp_err_t err = ESP_ERR_NO_MEM;
    s_events = xQueueCreate(1, sizeof(echo_event_t));
    if (!s_events) goto fail;
    const gpio_config_t trig = {
        .pin_bit_mask = 1ULL << HCSR04P_TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    if ((err = gpio_config(&trig)) != ESP_OK) goto fail;
    s_gpio_owned = true;
    if ((err = gpio_set_level(HCSR04P_TRIG_GPIO, 0)) != ESP_OK) goto fail;
    const mcpwm_capture_timer_config_t timer = {
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_APB,
        // ESP32-S3 capture resolution is the APB frequency, queried below.
    };
    if ((err = mcpwm_new_capture_timer(&timer, &s_timer)) != ESP_OK) goto fail;
    const mcpwm_capture_channel_config_t channel = {
        .gpio_num = HCSR04P_ECHO_GPIO,
        .prescale = 1,
        .flags.pos_edge = true,
        .flags.neg_edge = true,
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR < 6
        .flags.pull_down = true, // Disconnected ECHO stays low and times out.
#endif
    };
    if ((err = mcpwm_new_capture_channel(s_timer, &channel, &s_channel)) != ESP_OK) goto fail;
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 6
    // IDF 6 dropped the channel pull_down flag and leaves pulls to the GPIO API.
    // Same behaviour: a disconnected ECHO stays low and times out.
    if ((err = gpio_set_pull_mode(HCSR04P_ECHO_GPIO, GPIO_PULLDOWN_ONLY)) != ESP_OK) goto fail;
#endif
    const mcpwm_capture_event_callbacks_t callbacks = {.on_cap = echo_captured};
    if ((err = mcpwm_capture_channel_register_event_callbacks(s_channel, &callbacks, NULL)) != ESP_OK) goto fail;
    if ((err = mcpwm_capture_timer_get_resolution(s_timer, &s_resolution_hz)) != ESP_OK) goto fail;
    if (!s_resolution_hz) { err = ESP_ERR_INVALID_STATE; goto fail; }
    if ((err = mcpwm_capture_timer_enable(s_timer)) != ESP_OK) goto fail;
    s_timer_enabled = true;
    if ((err = mcpwm_capture_timer_start(s_timer)) != ESP_OK) goto fail;
    s_timer_started = true;
    if ((err = mcpwm_capture_channel_enable(s_channel)) != ESP_OK) goto fail;
    s_channel_enabled = true;
    s_ready = true;
    return ESP_OK;
fail:;
    esp_err_t cleanup = release_resources();
    if (cleanup != ESP_OK) ESP_LOGE("SENSOR", "hcsr04p | ERROR | Startup cleanup failed: %s", esp_err_to_name(cleanup));
    return err;
}

static esp_err_t decode_echo(uint32_t ticks, uint32_t hz, hcsr04p_reading_t *out)
{
    // Round trip at nominal 343 m/s: distance_cm = seconds * 34300 / 2.
    // Check range before float rounding, including the exact 2/400 cm limits.
    uint64_t scaled = (uint64_t)ticks * 34300;
    if (!hz || scaled < (uint64_t)HCSR04P_MIN_CM * 2 * hz ||
        scaled > (uint64_t)HCSR04P_MAX_CM * 2 * hz) return ESP_ERR_INVALID_RESPONSE;
    out->echo_us = (float)((double)ticks * 1000000 / hz);
    out->distance_cm = (float)((double)scaled / (2.0 * hz));
    return ESP_OK;
}

esp_err_t hcsr04p_sensor_read(hcsr04p_reading_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    *out = (hcsr04p_reading_t){0};
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    echo_event_t event;
    (void)xQueueReceive(s_events, &event, 0); // At most one stale completion.
    portENTER_CRITICAL(&s_lock);
    // Anchor to the physical trigger, even if the reader was preempted before
    // entering this short critical section. esp_timer_get_time is ISR-safe.
    int64_t started = esp_timer_get_time();
    if (s_last_trigger_us >= 0 && started - s_last_trigger_us < HCSR04P_MIN_CYCLE_MS * 1000LL) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    // A stuck-high pin cannot be the beginning of a new echo pulse.
    if (gpio_get_level(HCSR04P_ECHO_GPIO)) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_RESPONSE;
    }
    s_last_trigger_us = started;
    s_rising = false;
    s_deadline_us = started + HCSR04P_ECHO_TIMEOUT_MS * 1000LL;
    s_armed = true;
    esp_err_t high = gpio_set_level(HCSR04P_TRIG_GPIO, 1);
    // Only a short trigger pulse spins; ECHO is timed entirely in hardware.
    // Critical section prevents a task switch stretching the 12 us pulse.
    esp_rom_delay_us(12);
    esp_err_t low = gpio_set_level(HCSR04P_TRIG_GPIO, 0);
    if (high != ESP_OK || low != ESP_OK) s_armed = false;
    portEXIT_CRITICAL(&s_lock);
    if (high != ESP_OK || low != ESP_OK) return high != ESP_OK ? high : low;

    // One extra tick avoids an early timeout near a tick boundary. The ISR's
    // absolute deadline rejects echoes arriving in the rounding allowance.
    TickType_t wait = pdMS_TO_TICKS(HCSR04P_ECHO_TIMEOUT_MS + portTICK_PERIOD_MS - 1) + 1;
    BaseType_t received = xQueueReceive(s_events, &event, wait);
    portENTER_CRITICAL(&s_lock);
    s_armed = false;
    portEXIT_CRITICAL(&s_lock);
    if (received != pdTRUE) return ESP_ERR_TIMEOUT;
    if (event.error != ESP_OK) return event.error;
    return decode_echo(event.ticks, s_resolution_hz, out);
}
