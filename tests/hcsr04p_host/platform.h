#pragma once
#include "../sensor_host/platform.h"

#define portENTER_CRITICAL_ISR portENTER_CRITICAL
#define portEXIT_CRITICAL_ISR portEXIT_CRITICAL
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned length, unsigned item_size);
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait);
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *item, BaseType_t *wake);
void vQueueDelete(QueueHandle_t queue);
void vTaskDelete(TaskHandle_t task);
void esp_rom_delay_us(uint32_t us);

#define GPIO_MODE_OUTPUT 1
typedef struct { uint64_t pin_bit_mask; int mode; } gpio_config_t;
esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);
int gpio_get_level(gpio_num_t pin);
esp_err_t gpio_reset_pin(gpio_num_t pin);

typedef void *mcpwm_cap_timer_handle_t;
typedef void *mcpwm_cap_channel_handle_t;
#define MCPWM_CAPTURE_CLK_SRC_APB 1
#define MCPWM_CAP_EDGE_POS 0
#define MCPWM_CAP_EDGE_NEG 1
typedef struct { int group_id, clk_src; uint32_t resolution_hz; } mcpwm_capture_timer_config_t;
typedef struct {
    int gpio_num;
    uint32_t prescale;
    struct { bool pos_edge, neg_edge, pull_down; } flags;
} mcpwm_capture_channel_config_t;
typedef struct { int cap_edge; uint32_t cap_value; } mcpwm_capture_event_data_t;
typedef struct {
    bool (*on_cap)(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t *, void *);
} mcpwm_capture_event_callbacks_t;
esp_err_t mcpwm_new_capture_timer(const mcpwm_capture_timer_config_t *config, mcpwm_cap_timer_handle_t *out);
esp_err_t mcpwm_new_capture_channel(mcpwm_cap_timer_handle_t timer,
    const mcpwm_capture_channel_config_t *config, mcpwm_cap_channel_handle_t *out);
esp_err_t mcpwm_capture_channel_register_event_callbacks(mcpwm_cap_channel_handle_t channel,
    const mcpwm_capture_event_callbacks_t *callbacks, void *context);
esp_err_t mcpwm_capture_timer_get_resolution(mcpwm_cap_timer_handle_t timer, uint32_t *hz);
esp_err_t mcpwm_capture_timer_enable(mcpwm_cap_timer_handle_t timer);
esp_err_t mcpwm_capture_timer_start(mcpwm_cap_timer_handle_t timer);
esp_err_t mcpwm_capture_channel_enable(mcpwm_cap_channel_handle_t channel);
esp_err_t mcpwm_capture_channel_disable(mcpwm_cap_channel_handle_t channel);
esp_err_t mcpwm_capture_timer_stop(mcpwm_cap_timer_handle_t timer);
esp_err_t mcpwm_capture_timer_disable(mcpwm_cap_timer_handle_t timer);
esp_err_t mcpwm_del_capture_channel(mcpwm_cap_channel_handle_t channel);
esp_err_t mcpwm_del_capture_timer(mcpwm_cap_timer_handle_t timer);
