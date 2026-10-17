#pragma once
#include "../sensor_host/platform.h"
typedef void *QueueHandle_t;
typedef int uart_port_t;
#define UART_NUM_1 1
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define UART_PIN_NO_CHANGE -1
typedef struct { int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk; } uart_config_t;
typedef enum { UART_DATA, UART_FIFO_OVF, UART_BUFFER_FULL, UART_FRAME_ERR,
               UART_PARITY_ERR, UART_BREAK } uart_event_type_t;
typedef struct { uart_event_type_t type; size_t size; bool timeout_flag; } uart_event_t;
bool uart_is_driver_installed(uart_port_t port);
esp_err_t uart_driver_install(uart_port_t port, int rx, int tx, int count, QueueHandle_t *queue, int flags);
esp_err_t uart_driver_delete(uart_port_t port);
esp_err_t uart_param_config(uart_port_t port, const uart_config_t *config);
esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts);
esp_err_t uart_get_buffered_data_len(uart_port_t port, size_t *length);
int uart_read_bytes(uart_port_t port, void *data, uint32_t length, TickType_t wait);
esp_err_t uart_flush_input(uart_port_t port);
unsigned uxQueueMessagesWaiting(QueueHandle_t queue);
BaseType_t xQueueReceive(QueueHandle_t queue, void *event, TickType_t wait);
void vTaskDelete(TaskHandle_t task);
