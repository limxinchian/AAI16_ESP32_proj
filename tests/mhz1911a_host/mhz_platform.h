#pragma once
#include "../ld2450_host/platform.h"
#define UART_NUM_2 2
int uart_tx_chars(uart_port_t port, const char *bytes, uint32_t length);
esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t wait);
