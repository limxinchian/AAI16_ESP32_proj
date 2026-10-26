#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Call once from app_main. Owns the sixth registry slot and UART2 driver task.
void mhz1911a_reader_start(void);
// Warm-up, disabled and READ-stage failures retain their live control owner.
TaskHandle_t mhz1911a_reader_task_handle(void);
