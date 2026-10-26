#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Call once from app_main. Reader owns acquisition and runtime configuration.
void hcsr04p_reader_start(void);

// NULL until capture initialization succeeds, or after startup failure. The
// initialized reader stays alive and receives configuration while disabled.
TaskHandle_t hcsr04p_reader_task_handle(void);
