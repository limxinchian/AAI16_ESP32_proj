#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Call once from app_main. UART reception continues while telemetry is disabled.
void ld2450_reader_start(void);

// NULL until a valid frame proves initialization, or after startup failure.
// The initialized task stays alive and owns all runtime configuration commits.
TaskHandle_t ld2450_reader_task_handle(void);
