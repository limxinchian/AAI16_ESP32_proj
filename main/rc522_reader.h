#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Called once during app_main startup. Initializes the shared SPI bus, then
// starts a dedicated acquisition owner. RF reset/protocol waits run in that task.
// Failures are recorded locally; this function never aborts platform startup.
void rc522_reader_start(void);

// app_main uses this for the shared no-overwrite configuration handoff. NULL
// until initialization succeeds, or if startup failed. After successful init
// the task stays alive even when disabled; enabling does not recover hardware.
TaskHandle_t rc522_reader_task_handle(void);
