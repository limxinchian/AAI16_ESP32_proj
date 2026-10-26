#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Called once during app_main startup. Initializes the shared SPI bus (a no-op if
// the RC522 reader already did), then starts a dedicated acquisition owner. The
// sensor is initialized in that task. Failures are recorded locally; this never
// aborts platform startup.
void max6675_reader_start(void);

// app_main uses this for the shared no-overwrite configuration handoff. NULL
// until initialization succeeds, or if startup failed. After successful init
// the task stays alive even when disabled; enabling does not recover hardware.
TaskHandle_t max6675_reader_task_handle(void);
