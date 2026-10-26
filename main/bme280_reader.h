#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Called once during app_main startup, after the shared I2C bus is initialized.
// Starts a dedicated acquisition owner that probes/initializes the sensor in its
// own task. Failures are recorded locally; this never aborts platform startup.
void bme280_reader_start(void);

// app_main uses this for the shared no-overwrite configuration handoff. NULL
// until initialization succeeds, or if startup failed. After successful init
// the task stays alive even when disabled; enabling does not recover hardware.
TaskHandle_t bme280_reader_task_handle(void);
