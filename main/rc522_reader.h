#pragma once

// Called once during app_main startup. Initializes the shared SPI bus, then
// starts a dedicated acquisition owner. RF reset/protocol waits run in that task.
// Failures are recorded locally; this function never aborts platform startup.
void rc522_reader_start(void);
