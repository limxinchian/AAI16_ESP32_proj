#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define MHZ1911A_SAMPLE_MS 5000
#define MHZ1911A_WARMUP_MS 60000
#define MHZ1911A_RESPONSE_TIMEOUT_MS 2000
#define MHZ1911A_FRAME_BYTES 9
#define MHZ1911A_MIN_PPM 400
#define MHZ1911A_MAX_PPM 5000

typedef struct { uint16_t co2_ppm; } mhz1911a_reading_t;

typedef struct {
    uint32_t requests, valid_frames, rx_bytes, discarded_bytes;
    uint32_t timeouts, checksum_failures, invalid_frames, partial_responses;
    uint32_t uart_errors, rx_overflows, event_overflows, tx_failures, api_errors;
    int64_t last_read_us, max_read_us;
} mhz1911a_diagnostics_t;

// Validate one exact frame. Output remains untouched on any failure.
// Hardware-tested on one module; the exact UART protocol is not yet confirmed
// by the manufacturer. Bytes 4..7 remain opaque (including any status meaning).
esp_err_t mhz1911a_sensor_decode(const uint8_t *frame, size_t length,
                                mhz1911a_reading_t *out);

// Single acquisition owner only, including init/deinit/diagnostic access.
// UART2 TX15/RX16, 9600 8N1. UART setup does not establish sensor readiness.
// The reader must enforce warm-up and at least 5000 ms between acquisitions.
esp_err_t mhz1911a_sensor_init(void);
esp_err_t mhz1911a_sensor_deinit(void);
// One request, no immediate retries. Flush stale input; require a quiet input
// boundary; assemble an exact response with a short trailing-data guard.
// Bounded waits: 100 ms TX preflight + 20 ms quiet + 2000 ms response window
// (includes TX completion), plus RTOS scheduling/tick rounding. No busy wait.
esp_err_t mhz1911a_sensor_read(mhz1911a_reading_t *out);
void mhz1911a_sensor_get_diagnostics(mhz1911a_diagnostics_t *out);
