#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define LD2450_FRAME_BYTES 30
#define LD2450_TARGETS 3

typedef struct {
    bool present; // An unused slot is eight zero bytes, per the protocol.
    int16_t x_mm, y_mm, speed_cm_s;
    uint16_t resolution_mm; // Range gate size, NOT distance to the target.
} ld2450_target_t;

typedef struct {
    ld2450_target_t targets[LD2450_TARGETS];
    unsigned count;
} ld2450_report_t;

// Fixed-size streaming parser, also usable with captured UART data on a host.
// Target reports have no checksum: plausible payload corruption is undetectable.
typedef struct {
    uint8_t bytes[LD2450_FRAME_BYTES];
    size_t used;
    int64_t last_byte_us;
    uint32_t frames, framing_errors, discarded_bytes, partial_timeouts;
} ld2450_parser_t;

unsigned ld2450_parser_feed(ld2450_parser_t *parser, const uint8_t *data,
    size_t length, int64_t now_us, ld2450_report_t *latest);
void ld2450_parser_expire(ld2450_parser_t *parser, int64_t now_us);
void ld2450_parser_discard(ld2450_parser_t *parser);

typedef struct {
    uint32_t rx_bytes, frames, framing_errors, partial_timeouts;
    uint32_t discarded_bytes; // Known discarded bytes only; hardware loss is unknowable.
    uint32_t uart_errors, rx_overflows, event_overflows;
    size_t max_buffered_bytes;
    int64_t max_processing_us; // Excludes the blocking receive wait.
} ld2450_diagnostics_t;

// One owner task, UART1 TX17/RX18, 256000 8N1. No commands are transmitted.
// UART setup alone is NOT evidence that the radar is connected.
esp_err_t ld2450_sensor_init(void);
esp_err_t ld2450_sensor_deinit(void);
// Owner-only resume boundary: discard buffered input, queued old UART events
// and partial parser state, preserving driver ownership and diagnostics.
esp_err_t ld2450_sensor_discard_pending(void);
// Wait at most 100 ms, process at most 256 bytes, retain only the latest report.
// *frames counts all complete reports in this batch. A framing error may coexist
// with valid reports after resynchronization; UART errors invalidate the batch.
esp_err_t ld2450_sensor_receive(ld2450_report_t *latest, unsigned *frames);
void ld2450_sensor_get_diagnostics(ld2450_diagnostics_t *out);
