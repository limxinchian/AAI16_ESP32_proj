#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float temperature_c;
    // Last frame read, set even when it is rejected. For diagnostics only:
    // 0x0000 = SO stuck low, 0xFFFF = SO stuck high or floating.
    uint16_t raw;
} max6675_data_t;

// Single reader/task owns all calls. Shared SPI bus must already be initialized.
// CS=GPIO10, SCK=GPIO12, MISO(SO)=GPIO13, SPI mode 0 at 1 MHz; the chip has no MOSI.
// Waits one conversion (~250 ms) after adding the device, then requires a valid
// frame. An open thermocouple does NOT fail init: the chip answered, so the
// fault is reported by each read instead.
esp_err_t max6675_sensor_init(void);
esp_err_t max6675_sensor_deinit(void);

// One 16-bit frame. A read restarts the chip's ~0.22 s conversion, so callers
// must not poll faster than every 250 ms (a faster read returns a stale value).
// ESP_ERR_NOT_FOUND:        thermocouple open / not connected (fault bit set).
// ESP_ERR_INVALID_RESPONSE: framing bits wrong (floating MISO, bad CS/clock).
esp_err_t max6675_sensor_read(max6675_data_t *data);
