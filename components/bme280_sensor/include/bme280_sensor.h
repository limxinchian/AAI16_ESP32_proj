
#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float temperature_c;
    float pressure_hpa;
    // Valid only when has_humidity is true. A BMP280 (chip ID 0x58) has no
    // humidity sensor, so the field is never fabricated for that variant.
    float humidity_pct;
    bool has_humidity;
} bme280_data_t;

// Probe 0x76 then 0x77 on the shared I2C bus, verify the chip ID (BME280 0x60
// or BMP280 0x58), reset, and read the factory calibration once.
esp_err_t bme280_sensor_init(void);

// Release this sensor's device handle, never the shared bus. Call only when no
// reader is running.
esp_err_t bme280_sensor_deinit(void);

// One forced-mode measurement (x1 oversampling, filter off). Blocks at most
// ~60 ms in bounded 10 ms steps. Out-of-range results are rejected with
// ESP_ERR_INVALID_RESPONSE rather than published as plausible values.
esp_err_t bme280_sensor_read(bme280_data_t *data);
