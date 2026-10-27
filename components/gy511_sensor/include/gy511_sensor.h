
#pragma once

#include "esp_err.h"
#include <stdint.h>

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;
    // ESP timer at completion of the acceleration register burst, before the
    // magnetic read. Host acquisition time, not the chip's conversion instant.
    int64_t acquired_us;
} gy511_data_t;

// Initialize both GY-511 sensing devices.
esp_err_t gy511_sensor_init(void);

// Release this sensor's device handles, never the shared bus. Call only when
// no reader is running (initialization cleanup or reader-task creation failure).
esp_err_t gy511_sensor_deinit(void);

// Single reader only. Require new XYZ data (STATUS_REG_A.ZYXDA), then read
// all axes. No polling loop, cache or retry. ESP_ERR_INVALID_STATE also means
// no fresh acceleration yet. On any failure, *data is unchanged.
esp_err_t gy511_sensor_read(gy511_data_t *data);
