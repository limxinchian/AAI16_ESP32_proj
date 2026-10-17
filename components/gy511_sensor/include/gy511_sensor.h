
#pragma once

#include "esp_err.h"

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;
} gy511_data_t;

// Initialize both GY-511 sensing devices.
esp_err_t gy511_sensor_init(void);

// Release this sensor's device handles, never the shared bus. Call only when
// no reader is running (initialization cleanup or reader-task creation failure).
esp_err_t gy511_sensor_deinit(void);

// Read all three acceleration and magnetic axes.
esp_err_t gy511_sensor_read(gy511_data_t *data);
