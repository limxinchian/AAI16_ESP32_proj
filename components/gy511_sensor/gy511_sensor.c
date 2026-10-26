
#include "gy511_sensor.h"
#include "i2c_bus_manager.h"

#include <stdint.h>
#include <stddef.h>

#include "driver/i2c_master.h"
#include "esp_log.h"

#define GY511_ACCEL_ADDR  0x19
#define GY511_MAG_ADDR    0x1E

// Accelerometer registers
#define ACC_CTRL_REG1     0x20
#define ACC_CTRL_REG4     0x23
#define ACC_OUT_X_L       0x28

// Magnetometer registers
#define MAG_CRA_REG       0x00
#define MAG_CRB_REG       0x01
#define MAG_MODE_REG      0x02
#define MAG_OUT_X_H       0x03
#define MAG_ID_REG        0x0A

#define I2C_TIMEOUT_MS    100

static i2c_master_dev_handle_t s_accel = NULL;
static i2c_master_dev_handle_t s_mag = NULL;


// Write one sensor register.
static esp_err_t write_reg(
    i2c_master_dev_handle_t device,
    uint8_t reg,
    uint8_t value)
{
    uint8_t buffer[2] = {reg, value};

    return i2c_master_transmit(
        device,
        buffer,
        sizeof(buffer),
        I2C_TIMEOUT_MS
    );
}


// Read one or more sensor registers.
static esp_err_t read_regs(
    i2c_master_dev_handle_t device,
    uint8_t reg,
    uint8_t *buffer,
    size_t length)
{
    return i2c_master_transmit_receive(
        device,
        &reg,
        1,
        buffer,
        length,
        I2C_TIMEOUT_MS
    );
}


// Combine two bytes into a signed 16-bit value.
static int16_t decode_le16(uint8_t low, uint8_t high)
{
    uint16_t value =
        ((uint16_t)high << 8) | low;

    return (int16_t)value;
}

static int16_t decode_be16(uint8_t high, uint8_t low)
{
    uint16_t value =
        ((uint16_t)high << 8) | low;

    return (int16_t)value;
}


// Initialize GY-511.
esp_err_t gy511_sensor_init(void)
{
    if (s_accel != NULL && s_mag != NULL) {
        return ESP_OK;
    }

    i2c_master_bus_handle_t bus =
        i2c_bus_manager_get_handle();

    if (bus == NULL) {
        ESP_LOGE("SENSOR", "gy511   | ERROR | I2C bus is not initialized.");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t accel_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GY511_ACCEL_ADDR,
        .scl_speed_hz = 100000,
    };

    i2c_device_config_t mag_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GY511_MAG_ADDR,
        .scl_speed_hz = 100000,
    };

    esp_err_t err = i2c_master_bus_add_device(
        bus, &accel_cfg, &s_accel
    );

    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "gy511   | ERROR | Cannot add accelerometer.");
        return err;
    }

    err = i2c_master_bus_add_device(
        bus, &mag_cfg, &s_mag
    );

    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "gy511   | ERROR | Cannot add magnetometer.");
        goto fail;
    }

    // Verify the magnetometer identification.
    uint8_t id[3] = {0};

    err = read_regs(
        s_mag, MAG_ID_REG, id, sizeof(id)
    );

    if (err != ESP_OK) {
        goto fail;
    }

    if (id[0] != 0x48 ||
        id[1] != 0x34 ||
        id[2] != 0x33) {

        ESP_LOGE("SENSOR", "gy511   | ERROR | Unexpected magnetometer ID.");
        err = ESP_ERR_INVALID_RESPONSE;
        goto fail;
    }

    ESP_LOGI("SENSOR", "gy511   | HEALTH | Magnetometer ID verified: H43");

    // Accelerometer:
    // 0x88 = BDU, high resolution, +/-2g.
    err = write_reg(
        s_accel, ACC_CTRL_REG4, 0x88
    );
    if (err != ESP_OK) goto fail;

    // 0x57 = 100 Hz, X/Y/Z enabled.
    err = write_reg(
        s_accel, ACC_CTRL_REG1, 0x57
    );
    if (err != ESP_OK) goto fail;

    // Magnetometer:
    // 0x14 = 30 Hz output data rate.
    err = write_reg(
        s_mag, MAG_CRA_REG, 0x14
    );
    if (err != ESP_OK) goto fail;

    // 0x20 = +/-1.3 gauss.
    err = write_reg(
        s_mag, MAG_CRB_REG, 0x20
    );
    if (err != ESP_OK) goto fail;

    // 0x00 = continuous conversion mode.
    err = write_reg(
        s_mag, MAG_MODE_REG, 0x00
    );
    if (err != ESP_OK) goto fail;

    ESP_LOGI("SENSOR", "gy511   | HEALTH | GY-511 initialized successfully.");
    ESP_LOGI("SENSOR", "gy511   | HEALTH | Accelerometer: 100 Hz, +/-2g");
    ESP_LOGI("SENSOR", "gy511   | HEALTH | Magnetometer : 30 Hz, +/-1.3 gauss");

    return ESP_OK;

fail:
    ESP_LOGE("SENSOR", "gy511   | ERROR | GY-511 initialization failed: %s",
             esp_err_to_name(err));

    esp_err_t cleanup_err = gy511_sensor_deinit();
    if (cleanup_err != ESP_OK) {
        ESP_LOGE("SENSOR", "gy511   | ERROR | Device cleanup failed: %s", esp_err_to_name(cleanup_err));
    }
    return err;
}

esp_err_t gy511_sensor_deinit(void)
{
    esp_err_t result = ESP_OK;
    if (s_mag != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(s_mag);
        if (err == ESP_OK) s_mag = NULL;
        else result = err;
    }

    if (s_accel != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(s_accel);
        if (err == ESP_OK) s_accel = NULL;
        else if (result == ESP_OK) result = err;
    }
    return result;
}


// Read acceleration and magnetic field.
esp_err_t gy511_sensor_read(gy511_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_accel == NULL || s_mag == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t accel[6] = {0};
    uint8_t mag[6] = {0};

    // Accelerometer: auto-increment enabled.
    esp_err_t err = read_regs(
        s_accel,
        ACC_OUT_X_L | 0x80,
        accel,
        sizeof(accel)
    );

    if (err != ESP_OK) {
        return err;
    }

    // Magnetometer has automatic address increment.
    err = read_regs(
        s_mag,
        MAG_OUT_X_H,
        mag,
        sizeof(mag)
    );

    if (err != ESP_OK) {
        return err;
    }

    // Acceleration: little-endian, 12-bit
    // high-resolution data, left justified.
    int16_t ax = decode_le16(accel[0], accel[1]);
    int16_t ay = decode_le16(accel[2], accel[3]);
    int16_t az = decode_le16(accel[4], accel[5]);

    // +/-2g high-resolution sensitivity:
    // 1 mg per 12-bit count.
    data->accel_x_g = (float)ax / 16000.0f;
    data->accel_y_g = (float)ay / 16000.0f;
    data->accel_z_g = (float)az / 16000.0f;

    // Magnetometer uses big-endian X, Z, Y order.
    int16_t mx = decode_be16(mag[0], mag[1]);
    int16_t mz = decode_be16(mag[2], mag[3]);
    int16_t my = decode_be16(mag[4], mag[5]);

    // Treat the magnetic saturation marker as invalid.
    if (mx == -4096 || my == -4096 || mz == -4096) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // At +/-1.3 gauss:
    // X/Y = 1100 LSB/gauss, Z = 980 LSB/gauss.
    // 1 gauss = 100 microtesla.
    data->mag_x_ut = (float)mx * 100.0f / 1100.0f;
    data->mag_y_ut = (float)my * 100.0f / 1100.0f;
    data->mag_z_ut = (float)mz * 100.0f / 980.0f;

    return ESP_OK;
}
