#include "platform.h"
#include <assert.h>
#include <string.h>
#include "../../components/gy511_sensor/gy511_sensor.c"

static int accel_token, mag_token, bus_token;
static unsigned allocated, adds, removes;
static bool fail_second_add, fail_id, fail_remove_mag;
static bool saturated, fail_accel_read, fail_mag_read, no_fresh, fail_status, bad_readback;
static unsigned mag_reads, accel_reads;
static uint8_t accel_regs[256], mag_regs[256];

i2c_master_bus_handle_t i2c_bus_manager_get_handle(void) { return &bus_token; }
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
    const i2c_device_config_t *config, i2c_master_dev_handle_t *device)
{
    assert(bus == &bus_token && config->scl_speed_hz == 100000);
    adds++;
    if (fail_second_add && adds == 2) return ESP_ERR_NO_MEM;
    *device = config->device_address == 0x19 ? &accel_token : &mag_token;
    allocated++;
    return ESP_OK;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
    assert(device == &accel_token || device == &mag_token);
    removes++;
    if (fail_remove_mag && device == &mag_token) return ESP_ERR_INVALID_STATE;
    assert(allocated > 0); allocated--;
    return ESP_OK;
}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device,
    const uint8_t *buffer, size_t length, int timeout_ms)
{
    assert(timeout_ms == 100 && length == 2);
    (device == &accel_token ? accel_regs : mag_regs)[buffer[0]] = buffer[1];
    return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
    const uint8_t *reg, size_t reg_length, uint8_t *buffer, size_t length, int timeout_ms)
{
    assert(reg_length == 1 && timeout_ms == 100);
    if (*reg == MAG_ID_REG) {
        assert(device == &mag_token && length == 3);
        if (fail_id) return ESP_ERR_TIMEOUT;
        memcpy(buffer, "H43", 3);
    } else if (length == 1 && *reg != ACC_STATUS_REG) {
        *buffer = (device == &accel_token ? accel_regs : mag_regs)[*reg];
        if (bad_readback) *buffer ^= 1;
    } else if (*reg == ACC_STATUS_REG) {
        assert(device == &accel_token && length == 1);
        if (fail_status) return ESP_ERR_TIMEOUT;
        *buffer = no_fresh ? 0 : 0x88; // Overrun is expected when polling 100 Hz at 10 Hz.
    } else if (device == &accel_token) {
        assert(*reg == (ACC_OUT_X_L | 0x80) && length == 6);
        accel_reads++;
        if (fail_accel_read) return ESP_ERR_TIMEOUT;
        const uint8_t accel[] = {0x80, 0x3e, 0x80, 0xc1, 0x40, 0x1f};
        memcpy(buffer, accel, 6); // +1 g, -1 g, +0.5 g.
        test_now_us += 1000;
    } else {
        assert(device == &mag_token && *reg == MAG_OUT_X_H && length == 6);
        mag_reads++;
        if (fail_mag_read) return ESP_ERR_TIMEOUT;
        test_now_us += 2000;
        const uint8_t mag[] = {0x04, 0x4c, 0x03, 0xd4, 0xfb, 0xb4};
        memcpy(buffer, mag, 6); // X=+100, Z=+100, Y=-100 uT.
        if (saturated) { buffer[0] = 0xf0; buffer[1] = 0; }
    }
    return ESP_OK;
}

int main(void)
{
    fail_second_add = true;
    assert(gy511_sensor_init() == ESP_ERR_NO_MEM && allocated == 0 && removes == 1);
    assert(!s_accel && !s_mag);
    fail_second_add = false; fail_id = true;
    assert(gy511_sensor_init() == ESP_ERR_TIMEOUT && allocated == 0);
    assert(!s_accel && !s_mag);
    fail_id = false;
    bad_readback = true;
    assert(gy511_sensor_init() == ESP_ERR_INVALID_RESPONSE && allocated == 0);
    bad_readback = false;
    assert(gy511_sensor_init() == ESP_OK && allocated == 2);
    assert(accel_regs[ACC_CTRL_REG1] == 0x57 && accel_regs[ACC_CTRL_REG4] == 0x88);
    assert(mag_regs[MAG_CRA_REG] == 0x14);
    gy511_data_t data;
    test_now_us = 100000;
    assert(gy511_sensor_read(&data) == ESP_OK);
    assert(data.acquired_us == 101000 && test_now_us == 103000);
    assert(data.accel_x_g == 1 && data.accel_y_g == -1 && data.accel_z_g == 0.5f);
    assert(data.mag_x_ut == 100 && data.mag_y_ut == -100 && data.mag_z_ut == 100);
    gy511_data_t saved = data;
    no_fresh = true;
    unsigned before_accel = accel_reads, before_mag = mag_reads;
    assert(gy511_sensor_read(&data) == ESP_ERR_INVALID_STATE);
    assert(accel_reads == before_accel && mag_reads == before_mag);
    assert(!memcmp(&data, &saved, sizeof(data)));
    no_fresh = false; fail_status = true;
    assert(gy511_sensor_read(&data) == ESP_ERR_TIMEOUT && accel_reads == before_accel);
    fail_status = false; fail_mag_read = true;
    assert(gy511_sensor_read(&data) == ESP_ERR_TIMEOUT);
    assert(!memcmp(&data, &saved, sizeof(data)));
    fail_mag_read = false;
    assert(gy511_sensor_read(&data) == ESP_OK && data.acquired_us > saved.acquired_us);
    assert(data.accel_x_g == saved.accel_x_g); // Repeated values can be fresh.
    saved = data;
    saturated = true;
    assert(gy511_sensor_read(&data) == ESP_ERR_INVALID_RESPONSE);
    assert(!memcmp(&data, &saved, sizeof(data)));
    saturated = false; fail_accel_read = true;
    unsigned previous_mag_reads = mag_reads;
    assert(gy511_sensor_read(&data) == ESP_ERR_TIMEOUT && mag_reads == previous_mag_reads);
    fail_accel_read = false;
    assert(gy511_sensor_read(NULL) == ESP_ERR_INVALID_ARG);
    assert(gy511_sensor_deinit() == ESP_OK && allocated == 0);
    unsigned previous_removes = removes;
    assert(gy511_sensor_deinit() == ESP_OK && removes == previous_removes);
    assert(gy511_sensor_read(&data) == ESP_ERR_INVALID_STATE);

    assert(gy511_sensor_init() == ESP_OK);
    fail_remove_mag = true;
    assert(gy511_sensor_deinit() == ESP_ERR_INVALID_STATE);
    assert(allocated == 1 && s_mag == &mag_token && !s_accel);
    fail_remove_mag = false;
    assert(gy511_sensor_deinit() == ESP_OK && allocated == 0 && !s_mag);
    puts("PASS: GY-511 ODR readback, fresh XYZ, real timestamp, unchanged output on I2C/status/magnetic faults, cleanup");
    return 0;
}
