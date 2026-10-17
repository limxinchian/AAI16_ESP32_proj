#include "platform.h"
#include <assert.h>
#include <string.h>
#include "../../components/gy511_sensor/gy511_sensor.c"

static int accel_token, mag_token, bus_token;
static unsigned allocated, adds, removes;
static bool fail_second_add, fail_id, fail_remove_mag;
static bool saturated, fail_accel_read;
static unsigned mag_reads;

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
    assert(timeout_ms == 100 && length == 2); return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
    const uint8_t *reg, size_t reg_length, uint8_t *buffer, size_t length, int timeout_ms)
{
    assert(reg_length == 1 && timeout_ms == 100);
    if (*reg == MAG_ID_REG) {
        assert(device == &mag_token && length == 3);
        if (fail_id) return ESP_ERR_TIMEOUT;
        memcpy(buffer, "H43", 3);
    } else if (device == &accel_token) {
        assert(*reg == (ACC_OUT_X_L | 0x80) && length == 6);
        if (fail_accel_read) return ESP_ERR_TIMEOUT;
        const uint8_t accel[] = {0x80, 0x3e, 0x80, 0xc1, 0x40, 0x1f};
        memcpy(buffer, accel, 6); // +1 g, -1 g, +0.5 g.
    } else {
        assert(device == &mag_token && *reg == MAG_OUT_X_H && length == 6);
        mag_reads++;
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
    assert(gy511_sensor_init() == ESP_OK && allocated == 2);
    gy511_data_t data;
    assert(gy511_sensor_read(&data) == ESP_OK);
    assert(data.accel_x_g == 1 && data.accel_y_g == -1 && data.accel_z_g == 0.5f);
    assert(data.mag_x_ut == 100 && data.mag_y_ut == -100 && data.mag_z_ut == 100);
    saturated = true;
    assert(gy511_sensor_read(&data) == ESP_ERR_INVALID_RESPONSE);
    saturated = false; fail_accel_read = true;
    unsigned previous_mag_reads = mag_reads;
    assert(gy511_sensor_read(&data) == ESP_ERR_TIMEOUT && mag_reads == previous_mag_reads);
    fail_accel_read = false;
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
    puts("PASS: GY-511 partial-init cleanup, task-failure cleanup, idempotence, removal failure");
    return 0;
}
