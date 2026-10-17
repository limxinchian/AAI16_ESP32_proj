#include "platform.h"
#include <assert.h>
#include <string.h>
#include "../../components/gy511_sensor/gy511_sensor.c"

static int accel_token, mag_token, bus_token;
static unsigned allocated, adds, removes;
static bool fail_second_add, fail_id, fail_remove_mag;

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
    assert(device == &mag_token && reg_length == 1 && *reg == MAG_ID_REG);
    assert(timeout_ms == 100 && length == 3);
    if (fail_id) return ESP_ERR_TIMEOUT;
    memcpy(buffer, "H43", 3); return ESP_OK;
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
    assert(gy511_sensor_deinit() == ESP_OK && allocated == 0);
    unsigned previous_removes = removes;
    assert(gy511_sensor_deinit() == ESP_OK && removes == previous_removes);
    gy511_data_t data;
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
