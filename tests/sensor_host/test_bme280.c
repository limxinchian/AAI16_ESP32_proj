#include "platform.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include "../../components/bme280_sensor/bme280_sensor.c"

// Register-file model of the sensor behind two possible I2C addresses.
static int tokens[2], bus_token;
static unsigned allocated, delays;
static uint8_t regs[256];
static unsigned present_address;   // 0 = nothing answers
static unsigned measuring_polls;   // status reads that still report "measuring"
static bool stuck_measuring, reg_read_issued[256];

static unsigned address_of(i2c_master_dev_handle_t device)
{
    return 0x76u + (unsigned)((int *)device - tokens);
}

void vTaskDelay(TickType_t ticks) { delays++; (void)ticks; }
i2c_master_bus_handle_t i2c_bus_manager_get_handle(void) { return &bus_token; }
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
    const i2c_device_config_t *config, i2c_master_dev_handle_t *device)
{
    assert(bus == &bus_token && config->scl_speed_hz == 100000);
    assert(config->device_address == 0x76 || config->device_address == 0x77);
    *device = &tokens[config->device_address - 0x76];
    allocated++;
    return ESP_OK;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
    assert(allocated > 0); allocated--;
    return ESP_OK;
}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device,
    const uint8_t *buffer, size_t length, int timeout_ms)
{
    assert(timeout_ms == 100 && length == 2);
    if (present_address != address_of(device)) return ESP_ERR_TIMEOUT;
    regs[buffer[0]] = buffer[1];
    if (buffer[0] == REG_CTRL_MEAS && (buffer[1] & 3) == 1) measuring_polls = 2;
    return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
    const uint8_t *reg, size_t reg_length, uint8_t *buffer, size_t length, int timeout_ms)
{
    assert(reg_length == 1 && timeout_ms == 100);
    if (present_address != address_of(device)) return ESP_ERR_TIMEOUT;
    reg_read_issued[*reg] = true;
    for (size_t i = 0; i < length; ++i) buffer[i] = regs[*reg + i];
    if (*reg == REG_STATUS) {
        if (stuck_measuring) buffer[0] = STATUS_MEASURING;
        else if (measuring_polls) { measuring_polls--; buffer[0] = STATUS_MEASURING; }
    }
    return ESP_OK;
}

static void put16(uint8_t reg, uint16_t value) { regs[reg] = value & 0xff; regs[reg + 1] = value >> 8; }

// Bosch BMP280/BME280 datasheet calibration example (25.08 degC, 100653.27 Pa),
// plus typical BME280 humidity coefficients in their packed register layout.
static void load_sensor(uint8_t id, int32_t adc_t, int32_t adc_p, int32_t adc_h)
{
    memset(regs, 0, sizeof(regs));
    memset(reg_read_issued, 0, sizeof(reg_read_issued));
    regs[REG_ID] = id;
    put16(0x88, 27504); put16(0x8A, 26435); put16(0x8C, (uint16_t)-1000);
    put16(0x8E, 36477); put16(0x90, (uint16_t)-10685); put16(0x92, 3024);
    put16(0x94, 2855);  put16(0x96, 140);   put16(0x98, (uint16_t)-7);
    put16(0x9A, 15500); put16(0x9C, (uint16_t)-14600); put16(0x9E, 6000);
    regs[REG_CALIB_H1] = 75;
    put16(REG_CALIB_H2, 362);       // H2
    regs[0xE3] = 0;                 // H3
    regs[0xE4] = 0x13; regs[0xE5] = 0x2C; regs[0xE6] = 0x03; // H4=316, H5=50
    regs[0xE7] = 30;                // H6
    regs[REG_DATA + 0] = adc_p >> 12; regs[REG_DATA + 1] = adc_p >> 4; regs[REG_DATA + 2] = (adc_p << 4) & 0xf0;
    regs[REG_DATA + 3] = adc_t >> 12; regs[REG_DATA + 4] = adc_t >> 4; regs[REG_DATA + 5] = (adc_t << 4) & 0xf0;
    regs[REG_DATA + 6] = adc_h >> 8;  regs[REG_DATA + 7] = adc_h;
}

// Independent floating-point reference (Bosch BME280 driver, double formula).
static double reference_humidity(const calibration_t *c, int32_t t_fine, int32_t adc_h)
{
    double v = (double)t_fine - 76800.0;
    v = (adc_h - ((double)c->h4 * 64.0 + (double)c->h5 / 16384.0 * v)) *
        ((double)c->h2 / 65536.0 * (1.0 + (double)c->h6 / 67108864.0 * v *
        (1.0 + (double)c->h3 / 67108864.0 * v)));
    v = v * (1.0 - (double)c->h1 * v / 524288.0);
    return v < 0 ? 0 : v > 100 ? 100 : v;
}

int main(void)
{
    bme280_data_t d;
    const int32_t adc_t = 519888, adc_p = 415148, adc_h = 30000;

    // Nothing answers: both addresses probed, no handle leaked.
    load_sensor(0x60, adc_t, adc_p, adc_h); present_address = 0;
    assert(bme280_sensor_init() == ESP_ERR_TIMEOUT && allocated == 0 && !s_dev);

    // BME280 strapped to 0x77: 0x76 is probed first, released, then 0x77 kept.
    present_address = 0x77;
    assert(bme280_sensor_init() == ESP_OK && allocated == 1 && s_has_humidity);
    assert(regs[REG_RESET] == RESET_COMMAND && regs[REG_CONFIG] == CONFIG_NO_FILTER);
    assert(s_cal.h4 == 316 && s_cal.h5 == 50 && s_cal.h2 == 362 && s_cal.h6 == 30 && s_cal.h1 == 75);
    assert(bme280_sensor_init() == ESP_OK && allocated == 1); // Idempotent.

    assert(bme280_sensor_read(&d) == ESP_OK);
    assert(regs[REG_CTRL_HUM] == CTRL_HUM_X1 && regs[REG_CTRL_MEAS] == CTRL_MEAS_FORCED);
    assert(delays > 0 && measuring_polls == 0);
    assert(fabsf(d.temperature_c - 25.08f) < 0.005f);
    assert(fabsf(d.pressure_hpa - 1006.5327f) < 0.01f);
    int32_t t_fine; compensate_temperature(&s_cal, adc_t, &t_fine);
    assert(d.has_humidity && fabs(d.humidity_pct - reference_humidity(&s_cal, t_fine, adc_h)) < 0.1);
    assert(d.humidity_pct > 0.0f && d.humidity_pct < 100.0f);

    // Datasheet example: 25.08 degC (2508 centi-degC exactly) and 100653.27 Pa
    // from the floating-point formula. Bosch's integer formula rounds slightly
    // differently (about 0.013 Pa lower), so allow 0.05 Pa in Q24.8 counts.
    assert(compensate_temperature(&s_cal, adc_t, &t_fine) == 2508);
    int64_t q8_error = (int64_t)compensate_pressure(&s_cal, adc_p, t_fine) - 25767237;
    assert(q8_error >= -13 && q8_error <= 13);

    // Humidity clamps instead of leaving the 0..100 range.
    assert(compensate_humidity(&s_cal, 0, t_fine) == 0);
    assert(compensate_humidity(&s_cal, 0xFFFF, t_fine) / 1024.0 <= 100.0);

    // The skipped-measurement marker is invalid, never a reading.
    load_sensor(0x60, RAW_SKIPPED_TP, adc_p, adc_h);
    assert(bme280_sensor_read(&d) == ESP_ERR_INVALID_RESPONSE);
    load_sensor(0x60, adc_t, RAW_SKIPPED_TP, adc_h);
    assert(bme280_sensor_read(&d) == ESP_ERR_INVALID_RESPONSE);
    load_sensor(0x60, adc_t, adc_p, RAW_SKIPPED_H);
    assert(bme280_sensor_read(&d) == ESP_ERR_INVALID_RESPONSE);

    // A conversion that never finishes times out in bounded steps.
    load_sensor(0x60, adc_t, adc_p, adc_h);
    stuck_measuring = true; delays = 0;
    assert(bme280_sensor_read(&d) == ESP_ERR_TIMEOUT && delays == POLL_STEPS);
    stuck_measuring = false;

    assert(bme280_sensor_read(NULL) == ESP_ERR_INVALID_ARG);
    assert(bme280_sensor_deinit() == ESP_OK && allocated == 0);
    assert(bme280_sensor_deinit() == ESP_OK && allocated == 0);
    assert(bme280_sensor_read(&d) == ESP_ERR_INVALID_STATE);

    // BMP280 (ID 0x58) at 0x76: no humidity calibration read, no humidity value.
    load_sensor(0x58, adc_t, adc_p, adc_h); present_address = 0x76;
    assert(bme280_sensor_init() == ESP_OK && !s_has_humidity && allocated == 1);
    assert(!reg_read_issued[REG_CALIB_H1] && !reg_read_issued[REG_CALIB_H2]);
    assert(bme280_sensor_read(&d) == ESP_OK && !d.has_humidity);
    assert(fabsf(d.temperature_c - 25.08f) < 0.005f);
    assert(bme280_sensor_deinit() == ESP_OK);

    // Wrong chip and blank calibration are rejected without leaking the handle.
    load_sensor(0x55, adc_t, adc_p, adc_h);
    assert(bme280_sensor_init() == ESP_ERR_INVALID_RESPONSE && allocated == 0 && !s_dev);
    load_sensor(0x60, adc_t, adc_p, adc_h); put16(0x88, 0);
    assert(bme280_sensor_init() == ESP_ERR_INVALID_RESPONSE && allocated == 0 && !s_dev);

    puts("PASS: BME280 address probe, datasheet compensation, BMP280 variant, invalid data, bounded waits, cleanup");
    return 0;
}
