
#include "bme280_sensor.h"
#include "i2c_bus_manager.h"

#include <stdint.h>
#include <stddef.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BME280_ADDR_LOW   0x76
#define BME280_ADDR_HIGH  0x77

#define REG_CALIB_TP      0x88  // 24 bytes: T1..T3, P1..P9
#define REG_CALIB_H1      0xA1
#define REG_ID            0xD0
#define REG_RESET         0xE0
#define REG_CALIB_H2      0xE1  // 7 bytes: H2..H6
#define REG_CTRL_HUM      0xF2
#define REG_STATUS        0xF3
#define REG_CTRL_MEAS     0xF4
#define REG_CONFIG        0xF5
#define REG_DATA          0xF7  // 8 bytes: press, temp, hum

#define CHIP_ID_BMP280    0x58
#define CHIP_ID_BME280    0x60
#define RESET_COMMAND     0xB6

#define STATUS_MEASURING  0x08
#define STATUS_IM_UPDATE  0x01

// x1 oversampling on all channels, forced mode.
#define CTRL_HUM_X1       0x01
#define CTRL_MEAS_FORCED  ((1 << 5) | (1 << 2) | 0x01)
// Filter off, standby unused in forced mode.
#define CONFIG_NO_FILTER  0x00

// Skipped-measurement markers from the datasheet.
#define RAW_SKIPPED_TP    0x80000
#define RAW_SKIPPED_H     0x8000

#define I2C_TIMEOUT_MS    100
#define I2C_SPEED_HZ      100000
// Forced x1 conversion is ~8 ms. Poll in 10 ms steps, bounded to 60 ms total.
#define POLL_STEP_MS      10
#define POLL_STEPS        6

static const char *TAG = "AAI16_BME280";

typedef struct {
    uint16_t t1;
    int16_t t2, t3;
    uint16_t p1;
    int16_t p2, p3, p4, p5, p6, p7, p8, p9;
    uint8_t h1, h3;
    int16_t h2, h4, h5;
    int8_t h6;
} calibration_t;

static i2c_master_dev_handle_t s_dev = NULL;
static calibration_t s_cal;
static bool s_has_humidity;

static void wait_ms(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks ? ticks : 1);
}

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    return i2c_master_transmit(s_dev, buffer, sizeof(buffer), I2C_TIMEOUT_MS);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buffer, size_t length)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buffer, length, I2C_TIMEOUT_MS);
}

static uint16_t le_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t le_s16(const uint8_t *p) { return (int16_t)le_u16(p); }

static void parse_calibration(const uint8_t tp[24], uint8_t h1, const uint8_t h[7],
                              calibration_t *c)
{
    c->t1 = le_u16(&tp[0]);
    c->t2 = le_s16(&tp[2]);
    c->t3 = le_s16(&tp[4]);
    c->p1 = le_u16(&tp[6]);
    c->p2 = le_s16(&tp[8]);
    c->p3 = le_s16(&tp[10]);
    c->p4 = le_s16(&tp[12]);
    c->p5 = le_s16(&tp[14]);
    c->p6 = le_s16(&tp[16]);
    c->p7 = le_s16(&tp[18]);
    c->p8 = le_s16(&tp[20]);
    c->p9 = le_s16(&tp[22]);
    c->h1 = h1;
    c->h2 = le_s16(&h[0]);
    c->h3 = h[2];
    // H4/H5 are signed 12-bit values packed across 0xE4..0xE6.
    c->h4 = (int16_t)(((int16_t)(int8_t)h[3] << 4) | (h[4] & 0x0F));
    c->h5 = (int16_t)(((int16_t)(int8_t)h[5] << 4) | (h[4] >> 4));
    c->h6 = (int8_t)h[6];
}

// Bosch integer compensation from the BME280 datasheet, section 4.2.3.
static int32_t compensate_temperature(const calibration_t *c, int32_t adc_t, int32_t *t_fine)
{
    int32_t var1 = (((adc_t >> 3) - ((int32_t)c->t1 << 1)) * (int32_t)c->t2) >> 11;
    int32_t var2 = (((((adc_t >> 4) - (int32_t)c->t1) * ((adc_t >> 4) - (int32_t)c->t1)) >> 12) *
                    (int32_t)c->t3) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;  // 0.01 degC
}

// Returns Pa in Q24.8 format; 0 means the calibration would divide by zero.
static uint32_t compensate_pressure(const calibration_t *c, int32_t adc_p, int32_t t_fine)
{
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)c->p6;
    var2 = var2 + ((var1 * (int64_t)c->p5) << 17);
    var2 = var2 + (((int64_t)c->p4) << 35);
    var1 = ((var1 * var1 * (int64_t)c->p3) >> 8) + ((var1 * (int64_t)c->p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)c->p1) >> 33;
    if (var1 == 0) return 0;
    int64_t p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)c->p9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)c->p8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)c->p7 << 4);
    return (uint32_t)p;
}

// Returns %RH in Q22.10 format.
static uint32_t compensate_humidity(const calibration_t *c, int32_t adc_h, int32_t t_fine)
{
    int32_t v = t_fine - 76800;
    v = (((((adc_h << 14) - ((int32_t)c->h4 << 20) - ((int32_t)c->h5 * v)) + 16384) >> 15) *
         (((((((v * (int32_t)c->h6) >> 10) * (((v * (int32_t)c->h3) >> 11) + 32768)) >> 10) +
            2097152) * (int32_t)c->h2 + 8192) >> 14));
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)c->h1) >> 4);
    v = v < 0 ? 0 : v;
    v = v > 419430400 ? 419430400 : v;
    return (uint32_t)(v >> 12);
}

// Initialize BME280/BMP280.
esp_err_t bme280_sensor_init(void)
{
    if (s_dev != NULL) {
        return ESP_OK;
    }

    i2c_master_bus_handle_t bus = i2c_bus_manager_get_handle();
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus is not initialized.");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_ERR_NOT_FOUND;
    uint8_t id = 0;
    static const uint8_t addresses[] = {BME280_ADDR_LOW, BME280_ADDR_HIGH};
    uint8_t address = 0;

    for (unsigned i = 0; i < sizeof(addresses); ++i) {
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addresses[i],
            .scl_speed_hz = I2C_SPEED_HZ,
        };
        esp_err_t add_err = i2c_master_bus_add_device(bus, &cfg, &s_dev);
        if (add_err != ESP_OK) {
            ESP_LOGE(TAG, "Cannot add device at 0x%02X.", addresses[i]);
            err = add_err;
            goto fail;
        }
        err = read_regs(REG_ID, &id, 1);
        if (err == ESP_OK) {
            address = addresses[i];
            break;
        }
        // Nothing answered here; release the handle and try the other address.
        esp_err_t rm_err = i2c_master_bus_rm_device(s_dev);
        if (rm_err != ESP_OK) {
            err = rm_err;
            goto fail;
        }
        s_dev = NULL;
    }

    if (s_dev == NULL) {
        ESP_LOGE(TAG, "No BME280/BMP280 at 0x76 or 0x77. Check wiring and SDO.");
        return err;
    }

    if (id != CHIP_ID_BME280 && id != CHIP_ID_BMP280) {
        ESP_LOGE(TAG, "Unexpected chip ID 0x%02X.", id);
        err = ESP_ERR_INVALID_RESPONSE;
        goto fail;
    }
    s_has_humidity = id == CHIP_ID_BME280;

    // Soft reset, then wait for the NVM calibration copy to finish.
    err = write_reg(REG_RESET, RESET_COMMAND);
    if (err != ESP_OK) goto fail;
    wait_ms(10);

    uint8_t status = STATUS_IM_UPDATE;
    for (unsigned i = 0; i < POLL_STEPS && (status & STATUS_IM_UPDATE); ++i) {
        err = read_regs(REG_STATUS, &status, 1);
        if (err != ESP_OK) goto fail;
        if (status & STATUS_IM_UPDATE) wait_ms(POLL_STEP_MS);
    }
    if (status & STATUS_IM_UPDATE) {
        err = ESP_ERR_TIMEOUT;
        goto fail;
    }

    uint8_t tp[24] = {0}, h1 = 0, h[7] = {0};
    err = read_regs(REG_CALIB_TP, tp, sizeof(tp));
    if (err != ESP_OK) goto fail;
    if (s_has_humidity) {
        err = read_regs(REG_CALIB_H1, &h1, 1);
        if (err != ESP_OK) goto fail;
        err = read_regs(REG_CALIB_H2, h, sizeof(h));
        if (err != ESP_OK) goto fail;
    }
    parse_calibration(tp, h1, h, &s_cal);

    // Without calibration the compensation would divide by zero or return junk.
    if (s_cal.t1 == 0 || s_cal.p1 == 0) {
        ESP_LOGE(TAG, "Calibration data is blank.");
        err = ESP_ERR_INVALID_RESPONSE;
        goto fail;
    }

    err = write_reg(REG_CONFIG, CONFIG_NO_FILTER);
    if (err != ESP_OK) goto fail;

    ESP_LOGI(TAG, "%s found at 0x%02X (ID 0x%02X)",
             s_has_humidity ? "BME280" : "BMP280 (no humidity)", address, id);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "BME280 initialization failed: %s", esp_err_to_name(err));
    esp_err_t cleanup_err = bme280_sensor_deinit();
    if (cleanup_err != ESP_OK) {
        ESP_LOGE(TAG, "Device cleanup failed: %s", esp_err_to_name(cleanup_err));
    }
    return err;
}

esp_err_t bme280_sensor_deinit(void)
{
    if (s_dev == NULL) {
        return ESP_OK;
    }
    esp_err_t err = i2c_master_bus_rm_device(s_dev);
    if (err == ESP_OK) s_dev = NULL;
    return err;
}

// Trigger one forced measurement and read the compensated result.
esp_err_t bme280_sensor_read(bme280_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // ctrl_hum only takes effect after a write to ctrl_meas.
    esp_err_t err = write_reg(REG_CTRL_HUM, CTRL_HUM_X1);
    if (err != ESP_OK) return err;
    err = write_reg(REG_CTRL_MEAS, CTRL_MEAS_FORCED);
    if (err != ESP_OK) return err;

    uint8_t status = STATUS_MEASURING;
    for (unsigned i = 0; i < POLL_STEPS && (status & STATUS_MEASURING); ++i) {
        wait_ms(POLL_STEP_MS);
        err = read_regs(REG_STATUS, &status, 1);
        if (err != ESP_OK) return err;
    }
    if (status & STATUS_MEASURING) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t raw[8] = {0};
    err = read_regs(REG_DATA, raw, sizeof(raw));
    if (err != ESP_OK) return err;

    int32_t adc_p = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_t = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_h = ((int32_t)raw[6] << 8) | raw[7];

    if (adc_t == RAW_SKIPPED_TP || adc_p == RAW_SKIPPED_TP ||
        (s_has_humidity && adc_h == RAW_SKIPPED_H)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    int32_t t_fine = 0;
    int32_t temp_centi = compensate_temperature(&s_cal, adc_t, &t_fine);
    uint32_t press_q8 = compensate_pressure(&s_cal, adc_p, t_fine);
    float temperature_c = (float)temp_centi / 100.0f;
    float pressure_hpa = (float)press_q8 / 256.0f / 100.0f;

    // Datasheet operating range: -40..85 degC, 300..1100 hPa.
    if (press_q8 == 0 || temperature_c < -40.0f || temperature_c > 85.0f ||
        pressure_hpa < 300.0f || pressure_hpa > 1100.0f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    data->temperature_c = temperature_c;
    data->pressure_hpa = pressure_hpa;
    data->has_humidity = s_has_humidity;
    data->humidity_pct = 0.0f;
    if (s_has_humidity) {
        data->humidity_pct = (float)compensate_humidity(&s_cal, adc_h, t_fine) / 1024.0f;
    }
    return ESP_OK;
}
