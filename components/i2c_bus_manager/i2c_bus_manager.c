
#include "i2c_bus_manager.h"

#include "esp_log.h"

#define I2C_SDA_PIN GPIO_NUM_8
#define I2C_SCL_PIN GPIO_NUM_9

static const char *TAG = "AAI16_I2C";

static i2c_master_bus_handle_t s_bus = NULL;


// --------------------------------------------------
// Initialize shared I2C bus
// --------------------------------------------------

esp_err_t i2c_bus_manager_init(void)
{
    if (s_bus != NULL) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(
        &bus_config,
        &s_bus
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C bus initialized.");
    ESP_LOGI(TAG, "SDA: GPIO8 | SCL: GPIO9");

    return ESP_OK;
}


// --------------------------------------------------
// Scan connected I2C devices
// --------------------------------------------------

void i2c_bus_manager_scan(void)
{
    if (s_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized.");
        return;
    }

    ESP_LOGI(TAG, "========== I2C SCAN ==========");

    int devices_found = 0;

    for (int address = 0x08; address <= 0x77; address++) {

        esp_err_t err = i2c_master_probe(
            s_bus,
            address,
            50
        );

        if (err == ESP_OK) {

            ESP_LOGI(TAG, "Device found at 0x%02X",
                     address);

            devices_found++;

        } else if (err == ESP_ERR_TIMEOUT) {

            ESP_LOGW(TAG,
                     "I2C timeout. Check wiring and pull-ups.");
            break;
        }
    }

    ESP_LOGI(TAG, "Devices found: %d", devices_found);
    ESP_LOGI(TAG, "========== SCAN END ==========");
}


// --------------------------------------------------
// Return shared I2C bus handle
// --------------------------------------------------

i2c_master_bus_handle_t i2c_bus_manager_get_handle(void)
{
    return s_bus;
}
