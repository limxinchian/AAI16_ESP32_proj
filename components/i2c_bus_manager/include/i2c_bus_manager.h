
#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

// Initialize shared I2C bus.
esp_err_t i2c_bus_manager_init(void);

// Scan the bus for connected I2C devices.
void i2c_bus_manager_scan(void);

// Allow future sensor drivers to use this bus.
i2c_master_bus_handle_t i2c_bus_manager_get_handle(void);
