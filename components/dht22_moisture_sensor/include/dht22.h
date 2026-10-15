#pragma once

#include "esp_err.h"
#include "driver/gpio.h"

typedef struct {
    float temperature_c;
    float humidity_pct;
} dht22_data_t;

esp_err_t dht22_init(gpio_num_t gpio);

esp_err_t dht22_read(dht22_data_t *data);