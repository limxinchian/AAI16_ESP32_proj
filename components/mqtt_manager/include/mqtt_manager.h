
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t mqtt_manager_init(void);

esp_err_t mqtt_manager_publish_dht22(
    float temperature_c,
    float humidity_pct
);

// Publish an ACK after applying a configuration.
esp_err_t mqtt_manager_publish_config_ack(
    uint32_t interval_ms
);

// Retrieve a validated interval command, if available.
bool mqtt_manager_take_interval_update(uint32_t *out_ms);
