
#pragma once

#include "esp_err.h"

// Start the Wi-Fi station in the background.
// ESP_OK means startup succeeded, not that an IP was obtained.
esp_err_t wifi_manager_start(void);
