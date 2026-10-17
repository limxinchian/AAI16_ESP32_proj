#pragma once

#include <stddef.h>
#include "mqtt_manager.h"

// One notification word carries exactly one command: validated intervals are
// <= 10000; the high bit tags enable/disable and bit 0 holds its Boolean value.
// Keeping both operations on index 0 preserves their application order.
#define CONFIG_ENABLED_FLAG (UINT32_C(1) << 31)

// Call in the acquisition owner, after older commands have committed. A
// dispatcher snapshot cannot safely decide the disabled-interval policy.
static inline bool config_can_apply(const mqtt_config_update_t *update, sensor_status_t *status)
{
    const char *reason = NULL;
    if (!sensor_status_get(update->sensor, status) || !status->initialized ||
        status->state == SENSOR_UNAVAILABLE || status->state == SENSOR_UNINITIALIZED) {
        reason = "sensor_unavailable";
    } else if (update->operation == MQTT_CONFIG_INTERVAL && status->state == SENSOR_DISABLED) {
        reason = "sensor_disabled";
    }
    if (reason != NULL) (void)mqtt_manager_reject_config(update, reason);
    return reason == NULL;
}
