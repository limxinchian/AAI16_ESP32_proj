#pragma once

#include <stddef.h>
#include "mqtt_manager.h"

// Notification index 0 wakes the owner; the bounded sidecar carries identity,
// generations and the authoritative operation. The word remains a bounded hint:
// intervals are <= 60000; the high bit tags enable/disable and bit 0 its value.
// Keeping both operations on index 0 preserves their application order.
#define CONFIG_ENABLED_FLAG (UINT32_C(1) << 31)

// Call in the acquisition owner, after older commands have committed. A
// dispatcher snapshot cannot safely decide the disabled-interval policy.
static inline bool config_can_apply(mqtt_config_update_t *update, sensor_status_t *status)
{
    if (!mqtt_manager_begin_owner_config(update)) return false;
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
