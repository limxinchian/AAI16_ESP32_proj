// Protocol/transaction behavior is tested with the real manager in mqtt_host.
// These doubles preserve the actual owners' scheduling and sidecar handoff.
#include <assert.h>
#include "mqtt_manager.h"
static mqtt_config_update_t pending[SENSOR_STATUS_CAPACITY];
static bool occupied[SENSOR_STATUS_CAPACITY];
unsigned test_config_sidecar_busy;
static uint32_t generation[SENSOR_STATUS_CAPACITY];
bool mqtt_manager_stage_owner_config(const mqtt_config_update_t *u)
{
    if (occupied[u->sensor]) { ++test_config_sidecar_busy; return false; }
    pending[u->sensor] = *u; occupied[u->sensor] = true; return true;
}
void mqtt_manager_cancel_owner_config(sensor_id_t sensor) { occupied[sensor] = false; }
bool mqtt_manager_take_owner_config(sensor_id_t sensor, mqtt_config_update_t *u)
{
    if (!occupied[sensor]) return false;
    *u = pending[sensor]; occupied[sensor] = false; return true;
}
bool mqtt_manager_begin_owner_config(mqtt_config_update_t *u)
{
    u->generation = generation[u->sensor] + 1; return true;
}
void test_config_committed(const mqtt_config_update_t *u)
{
    assert(u->generation == generation[u->sensor] + 1);
    generation[u->sensor] = u->generation;
}
