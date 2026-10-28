// The worker has its own fault tests. These reader/scheduler regressions verify
// the real GY-511 owner supplies coherent metadata without adding a mock task.
#include "platform.h"
#include "motion_ml.h"
#include <assert.h>
static uint32_t ml_sequence, ml_generation, ml_period=500;
static bool ml_enabled=true;
esp_err_t motion_ml_start(uint32_t boot_id, motion_result_callback_t callback)
{ (void)boot_id; (void)callback; return ESP_OK; }
bool motion_ml_submit(const motion_sample_t *sample)
{
    assert(sample->sequence == ++ml_sequence);
    assert(sample->generation == ml_generation && sample->interval_ms == ml_period && ml_enabled);
    assert(sample->boot_id == 0x12345678 && sample->acquired_us <= test_now_us);
    return true;
}
void motion_ml_configure(uint32_t ms, uint32_t generation, bool enabled)
{ assert(generation == ++ml_generation); ml_period=ms; ml_enabled=enabled; }
void motion_ml_set_sensor_available(bool available) { assert(available); }
uint32_t mqtt_manager_boot_id(void) { return 0x12345678; }
esp_err_t mqtt_manager_publish_motion(const motion_result_t *result) { (void)result; return ESP_OK; }
