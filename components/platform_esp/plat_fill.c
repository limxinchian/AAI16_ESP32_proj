/* ESP-IDF glue: plat_fill() fills the node_platform_t with the real implementations (clock, free heap, reboot, log line,
 * bus drivers, storage, rule outputs, network publish, TFLM inference) so node_core stays platform independent. */
#include <stdio.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "platform_esp.h"

/* implemented in plat_ml.cc */
aai16_ModelReason plat_ml_load(void *ctx, const ml_package_t *pkg);
bool plat_ml_invoke(void *ctx, const int8_t *in, uint32_t n_in, int8_t *out, uint32_t n_out, uint32_t *micros);

static uint64_t p_now(void *c) { (void)c; return (uint64_t)esp_timer_get_time(); }
static uint32_t p_heap(void *c) { (void)c; return (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL); }
#include "esp_system.h"
static void p_reboot(void *c) { (void)c; esp_restart(); }
static void p_log(void *c, const char *line) { (void)c; ESP_LOGW("node", "%s", line); }

void plat_fill(node_platform_t *pf, const char *node_id)
{
    *pf = (node_platform_t){0};
    pf->node_id = node_id;
    pf->bus = plat_bus_backend();
    plat_storage_fill(pf);
    plat_io_fill(pf);
    pf->publish = plat_net_publish;
    pf->ml_load = plat_ml_load;
    pf->ml_invoke = plat_ml_invoke;
    pf->now_us = p_now;
    pf->free_heap = p_heap;
    pf->log = p_log;
    pf->reboot = p_reboot;
}
