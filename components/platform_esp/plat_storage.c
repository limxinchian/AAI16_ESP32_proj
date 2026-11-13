/* Persistent storage: NVS blobs (committed config, model metadata) and the two model-bank flash
 * partitions. NOT hardware-verified. */
#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "platform_esp.h"

static const char *TAG = "plat_store";
#define NS "aai16"

/* ---------------------------------------------------------------- NVS */
static bool nvs_put(const char *key, const void *data, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = nvs_set_blob(h, key, data, len);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK) ESP_LOGE(TAG, "nvs write %s failed: %s", key, esp_err_to_name(e));
    else ESP_LOGI(TAG, "nvs wrote %s (%u bytes)", key, (unsigned)len);
    return e == ESP_OK;
}

static bool nvs_get(const char *key, void *buf, size_t cap, size_t *len)
{
    nvs_handle_t h;
    esp_err_t oe = nvs_open(NS, NVS_READONLY, &h);
    if (oe != ESP_OK) { ESP_LOGI(TAG, "nvs open for read: %s", esp_err_to_name(oe)); return false; }
    size_t n = cap;
    const esp_err_t e = nvs_get_blob(h, key, buf, &n);
    nvs_close(h);
    if (e != ESP_OK) { ESP_LOGI(TAG, "nvs read %s: %s", key, esp_err_to_name(e)); return false; }
    ESP_LOGI(TAG, "nvs read %s (%u bytes)", key, (unsigned)n);
    if (len) *len = n;
    return true;
}

static bool s_persist_config(void *c, const uint8_t *blob, size_t len) { (void)c; return nvs_put("cfg", blob, len); }
static bool s_load_config(void *c, uint8_t *buf, size_t cap, size_t *len) { (void)c; return nvs_get("cfg", buf, cap, len); }

static bool s_persist_state(void *c, const uint8_t *blob, size_t len) { (void)c; return nvs_put("state", blob, len); }
static bool s_load_state(void *c, uint8_t *buf, size_t cap, size_t *len) { (void)c; return nvs_get("state", buf, cap, len); }

static bool s_save_meta(void *c, const model_meta_t *m) { (void)c; return nvs_put("meta", m, sizeof(*m)); }
static bool s_load_meta(void *c, model_meta_t *m)
{
    (void)c;
    size_t n = 0;
    return nvs_get("meta", m, sizeof(*m), &n) && n == sizeof(*m);
}

/* ---------------------------------------------------------- model banks */
static struct {
    const esp_partition_t *part;
    const void *map_ptr;
    size_t mapped_len;
    esp_partition_mmap_handle_t handle;
} s_bank[2];

static const esp_partition_t *bank_part(uint8_t b)
{
    if (b > 1) return NULL;
    if (!s_bank[b].part) {
        s_bank[b].part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)(0x40 + b), NULL);
        if (!s_bank[b].part) ESP_LOGE(TAG, "partition bank%u not found", b);
    }
    return s_bank[b].part;
}

static void bank_unmap(uint8_t b)
{
    if (s_bank[b].map_ptr) {
        esp_partition_munmap(s_bank[b].handle);
        s_bank[b].map_ptr = NULL;
        s_bank[b].mapped_len = 0;
    }
}

static bool s_erase(void *c, uint8_t b, size_t size)
{
    (void)c;
    const esp_partition_t *p = bank_part(b);
    if (!p) return false;
    bank_unmap(b);
    const size_t sec = p->erase_size;
    size_t n = ((size + sec - 1) / sec) * sec;
    if (n > p->size) n = p->size;
    return esp_partition_erase_range(p, 0, n) == ESP_OK;
}

static bool s_write(void *c, uint8_t b, uint32_t off, const uint8_t *data, size_t len)
{
    (void)c;
    const esp_partition_t *p = bank_part(b);
    bank_unmap(b); /* a cached mapping would be stale after writes */
    return p && esp_partition_write(p, off, data, len) == ESP_OK;
}

static const uint8_t *s_map(void *c, uint8_t b, size_t len)
{
    (void)c;
    const esp_partition_t *p = bank_part(b);
    if (!p || len == 0 || len > p->size) return NULL;
    if (s_bank[b].map_ptr && s_bank[b].mapped_len >= len) return s_bank[b].map_ptr;
    bank_unmap(b);
    const void *ptr = NULL;
    if (esp_partition_mmap(p, 0, len, ESP_PARTITION_MMAP_DATA, &ptr, &s_bank[b].handle) != ESP_OK) return NULL;
    s_bank[b].map_ptr = ptr;
    s_bank[b].mapped_len = len;
    return ptr;
}

void plat_storage_fill(node_platform_t *pf)
{
    pf->persist_config = s_persist_config;
    pf->load_config = s_load_config;
    pf->persist_state = s_persist_state;
    pf->load_state = s_load_state;
    const esp_partition_t *p0 = bank_part(0);
    pf->store = (model_store_ops_t){s_erase, s_write, s_map, s_load_meta, s_save_meta, NULL, p0 ? (uint32_t)p0->size : 0};
}
