#include "platform.h"
#include <assert.h>
#include <string.h>
#include "../../components/spi_bus_manager/spi_bus_manager.c"

static unsigned init_calls, added, removed, queued;
static esp_err_t init_error, add_error, queue_error, result_error, remove_error;
static int tokens[2];
static spi_transaction_t *in_flight[2];
static bool delayed[2];

esp_err_t spi_bus_initialize(int host, const spi_bus_config_t *config, int dma)
{
    init_calls++;
    assert(host == SPI2_HOST && dma == SPI_DMA_DISABLED);
    assert(config->sclk_io_num == 12 && config->mosi_io_num == 11 && config->miso_io_num == 13);
    assert(config->quadwp_io_num == -1 && config->quadhd_io_num == -1 && config->max_transfer_sz == 4);
    assert(config->data4_io_num == -1 && config->data7_io_num == -1);
    return init_error;
}
esp_err_t spi_bus_add_device(int host, const spi_device_interface_config_t *config, spi_device_handle_t *handle)
{
    assert(host == SPI2_HOST && config->queue_size == 1 && config->mode == 0);
    if (add_error) return add_error;
    assert(added < 2 && config->spics_io_num == (added ? 10 : 14));
    assert(config->clock_speed_hz == (added ? 2000000 : 1000000));
    *handle = &tokens[added++]; return ESP_OK;
}
esp_err_t spi_bus_remove_device(spi_device_handle_t handle)
{
    if (remove_error) return remove_error;
    removed++; return ESP_OK;
}
esp_err_t spi_device_queue_trans(spi_device_handle_t handle, spi_transaction_t *transaction, TickType_t wait)
{
    assert(wait == 0 && transaction->flags == (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA));
    assert(transaction->length > 0 && transaction->length <= 32);
    if (queue_error) return queue_error;
    unsigned id = handle == &tokens[0] ? 0 : 1;
    assert(!in_flight[id]); in_flight[id] = transaction; queued++;
    return ESP_OK;
}
esp_err_t spi_device_get_trans_result(spi_device_handle_t handle, spi_transaction_t **transaction, TickType_t wait)
{
    assert(wait == 0 || wait == pdMS_TO_TICKS(20));
    unsigned id = handle == &tokens[0] ? 0 : 1;
    if (delayed[id]) return ESP_ERR_TIMEOUT;
    assert(in_flight[id]);
    *transaction = in_flight[id];
    for (unsigned i = 0; i < 4; ++i) (*transaction)->rx_data[i] = (*transaction)->tx_data[i] ^ 0xff;
    in_flight[id] = NULL;
    return result_error;
}

int main(void)
{
    spi_bus_device_t rc = {0}, thermocouple = {0};
    assert(spi_bus_manager_add_device(&rc, 14, 1000000, 0) == ESP_ERR_INVALID_STATE);
    init_error = ESP_ERR_NO_MEM;
    assert(spi_bus_manager_init() == init_error);
    init_error = ESP_OK;
    assert(spi_bus_manager_init() == ESP_OK && spi_bus_manager_init() == ESP_OK && init_calls == 2);
    add_error = ESP_ERR_NO_MEM;
    assert(spi_bus_manager_add_device(&rc, 14, 1000000, 0) == add_error && !rc.handle);
    add_error = ESP_OK;
    assert(spi_bus_manager_add_device(&rc, 14, 1000000, 0) == ESP_OK);
    assert(spi_bus_manager_add_device(&thermocouple, 10, 2000000, 0) == ESP_OK && init_calls == 2);
    uint8_t tx[] = {0xee, 0}, rx[] = {0x55, 0x55};
    assert(spi_bus_manager_transfer(&rc, tx, rx, 2) == ESP_OK && rx[0] == 0x11 && rx[1] == 0xff);
    assert(spi_bus_manager_transfer(&rc, tx, rx, 5) == ESP_ERR_INVALID_ARG);
    queue_error = ESP_ERR_TIMEOUT;
    assert(spi_bus_manager_transfer(&rc, tx, rx, 2) == queue_error && !rc.pending);
    queue_error = ESP_OK; delayed[0] = true;
    rx[0] = 0x55;
    assert(spi_bus_manager_transfer(&rc, tx, rx, 2) == ESP_ERR_TIMEOUT && rc.pending && rx[0] == 0x55);
    unsigned old_queued = queued;
    tx[0] = 0x11; // Caller memory changes while SDK still owns the copied descriptor.
    assert(in_flight[0] == &rc.transaction && in_flight[0]->tx_data[0] == 0xee);
    assert(spi_bus_manager_transfer(&rc, tx, rx, 2) == ESP_ERR_TIMEOUT && queued == old_queued);
    assert(spi_bus_manager_remove_device(&rc) == ESP_ERR_TIMEOUT && removed == 0);
    // Another device uses the shared bus independently; no bus reset on RC522 timeout.
    assert(spi_bus_manager_transfer(&thermocouple, tx, rx, 2) == ESP_OK);
    delayed[0] = false;
    assert(spi_bus_manager_transfer(&rc, tx, rx, 2) == ESP_OK && !rc.pending && rx[0] == 0xee);
    result_error = ESP_ERR_INVALID_STATE;
    assert(spi_bus_manager_transfer(&rc, tx, rx, 2) == result_error && !rc.pending);
    result_error = ESP_OK; remove_error = ESP_FAIL;
    assert(spi_bus_manager_remove_device(&rc) == ESP_FAIL && rc.handle);
    remove_error = ESP_OK;
    assert(spi_bus_manager_remove_device(&rc) == ESP_OK && !rc.handle);
    assert(spi_bus_manager_remove_device(&rc) == ESP_OK && removed == 1);
    assert(thermocouple.handle && spi_bus_manager_transfer(&thermocouple, tx, rx, 2) == ESP_OK);
    puts("PASS: shared SPI initialization, per-device CS/clock, bounded transfers, late completion lifetime, cleanup");
    return 0;
}
