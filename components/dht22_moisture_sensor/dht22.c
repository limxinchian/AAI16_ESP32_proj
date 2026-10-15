
// #include "dht22.h"
// #include "driver/rmt_rx.h"
// #include "esp_log.h"

// static const char *TAG = "DHT22";
// static rmt_channel_handle_t s_rx_channel = NULL;

// esp_err_t dht22_init(gpio_num_t gpio)
// {
//     if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     if (s_rx_channel != NULL) {
//         return ESP_ERR_INVALID_STATE;
//     }

//     // Configure RMT receiver: 1 tick = 1 microsecond.
//     rmt_rx_channel_config_t rx_config = {
//         .gpio_num = gpio,
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//         .resolution_hz = 1000000,
//         .mem_block_symbols = 48,
//     };

//     esp_err_t err = rmt_new_rx_channel(
//         &rx_config, &s_rx_channel
//     );

//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "RMT setup failed: %s",
//                  esp_err_to_name(err));
//         return err;
//     }

//     err = rmt_enable(s_rx_channel);

//     if (err != ESP_OK) {
//         rmt_del_channel(s_rx_channel);
//         s_rx_channel = NULL;
//         return err;
//     }

//     ESP_LOGI(TAG, "RMT receiver initialized on GPIO%d",
//              (int)gpio);

//     return ESP_OK;
// }

// esp_err_t dht22_read(dht22_data_t *data)
// {
//     if (data == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     if (s_rx_channel == NULL) {
//         return ESP_ERR_INVALID_STATE;
//     }

//     // Actual DHT22 transaction and decoding come next.
//     return ESP_ERR_NOT_SUPPORTED;
// }



#include "dht22.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define DHT22_RMT_SYMBOLS 64
#define DHT22_TIMEOUT_MS 100

static const char *TAG = "DHT22";

static gpio_num_t s_gpio = (gpio_num_t)-1;
static rmt_channel_handle_t s_rx = NULL;
static SemaphoreHandle_t s_done = NULL;
static volatile size_t s_symbol_count = 0;

static rmt_symbol_word_t s_symbols[DHT22_RMT_SYMBOLS];

typedef struct {
    uint16_t duration_us;
    uint8_t level;
} pulse_t;

// ISR callback: notify only, do not decode data here.
static bool IRAM_ATTR on_rx_done(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *event,
    void *user_data)
{
    (void)channel;
    (void)user_data;

    BaseType_t woken = pdFALSE;

    s_symbol_count = event->num_symbols;
    xSemaphoreGiveFromISR(s_done, &woken);

    return woken == pdTRUE;
}

esp_err_t dht22_init(gpio_num_t gpio)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_rx != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;

    s_done = xSemaphoreCreateBinary();
    if (s_done == NULL) return ESP_ERR_NO_MEM;

    rmt_rx_channel_config_t rx_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = DHT22_RMT_SYMBOLS,
        .flags.io_loop_back = true
    };

    err = rmt_new_rx_channel(&rx_config, &s_rx);
    if (err != ESP_OK) goto fail;

    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = on_rx_done
    };

    err = rmt_rx_register_event_callbacks(
        s_rx, &callbacks, NULL
    );
    if (err != ESP_OK) goto fail;

    err = rmt_enable(s_rx);
    if (err != ESP_OK) goto fail;


    // Release the DHT22 data line.
    gpio_set_level(gpio, 1);
    s_gpio = gpio;


    ESP_LOGI(TAG, "DHT22 driver ready on GPIO%d",
             (int)gpio);
    return ESP_OK;

fail:
    if (s_rx != NULL) {
        rmt_del_channel(s_rx);
        s_rx = NULL;
    }
    vSemaphoreDelete(s_done);
    s_done = NULL;
    return err;
}

static bool in_range(unsigned value,
                     unsigned minimum,
                     unsigned maximum)
{
    return value >= minimum && value <= maximum;
}

static esp_err_t decode_data(dht22_data_t *data)
{
    pulse_t pulses[DHT22_RMT_SYMBOLS * 2];
    size_t count = 0;

    // Convert RMT symbols into individual HIGH/LOW pulses.
    for (size_t i = 0; i < s_symbol_count &&
                       i < DHT22_RMT_SYMBOLS; ++i) {
        if (s_symbols[i].duration0) {
            pulses[count++] = (pulse_t){
                s_symbols[i].duration0,
                s_symbols[i].level0
            };
        }

        if (s_symbols[i].duration1) {
            pulses[count++] = (pulse_t){
                s_symbols[i].duration1,
                s_symbols[i].level1
            };
        }
    }

    // Find sensor response: approximately 80 us LOW,
    // followed by 80 us HIGH.
    size_t start = count;

    for (size_t i = 0; i + 1 < count; ++i) {
        if (pulses[i].level == 0 &&
            pulses[i + 1].level == 1 &&
            in_range(pulses[i].duration_us, 60, 110) &&
            in_range(pulses[i + 1].duration_us, 60, 110)) {
            start = i + 2;
            break;
        }
    }

    if (start == count || count - start < 80) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t bytes[5] = {0};

    // Each bit: around 50 us LOW, then HIGH.
    // Short HIGH = 0, long HIGH = 1.
    for (int bit = 0; bit < 40; ++bit) {
        pulse_t low = pulses[start + bit * 2];
        pulse_t high = pulses[start + bit * 2 + 1];

        if (low.level != 0 || high.level != 1 ||
            !in_range(low.duration_us, 30, 80) ||
            !in_range(high.duration_us, 15, 100)) {
            return ESP_ERR_INVALID_RESPONSE;
        }

        bytes[bit / 8] <<= 1;

        if (high.duration_us > 48) {
            bytes[bit / 8] |= 1;
        }
    }

    uint8_t checksum = (uint8_t)(
        bytes[0] + bytes[1] + bytes[2] + bytes[3]
    );

    if (checksum != bytes[4]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t humidity_raw =
        ((uint16_t)bytes[0] << 8) | bytes[1];

    uint16_t temperature_raw =
        ((uint16_t)bytes[2] << 8) | bytes[3];

    float temperature =
        (temperature_raw & 0x7FFF) / 10.0f;

    if (temperature_raw & 0x8000) {
        temperature = -temperature;
    }

    data->temperature_c = temperature;
    data->humidity_pct = humidity_raw / 10.0f;

    return ESP_OK;
}

esp_err_t dht22_read(dht22_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    if (s_rx == NULL) return ESP_ERR_INVALID_STATE;

    // Clear the previous completion notification.
    xSemaphoreTake(s_done, 0);
    s_symbol_count = 0;

    // DHT22 start: hold LOW for approximately 1.2 ms.
    esp_err_t err = gpio_set_level(s_gpio, 0);
    if (err != ESP_OK) return err;

    esp_rom_delay_us(1200);

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 200000
    };

    // Arm RMT before releasing the data line.
    err = rmt_receive(
        s_rx,
        s_symbols,
        sizeof(s_symbols),
        &receive_config
    );

    gpio_set_level(s_gpio, 1);

    if (err != ESP_OK) return err;

    if (xSemaphoreTake(
            s_done,
            pdMS_TO_TICKS(DHT22_TIMEOUT_MS)
        ) != pdTRUE) {
        rmt_disable(s_rx);
        rmt_enable(s_rx);
        return ESP_ERR_TIMEOUT;
    }

    err = decode_data(data);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Read failed: %s, symbols=%u",
                 esp_err_to_name(err),
                 (unsigned)s_symbol_count);
    }

    return err;
}
