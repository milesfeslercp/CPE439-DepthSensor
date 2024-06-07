#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_timer.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_pm.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)

#define EXAMPLE_READ_LEN                    256
#define RANGE                               5000 // Depth measuring range 5000mm (for water)
#define VREF                                5000 // ADC's reference voltage, typical value: 5000mV
#define CURRENT_INIT                        2.7 // Current @ 0mm (unit: mA)
#define DENSITY_WATER                       1    // Pure water density normalized to 1
#define PRINT_INTERVAL                      1000
#define AVERAGE_SAMPLES                     50   // Number of samples for averaging
#define Depth_Offset                        3380    // Offset for depth calculation in mm

static adc_channel_t channel[1] = {ADC_CHANNEL_4};
static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0x08, 0xD1, 0xF9, 0xEC, 0x74, 0x08}; // Replace with the peer's MAC address

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

float calculate_moving_average(float new_value, float *history, int history_size, int *current_index, float *total)
{
    *total -= history[*current_index];
    history[*current_index] = new_value;
    *total += new_value;
    *current_index = (*current_index + 1) % history_size;
    return *total / history_size;
}

static void IRAM_ATTR espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // Empty callback function for ESP-NOW send status
}

void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
}

void app_main(void)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    // Initialize WiFi (required for ESPNOW)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Initialize ESPNOW
    espnow_init();

    // Configure light sleep
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
        .light_sleep_enable = true
    };
    if (esp_pm_configure(&pm_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure power management");
    }

    unsigned long timepoint_measure = 0;
    int16_t dataVoltage;
    float dataCurrent, depth;
    float history[AVERAGE_SAMPLES] = {0};
    int current_index = 0;
    float total = 0;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int sample_count = 0;
        float depth_sum = 0.0;

        while (sample_count < 50) {
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);

                    if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                        dataVoltage = data / 4096.0 * VREF; // Adjust if ADC resolution is different
                        dataCurrent = dataVoltage / 120.0;  // Sense xxResistor: 120 ohms
                        float current_depth = (dataCurrent - CURRENT_INIT) * (RANGE / DENSITY_WATER / 16.0) - Depth_Offset; // Calculate depth from current readings

                        if (current_depth < 0) {
                            current_depth = 0.0;
                        }

                        depth_sum += current_depth;
                        sample_count++;

                        if (sample_count >= 50) {
                            break;
                        }
                    } else {
                        ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT), chan_num, data);
                    }
                }
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }

        if (sample_count > 0) {
            depth = depth_sum / sample_count;

            ESP_LOGI(TAG, "Average Depth: %.2f mm", depth);

            // Transmit depth via ESPNOW
            esp_err_t send_err = esp_now_send(peer_mac, (uint8_t*)&depth, sizeof(depth));
            if (send_err == ESP_OK) {
                ESP_LOGI(TAG, "Depth sent successfully");
            } else {
                ESP_LOGE(TAG, "Error sending depth: %s", esp_err_to_name(send_err));
            }
        }

        // Enter light sleep for 5 seconds
        ESP_LOGI(TAG, "Entering light sleep for 1 seconds");
        esp_sleep_enable_timer_wakeup(1000000); // 1 seconds
        esp_light_sleep_start();

    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}