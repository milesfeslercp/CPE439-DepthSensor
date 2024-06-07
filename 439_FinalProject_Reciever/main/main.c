// MAC OF SENDER D4:8A:FC:A0:F1:F4
// RECIEVER MAC 08:D1:F9:EC:74:08 

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "Receiver";
static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0x08, 0xD1, 0xF9, 0xEC, 0x74, 0x08}; 

typedef struct {
    float depth;
} sensor_data_t;

static SemaphoreHandle_t recv_semaphore;

static void on_data_recv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    printf("Data received\n");
    if (data_len == sizeof(sensor_data_t)) {
        sensor_data_t received_data;
        memcpy(&received_data, data, sizeof(sensor_data_t));
        ESP_LOGI(TAG, "Received Depth: %.2f mm", received_data.depth);
    } else {
        ESP_LOGE(TAG, "Received data length mismatch: %d", data_len);
    }
    xSemaphoreGive(recv_semaphore);
}

void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
}

void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
    recv_semaphore = xSemaphoreCreateBinary();

    wifi_init();
    espnow_init();

    while (1) {
        if (xSemaphoreTake(recv_semaphore, portMAX_DELAY)) {
            // Data received and logged in on_data_recv callback
        }
    }
}
