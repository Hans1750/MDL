#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

void init_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    ESP_ERROR_CHECK(esp_wifi_start());
}

void init_espnow(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_LOGI("ESP-NOW", "ESP-NOW inicializiran");
}

void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    char mac_str[18];
    const uint8_t *mac_addr = recv_info->src_addr;

    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    ESP_LOGI("ESP-NOW", "Sporočilo prejeto od %s: %.*s", mac_str, len, data);
}
void register_receive_cb()
{
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
}

void app_main(void)
{
    nvs_flash_init(); // pomembno za ESP-NOW
    init_wifi();
    init_espnow();
    register_receive_cb();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}