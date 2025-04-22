#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"


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

static const uint8_t peer_mac[6] = {0x34, 0x85, 0x18, 0x8d, 0x65, 0x88}; // zamenjaj z MAC naslovom sprejemnika

void send_data()
{
    const char *msg = "Pozdrav s posiljatelja!";
  
    esp_err_t result = esp_now_send(peer_mac, (uint8_t *)msg, strlen(msg));
    if (result == ESP_OK)
    {
        ESP_LOGI("ESP-NOW", "Podatki poslani!");
    }
    else
    {
        ESP_LOGE("ESP-NOW", "Napaka pri pošiljanju: %d", result);
    }
}

void app_main()
{
    nvs_flash_init(); // pomembno za ESP-NOW
    init_wifi();
    init_espnow();
    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, peer_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_LOGI("ESP-NOW", "Pošiljam podatke na: %02X:%02X:%02X:%02X:%02X:%02X",
             peer_mac[0], peer_mac[1], peer_mac[2],
             peer_mac[3], peer_mac[4], peer_mac[5]);

    uint8_t primary;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    ESP_LOGI("ESP-NOW", "Trenutni kanal: %d", primary);

    while (1)
    {
        send_data();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}