#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "init.h"
#include "nfc7click.h"
#include "ledring.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_now.h"

#define UART_BUF_SIZE 512
static const char *TAG = "main";

void nfc_task(void *pvParameters) {
    while (1) {
        nfc7i2c_process_card();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void qr_task(void *pvParameters) {
    uint8_t data[UART_BUF_SIZE];
    uint8_t mac_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //change to the mac adress of the receiver

    while (1) {
        int len = uart_read_bytes(UART_NUM_1, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "QR Code: %s", data);
            esp_now_send(mac_address, data, len);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    init_all();

    xTaskCreate(nfc_task, "nfc_task", 4096, NULL, 5, NULL);
    xTaskCreate(qr_task, "qr_task", 4096, NULL, 5, NULL);
}
