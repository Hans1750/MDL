#include "init.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nfc7i2c.h"
#include "driver/i2c.h"
#include "freertos/task.h"
#include "string.h"
#include "driver/rmt.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "cryptoauthlib.h"



static const char *TAG = "init";

//barcode reader
#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TX_PIN 6
#define RX_PIN 7

//led ring
#define LED_PIN        5
#define LED_COUNT      24
#define BRIGHTNESS     5
#define RMT_CHANNEL    RMT_CHANNEL_0
#define RMT_CLK_DIV    2
// Duration of logical '0' high signal (0.35 µs at 25 ns resolution)
#define T0H 14
// Duration of logical '0' low signal (0.85 µs at 25 ns resolution)
#define T0L 34
// Duration of logical '1' high signal (0.7 µs at 25 ns resolution)
#define T1H 28
// Duration of logical '1' low signal (0.6 µs at 25 ns resolution)
#define T1L 20
// Reset time between data frames (80 µs > 50 µs minimum required)
#define RESET_US 80

// Initializes the GM65 barcode scanner over UART
static void init_barcode_scanner(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600, // default baudrate for GM65
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);

    ESP_LOGI("BARCODE", "Barcode scanner initialized");
}

static const char *TAG_NFC = "nfc7i2c_main";
static nfc7i2c_t nfc7i2c;

// Initializes the NFC 7 Click reader over I2C
static void init_nfc_reader(void) {
    nfc7i2c_cfg_t nfc7i2c_cfg;
    ESP_LOGI(TAG_NFC, "Application Init");

    nfc7i2c_cfg_setup(&nfc7i2c_cfg);
    nfc7i2c_cfg.scl = GPIO_NUM_2;
    nfc7i2c_cfg.sda = GPIO_NUM_1;
    nfc7i2c_cfg.ven = GPIO_NUM_7;
    nfc7i2c_cfg.irq = GPIO_NUM_NC;
    nfc7i2c_cfg.i2c_speed = 100000;
    nfc7i2c_cfg.i2c_address = 0x7C;

    if (nfc7i2c_init(&nfc7i2c, &nfc7i2c_cfg) == NFC7I2C_ERROR) {
        ESP_LOGE(TAG_NFC, "Communication init failed.");
        while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (nfc7i2c_default_cfg(&nfc7i2c) == NFC7I2C_ERROR) {
        ESP_LOGE(TAG_NFC, "Default configuration failed.");
        while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG_NFC, "FW version: %02X.%02X.%02X", 
             nfc7i2c.fw_version[0], 
             nfc7i2c.fw_version[1], 
             nfc7i2c.fw_version[2]);

    ESP_LOGI(TAG_NFC, "Application Task Ready");
}

// Initializes the LED ring (WS2812) using the RMT peripheral
static void init_led_ring(void) {
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL,
        .gpio_num = LED_PIN,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 1,
        .tx_config = {
            .loop_en = false,
            .carrier_en = false,
            .idle_output_en = true,
            .idle_level = RMT_IDLE_LEVEL_LOW,
        }
    };
    rmt_config(&config);
    rmt_driver_install(RMT_CHANNEL, 0, 0);

    ESP_LOGI("LED_RING", "LED ring initialized");
}

// Initializes Wi-Fi in Station mode and sets channel
static void init_wifi(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("WIFI", "Wi-Fi initialized in STA mode");
}

// Initializes ESP-NOW communication protocol
static void init_espnow(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_LOGI("ESP-NOW", "ESP-NOW initialized");
}

// Initializes Secure 8 Click (ATECC608A) and configures multiple data and key slots
void secure8_init_and_configure(void) {
    ESP_LOGI(TAG, "Initializing Secure 8 Click with cryptoauthlib...");

    ATCAIfaceCfg cfg = {
        .iface_type = ATCA_I2C_IFACE,
        .devtype = ATECC608A,
        .atcai2c.address = 0x60,
        .atcai2c.bus = 0,
        .atcai2c.baud = 100000,
        .wake_delay = 1500,
        .rx_retries = 20,
        .cfg_data = NULL
    };

    if (atcab_init(&cfg) != ATCA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize ATECC608A");
        return;
    }

    // Example: storing server data in dedicated slots
    const char *server_address = "server.example.com";
    const char *server_port = "443";
    const char *auth_token = "my_secure_token";

    atcab_write_zone(ATCA_ZONE_DATA, 8, 0, 0, (const uint8_t *)server_address, strlen(server_address));  // Slot 8: server address
    atcab_write_zone(ATCA_ZONE_DATA, 9, 0, 0, (const uint8_t *)server_port, strlen(server_port));        // Slot 9: port
    atcab_write_zone(ATCA_ZONE_DATA, 10, 0, 0, (const uint8_t *)auth_token, strlen(auth_token));        // Slot 10: token

    // Initialize keys in slots 0–3
    // Slot 0: ECDSA signing key (private key)
    if (atcab_genkey(0, NULL) == ATCA_SUCCESS) {
        ESP_LOGI(TAG, "ECDSA key generated in slot 0");
    }

    // Slot 1: verification key (public component)
    // Retrieve public key from slot 0 for use in verification
    uint8_t pubkey[64];
    if (atcab_get_pubkey(0, pubkey) == ATCA_SUCCESS) {
        ESP_LOGI(TAG, "Public key retrieved from slot 0");
    }

    // Slot 2: symmetric key for HMAC
    uint8_t hmac_key[32] = {0xA1, 0xB2, 0xC3}; // example key bytes
    if (atcab_write_zone(ATCA_ZONE_DATA, 2, 0, 0, hmac_key, sizeof(hmac_key)) == ATCA_SUCCESS) {
        ESP_LOGI(TAG, "Symmetric key written to slot 2");
    }

    // Slot 3: reserved for AES encryption
    uint8_t aes_key[32] = {0x11, 0x22, 0x33};
    if (atcab_write_zone(ATCA_ZONE_DATA, 3, 0, 0, aes_key, sizeof(aes_key)) == ATCA_SUCCESS) {
        ESP_LOGI(TAG, "AES key written to slot 3");
    }

    ESP_LOGI(TAG, "Secure 8 configuration complete: data stored in slots 8–10");
}


// Initializes all peripherals and components
void init_all(void) {
    ESP_LOGI("INIT", "Začenjam inicializacijo naprave...");

    init_barcode_scanner();
    init_nfc_reader();
    init_led_ring();
    init_wifi();
    init_espnow();
    secure8_init_and_configure();

    ESP_LOGI("INIT", "Inicializacija zaključena.");
}