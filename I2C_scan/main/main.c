#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 2        // GPIO za SCL
#define I2C_MASTER_SDA_IO 1        // GPIO za SDA
#define I2C_MASTER_FREQ_HZ 100000   // 100kHz
#define I2C_MASTER_PORT I2C_NUM_0   // I2C port 0
#define TAG "I2C_SCAN"

void i2c_scan() {
    esp_err_t ret;

    // Inicializacija I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return;
    }
    int i;
    esp_err_t espRc;
    printf("Scanning I2C bus...\n");
    
    for (i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        espRc = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (espRc == ESP_OK) {
            printf("Found device at address 0x%02X\n", i);
        }
    }
    printf("I2C scan complete.\n");
}

void app_main() {


    i2c_scan(); // Zagon I2C skeniranja
}
