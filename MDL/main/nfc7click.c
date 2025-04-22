#include "nfc7i2c.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "string.h"
#include "cbor.h"
#include "esp_now.h"

static const char *TAG = "nfc7i2c_main";

static nfc7i2c_t nfc7i2c;

static void nfc7i2c_handle_iso14443_3a(nfc7i2c_t *ctx);
static void nfc7i2c_handle_iso14443_4(nfc7i2c_t *ctx);
static void nfc7i2c_display_card_info(nfc7i2c_rf_intf_t rf_intf);
static void decode_cbor_data(const uint8_t *data, size_t data_len);

void nfc7i2c_process_card(void) {
    nfc7i2c_rf_intf_t rf_intf;
    ESP_LOGI(TAG, "WAITING FOR DEVICE DISCOVERY\n");

    if (NFC7I2C_OK == nfc7i2c_wait_discovery(&nfc7i2c, &rf_intf)) {
        if ((NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_A == rf_intf.mode_tech) || 
            (NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_B == rf_intf.mode_tech) || 
            (NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_F == rf_intf.mode_tech) || 
            (NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_15693 == rf_intf.mode_tech)) {

            while (true) {
                nfc7i2c_display_card_info(rf_intf);

                switch (rf_intf.protocol) {
                    case NFC7I2C_NCI_RF_PROT_T2T:
                        nfc7i2c_handle_iso14443_3a(&nfc7i2c);
                        break;
                    case NFC7I2C_NCI_RF_PROT_ISODEP:
                        nfc7i2c_handle_iso14443_4(&nfc7i2c);
                        break;
                    default:
                        break;
                }

                if (!rf_intf.more_tags) {
                    break;
                }
                nfc7i2c_reader_act_next(&nfc7i2c, &rf_intf);
            }

            nfc7i2c_presence_check(&nfc7i2c, &rf_intf);
            ESP_LOGI(TAG, "- CARD REMOVED\n");

            nfc7i2c_stop_discovery(&nfc7i2c);
            while (NFC7I2C_OK != nfc7i2c_start_discovery(&nfc7i2c)) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }

        } else {
            ESP_LOGW(TAG, "- WRONG DISCOVERY\n");
        }
    }
}

static void decode_cbor_data(const uint8_t *data, size_t data_len) {
    CborParser parser;
    CborValue it;
    CborValue map_it;

    if (cbor_parser_init(data, data_len, 0, &parser, &it) != CborNoError || !cbor_value_is_map(&it)) {
        ESP_LOGE(TAG, "CBOR: Failed to parse or not a map");
        return;
    }

    char json_msg[256] = "{";
    size_t json_len = strlen(json_msg);

    cbor_value_enter_container(&it, &map_it);

    while (!cbor_value_at_end(&map_it)) {
        char key[64] = {0};
        char value[128] = {0};
        size_t len;

        if (cbor_value_is_text_string(&map_it)) {
            len = sizeof(key) - 1;
            cbor_value_copy_text_string(&map_it, key, &len, &map_it);
        }

        if (cbor_value_is_text_string(&map_it)) {
            len = sizeof(value) - 1;
            cbor_value_copy_text_string(&map_it, value, &len, &map_it);
        }

        ESP_LOGI(TAG, "CBOR -> %s: %s", key, value);

        // Add to JSON-like string
        int written = snprintf(json_msg + json_len, sizeof(json_msg) - json_len, "\"%s\":\"%s\",", key, value);
        if (written > 0 && (json_len + written < sizeof(json_msg))) {
            json_len += written;
        }
    }

    cbor_value_leave_container(&it, &map_it);

    // Remove trailing comma if needed and close JSON object
    if (json_len > 1 && json_msg[json_len - 1] == ',') {
        json_msg[json_len - 1] = '}';
        json_msg[json_len] = 'NULL';
    } else {
        strncat(json_msg, "}", sizeof(json_msg) - strlen(json_msg) - 1);
    }

    // Send full structured message via ESP-NOW
    uint8_t mac_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // change to receiver mac
    esp_now_send(mac_address, (uint8_t *)json_msg, strlen(json_msg));
    ESP_LOGI(TAG, "ESP-NOW sent: %s", json_msg);


    cbor_value_enter_container(&it, &map_it);

    while (!cbor_value_at_end(&map_it)) {
        char key[64] = {0};
        char value[128] = {0};
        size_t len;

        if (cbor_value_is_text_string(&map_it)) {
            len = sizeof(key) - 1;
            cbor_value_copy_text_string(&map_it, key, &len, &map_it);
        }

        if (cbor_value_is_text_string(&map_it)) {
            len = sizeof(value) - 1;
            cbor_value_copy_text_string(&map_it, value, &len, &map_it);
        }

        ESP_LOGI(TAG, "CBOR -> %s: %s", key, value);
    }

    cbor_value_leave_container(&it, &map_it);
}

  

    static void nfc7i2c_handle_iso14443_4(nfc7i2c_t *ctx) {
    err_t error_flag = NFC7I2C_OK;

    ctx->pkt_data.payload_len = strlen(NFC7I2C_T4T_PPSE_APDU) + 6;
    ctx->pkt_data.payload[0] = NFC7I2C_T4T_CLA_NO_SECURE;
    ctx->pkt_data.payload[1] = NFC7I2C_T4T_INS_SELECT;
    ctx->pkt_data.payload[2] = NFC7I2C_T4T_P1_SELECT_BY_NAME;
    ctx->pkt_data.payload[3] = NFC7I2C_T4T_P2_ONLY_OCCURANCE;
    ctx->pkt_data.payload[4] = strlen(NFC7I2C_T4T_PPSE_APDU);
    memcpy(&ctx->pkt_data.payload[5], NFC7I2C_T4T_PPSE_APDU, strlen(NFC7I2C_T4T_PPSE_APDU));
    ctx->pkt_data.payload[ctx->pkt_data.payload_len - 1] = NFC7I2C_T4T_LE_RSP_MAY_PRESENT;

    error_flag = nfc7i2c_reader_tag_cmd(ctx, &ctx->pkt_data);
    if ((NFC7I2C_OK != error_flag) || 
        (NFC7I2C_T4T_RSP_COMPLETE_1 != ctx->pkt_data.payload[ctx->pkt_data.payload_len - 2]) || 
        (NFC7I2C_T4T_RSP_COMPLETE_2 != ctx->pkt_data.payload[ctx->pkt_data.payload_len - 1])) {
        ESP_LOGE(TAG, "Select PPSE failed with error %.2X %.2X", 
                 ctx->pkt_data.payload[ctx->pkt_data.payload_len - 2], 
                 ctx->pkt_data.payload[ctx->pkt_data.payload_len - 1]);
        return;
    }
    ESP_LOGI(TAG, "Select PPSE Application succeed");

    decode_cbor_data(ctx->pkt_data.payload, ctx->pkt_data.payload_len);
}

static void nfc7i2c_handle_iso14443_3a(nfc7i2c_t *ctx) {
    #define BLK_NB_ISO14443_3A      32
    #define DATA_WRITE_ISO14443_3A  0x11, 0x22, 0x33, 0x44
    uint8_t rd_block[] = { NFC7I2C_T2T_CMD_READ, BLK_NB_ISO14443_3A };
    uint8_t wr_block[] = { NFC7I2C_T2T_CMD_WRITE, BLK_NB_ISO14443_3A, DATA_WRITE_ISO14443_3A };
    err_t error_flag = NFC7I2C_OK;

    ctx->pkt_data.payload_len = sizeof(rd_block);
    memcpy(ctx->pkt_data.payload, rd_block, ctx->pkt_data.payload_len);
    error_flag = nfc7i2c_reader_tag_cmd(ctx, &ctx->pkt_data);
    if ((NFC7I2C_OK != error_flag) || 
        (NFC7I2C_NCI_STAT_OK != ctx->pkt_data.payload[ctx->pkt_data.payload_len - 1])) {
        ESP_LOGE(TAG, "Read block %u failed with error %.2X", rd_block[1], ctx->pkt_data.payload[ctx->pkt_data.payload_len - 1]);
        return;
    }
    ESP_LOGI(TAG, "Read block %u: %.2X %.2X %.2X %.2X", rd_block[1],
             ctx->pkt_data.payload[0], ctx->pkt_data.payload[1],
             ctx->pkt_data.payload[2], ctx->pkt_data.payload[3]);

    decode_cbor_data(ctx->pkt_data.payload, ctx->pkt_data.payload_len);

    ctx->pkt_data.payload_len = sizeof(wr_block);
    memcpy(ctx->pkt_data.payload, wr_block, ctx->pkt_data.payload_len);
    error_flag = nfc7i2c_reader_tag_cmd(ctx, &ctx->pkt_data);
    if ((NFC7I2C_OK != error_flag) || (NFC7I2C_T2T_ACK != ctx->pkt_data.payload[0])) {
        ESP_LOGE(TAG, "Write block %u failed with error %.2X", wr_block[1], ctx->pkt_data.payload[ctx->pkt_data.payload_len - 1]);
        return;
    }
    ESP_LOGI(TAG, "Block %u written", wr_block[1]);

    ctx->pkt_data.payload_len = sizeof(rd_block);
    memcpy(ctx->pkt_data.payload, rd_block, ctx->pkt_data.payload_len);
    error_flag = nfc7i2c_reader_tag_cmd(ctx, &ctx->pkt_data);
    if ((NFC7I2C_OK != error_flag) || 
        (NFC7I2C_NCI_STAT_OK != ctx->pkt_data.payload[ctx->pkt_data.payload_len - 1])) {
        ESP_LOGE(TAG, "Read block %u failed with error %.2X", rd_block[1], ctx->pkt_data.payload[ctx->pkt_data.payload_len - 1]);
        return;
    }
    ESP_LOGI(TAG, "Read block %u: %.2X %.2X %.2X %.2X", rd_block[1],
             ctx->pkt_data.payload[0], ctx->pkt_data.payload[1],
             ctx->pkt_data.payload[2], ctx->pkt_data.payload[3]);

    decode_cbor_data(ctx->pkt_data.payload, ctx->pkt_data.payload_len);
}
    
        static void nfc7i2c_display_card_info(nfc7i2c_rf_intf_t rf_intf) {
    switch (rf_intf.protocol) {
        case NFC7I2C_NCI_RF_PROT_T1T:
        case NFC7I2C_NCI_RF_PROT_T2T:
        case NFC7I2C_NCI_RF_PROT_T3T:
        case NFC7I2C_NCI_RF_PROT_ISODEP:
            ESP_LOGI(TAG, " - POLL MODE: Remote T%uT activated", rf_intf.protocol);
            break;
        case NFC7I2C_NCI_RF_PROT_T5T:
            ESP_LOGI(TAG, " - MODE: Remote ISO15693 card activated");
            break;
        case NFC7I2C_NCI_RF_PROT_MIFARE:
            ESP_LOGI(TAG, " - POLL MODE: Remote MIFARE card activated");
            break;
        default:
            ESP_LOGW(TAG, " - POLL MODE: Undetermined target");
            return;
    }

    switch (rf_intf.mode_tech) {
        case NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_A:
            ESP_LOGI(TAG, "\tSENS_RES = %.2X %.2X", rf_intf.info.nfc_app.sens_res[0], rf_intf.info.nfc_app.sens_res[1]);
            ESP_LOGI(TAG, "\tNFCID =");
            for (uint8_t i = 0; i < rf_intf.info.nfc_app.nfc_id_len; i++) {
                ESP_LOGI(TAG, "%.2X", rf_intf.info.nfc_app.nfc_id[i]);
            }
            if (0 != rf_intf.info.nfc_app.sel_res_len) {
                ESP_LOGI(TAG, "\tSEL_RES = %.2X", rf_intf.info.nfc_app.sens_res[0]);
            }
            break;
        case NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_B:
            if (0 != rf_intf.info.nfc_bpp.sens_res_len) {
                ESP_LOGI(TAG, "\tSENS_RES =");
                for (uint8_t i = 0; i < rf_intf.info.nfc_bpp.sens_res_len; i++) {
                    ESP_LOGI(TAG, "%.2X", rf_intf.info.nfc_bpp.sens_res[i]);
                }
            }
            break;
        case NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_F:
            ESP_LOGI(TAG, "\tBitrate = %s", (1 == rf_intf.info.nfc_fpp.bitrate) ? "212" : "424");
            if (0 != rf_intf.info.nfc_fpp.sens_res_len) {
                ESP_LOGI(TAG, "\tSENS_RES =");
                for (uint8_t i = 0; i < rf_intf.info.nfc_fpp.sens_res_len; i++) {
                    ESP_LOGI(TAG, "%.2X", rf_intf.info.nfc_fpp.sens_res[i]);
                }
            }
            break;
        case NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_15693:
            ESP_LOGI(TAG, "\tID =");
            for (uint8_t i = 0; i < sizeof(rf_intf.info.nfc_vpp.id); i++) {
                ESP_LOGI(TAG, "%.2X", rf_intf.info.nfc_vpp.id[i]);
            }
            ESP_LOGI(TAG, "\tAFI = %.2X", rf_intf.info.nfc_vpp.afi);
            ESP_LOGI(TAG, "\tDSFID = %.2X", rf_intf.info.nfc_vpp.dsf_id);
            break;
        default:
            break;
    }
}
