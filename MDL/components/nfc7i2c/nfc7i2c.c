#include "nfc7i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "string.h"
#include "esp_check.h"

#define TAG "NFC7_DRV"
#define NFC7_PIN_VEN GPIO_NUM_4
#define NFC7_PIN_IRQ GPIO_NUM_3

void nfc7i2c_cfg_setup(nfc7i2c_cfg_t *cfg)
{
    cfg->scl = GPIO_NUM_6;
    cfg->sda = GPIO_NUM_5;
    cfg->irq = GPIO_NUM_3;
    cfg->ven = GPIO_NUM_4;
    cfg->i2c_speed = 400000;
    cfg->i2c_address = 0x28;
    cfg->i2c_port = I2C_NUM_0;
}

void nfc7i2c_fill_intf_info(nfc7i2c_rf_intf_t *rf_intf, uint8_t *data_in)
{
    switch (rf_intf->mode_tech)
    {
    case NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_A:
        memcpy(rf_intf->info.nfc_app.sens_res, &data_in[0], 2);
        rf_intf->info.nfc_app.nfc_id_len = data_in[2];
        memcpy(rf_intf->info.nfc_app.nfc_id, &data_in[3], rf_intf->info.nfc_app.nfc_id_len);
        rf_intf->info.nfc_app.sel_res_len = data_in[rf_intf->info.nfc_app.nfc_id_len + 3];
        if (1 == rf_intf->info.nfc_app.sel_res_len)
        {
            rf_intf->info.nfc_app.sel_res[0] = data_in[rf_intf->info.nfc_app.nfc_id_len + 4];
        }
        if (0 != data_in[rf_intf->info.nfc_app.nfc_id_len + 8])
        {
            rf_intf->info.nfc_app.rats_len = data_in[rf_intf->info.nfc_app.nfc_id_len + 9];
            memcpy(rf_intf->info.nfc_app.rats,
                   &data_in[rf_intf->info.nfc_app.nfc_id_len + 10],
                   data_in[rf_intf->info.nfc_app.nfc_id_len + 9]);
        }
        else
        {
            rf_intf->info.nfc_app.rats_len = 0;
        }
        break;
    case NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_B:
        rf_intf->info.nfc_bpp.sens_res_len = data_in[0];
        memcpy(rf_intf->info.nfc_bpp.sens_res, &data_in[1], rf_intf->info.nfc_bpp.sens_res_len);
        if (0 != data_in[rf_intf->info.nfc_bpp.sens_res_len + 4])
        {
            rf_intf->info.nfc_bpp.attrib_res_len = data_in[rf_intf->info.nfc_bpp.sens_res_len + 5];
            memcpy(rf_intf->info.nfc_bpp.attrib_res,
                   &data_in[rf_intf->info.nfc_bpp.sens_res_len + 6],
                   data_in[rf_intf->info.nfc_bpp.sens_res_len + 5]);
        }
        else
        {
            rf_intf->info.nfc_bpp.attrib_res_len = 0;
        }
        break;
    case NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_F:
        rf_intf->info.nfc_fpp.bitrate = data_in[0];
        rf_intf->info.nfc_fpp.sens_res_len = data_in[1];
        memcpy(rf_intf->info.nfc_fpp.sens_res, &data_in[2], rf_intf->info.nfc_fpp.sens_res_len);
        break;
    case NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_15693:
        rf_intf->info.nfc_vpp.afi = data_in[0];
        rf_intf->info.nfc_vpp.dsf_id = data_in[1];
        for (uint8_t cnt = 0; cnt < 8; cnt++)
        {
            rf_intf->info.nfc_vpp.id[cnt] = data_in[2 + cnt];
        }
        break;
    }
}

err_t nfc7i2c_init(nfc7i2c_t *ctx, nfc7i2c_cfg_t *cfg)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg->sda,
        .scl_io_num = cfg->scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->i2c_speed,
    };

    if (cfg->sda == GPIO_NUM_NC || cfg->scl == GPIO_NUM_NC)
    {
        return ESP_FAIL;
    }

    ctx->slave_address = cfg->i2c_address;

    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK)
        return ret;

    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK)
        return ret;

    if (cfg->ven != GPIO_NUM_NC)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << cfg->ven,
            .mode = GPIO_MODE_INPUT_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};
        gpio_config(&io_conf);
        gpio_set_level(cfg->ven, 1);
        ctx->ven = cfg->ven;
    }

    if (cfg->irq != GPIO_NUM_NC)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << cfg->irq,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};
        gpio_config(&io_conf);
        ctx->irq = cfg->irq;
    }

    return ESP_OK;
}

err_t nfc7i2c_default_cfg(nfc7i2c_t *ctx)
{
    err_t error_flag = NFC7I2C_OK;
    nfc7i2c_reset_device(ctx);

    if (NFC7I2C_OK != nfc7i2c_core_init(ctx))
    {
        ESP_LOGE("lib", "core init failed");
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_config_settings(ctx))
    {
        ESP_LOGE("lib", "config failed");
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_map_rf_interface(ctx))
    {
        ESP_LOGE("lib", "map_rf failed");
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_start_discovery(ctx))
    {
        ESP_LOGE("lib", "discovery failed");
        return NFC7I2C_ERROR;
    }

    return error_flag;
}

void nfc7i2c_enable_device(nfc7i2c_t *ctx)
{
    gpio_set_level(ctx->ven, 1);
    ESP_LOGI("I2C", "Device enabled");
    ESP_LOGI("GPIO", "VEN (GPIO %d) level: %d", ctx->ven, gpio_get_level(ctx->ven));
    vTaskDelay(pdMS_TO_TICKS(10));
}

void nfc7i2c_disable_device(nfc7i2c_t *ctx)
{
    gpio_set_level(ctx->ven, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void nfc7i2c_reset_device(nfc7i2c_t *ctx)
{
    nfc7i2c_disable_device(ctx);
    nfc7i2c_enable_device(ctx);
}

uint8_t nfc7i2c_get_irq_pin(nfc7i2c_t *ctx)
{
    return gpio_get_level(ctx->irq);
}

err_t nfc7i2c_tx(nfc7i2c_t *ctx, uint8_t *data_in, uint16_t len)
{
    if (len > NFC7I2C_MAX_NCI_FRAME_SIZE)
    {
        return NFC7I2C_ERROR;
    }

    esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, ctx->slave_address, data_in, len, pdMS_TO_TICKS(100));
    if (err != ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        err = i2c_master_write_to_device(I2C_NUM_0, ctx->slave_address, data_in, len, pdMS_TO_TICKS(100));
        if (err != ESP_OK)
        {
            return NFC7I2C_ERROR;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    return NFC7I2C_OK;
}

err_t nfc7i2c_rx(nfc7i2c_t *ctx, uint8_t *data_out, uint16_t *len, uint16_t timeout)
{
    uint16_t timeout_cnt = 0;
    while (!nfc7i2c_get_irq_pin(ctx))
    {
        if ((NFC7I2C_TIMEOUT_INFINITE != timeout) && (timeout_cnt++ >= timeout))
        {
            ESP_LOGE("RX", "irq error");
            return NFC7I2C_ERROR;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (i2c_master_read_from_device(I2C_NUM_0, ctx->slave_address, data_out, 3, pdMS_TO_TICKS(100)) != ESP_OK)
    {

        ESP_LOGI("GPIO", "VEN (GPIO %d) level: %d", ctx->ven, gpio_get_level(ctx->ven));
        ESP_LOGI("GPIO", "IRQ (GPIO %d) level: %d", ctx->irq, gpio_get_level(ctx->irq));
        ESP_LOGI("RX", "Trying to read from 0x%02X", ctx->slave_address);

        ESP_LOGE("RX", "I2C, read error");
        return NFC7I2C_ERROR;
    }

    if ((data_out[2] + 3) > NFC7I2C_MAX_NCI_FRAME_SIZE)
    {
        return NFC7I2C_ERROR;
    }

    if (data_out[2] > 0)
    {
        if (i2c_master_read_from_device(I2C_NUM_0, ctx->slave_address, &data_out[3], data_out[2], pdMS_TO_TICKS(100)) != ESP_OK)
        {
            return NFC7I2C_ERROR;
        }
    }

    *len = data_out[2] + 3;
    vTaskDelay(pdMS_TO_TICKS(1));
    return NFC7I2C_OK;
}

err_t nfc7i2c_trx(nfc7i2c_t *ctx, uint8_t *data_in, uint16_t in_len, uint8_t *data_out, uint16_t *out_len)
{
    if (in_len > NFC7I2C_MAX_NCI_FRAME_SIZE)
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_tx(ctx, data_in, in_len))
    {
        return NFC7I2C_ERROR;
    }

    return nfc7i2c_rx(ctx, data_out, out_len, NFC7I2C_TIMEOUT_1S);
}

err_t nfc7i2c_pkt_ctrl_tx(nfc7i2c_t *ctx, nfc7i2c_pkt_ctrl_t *pkt)
{
    if ((NFC7I2C_NCI_PKT_MT_CTRL_CMD != pkt->msg_type) &&
        (NFC7I2C_NCI_PKT_MT_CTRL_RSP != pkt->msg_type) &&
        (NFC7I2C_NCI_PKT_MT_CTRL_NTF != pkt->msg_type))
    {
        return NFC7I2C_ERROR;
    }

    ctx->cmd[0] = ((pkt->msg_type << NFC7I2C_NCI_PKT_MT_SHIFT) & NFC7I2C_NCI_PKT_MT_MASK) |
                  (pkt->gid & NFC7I2C_NCI_PKT_CTRL_GID_MASK);
    ctx->cmd[1] = (pkt->oid & NFC7I2C_NCI_PKT_CTRL_OID_MASK);
    ctx->cmd[2] = pkt->payload_len;
    memcpy(&ctx->cmd[3], pkt->payload, pkt->payload_len);

    return nfc7i2c_tx(ctx, ctx->cmd, pkt->payload_len + 3);
}

err_t nfc7i2c_pkt_ctrl_rx(nfc7i2c_t *ctx, nfc7i2c_pkt_ctrl_t *pkt, uint16_t timeout)
{
    if (NFC7I2C_OK != nfc7i2c_rx(ctx, ctx->rsp, &ctx->rsp_len, timeout))
    {
        ESP_LOGE("RX", "ctrl_rx failed");
        return NFC7I2C_ERROR;
    }

    pkt->msg_type = (ctx->rsp[0] & NFC7I2C_NCI_PKT_MT_MASK) >> NFC7I2C_NCI_PKT_MT_SHIFT;
    pkt->gid = ctx->rsp[0] & NFC7I2C_NCI_PKT_CTRL_GID_MASK;
    pkt->oid = ctx->rsp[1] & NFC7I2C_NCI_PKT_CTRL_OID_MASK;
    pkt->payload_len = ctx->rsp[2];
    memcpy(pkt->payload, &ctx->rsp[3], ctx->rsp[2]);

    if ((NFC7I2C_NCI_PKT_MT_CTRL_CMD != pkt->msg_type) &&
        (NFC7I2C_NCI_PKT_MT_CTRL_RSP != pkt->msg_type) &&
        (NFC7I2C_NCI_PKT_MT_CTRL_NTF != pkt->msg_type))
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_pkt_ctrl_trx(nfc7i2c_t *ctx, nfc7i2c_pkt_ctrl_t *pkt)
{
    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_tx(ctx, pkt))
    {
        ESP_LOGE("TRX", "TX failed");
        return NFC7I2C_ERROR;
    }
    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_rx(ctx, pkt, NFC7I2C_TIMEOUT_1S))
    {
        ESP_LOGE("TRX", "RX failed");
        return NFC7I2C_ERROR;
    }
    if ((NFC7I2C_NCI_PKT_MT_CTRL_RSP != pkt->msg_type) ||
        (pkt->gid != (ctx->cmd[0] & NFC7I2C_NCI_PKT_CTRL_GID_MASK)) ||
        (pkt->oid != (ctx->cmd[1] & NFC7I2C_NCI_PKT_CTRL_OID_MASK)))
    {
        return NFC7I2C_ERROR;
    }
    return NFC7I2C_OK;
}

err_t nfc7i2c_pkt_data_tx(nfc7i2c_t *ctx, nfc7i2c_pkt_data_t *pkt)
{
    ctx->cmd[0] = (pkt->cid & NFC7I2C_NCI_PKT_DATA_CID_MASK);
    ctx->cmd[1] = NFC7I2C_NCI_PKT_DATA_RFU;
    ctx->cmd[2] = pkt->payload_len;
    memcpy(&ctx->cmd[3], pkt->payload, pkt->payload_len);
    return nfc7i2c_tx(ctx, ctx->cmd, pkt->payload_len + 3);
}

err_t nfc7i2c_pkt_data_rx(nfc7i2c_t *ctx, nfc7i2c_pkt_data_t *pkt, uint16_t timeout)
{
    if (NFC7I2C_OK != nfc7i2c_rx(ctx, ctx->rsp, &ctx->rsp_len, timeout))
    {
        return NFC7I2C_ERROR;
    }

    pkt->cid = ctx->rsp[0] & NFC7I2C_NCI_PKT_DATA_CID_MASK;
    pkt->payload_len = ctx->rsp[2];
    memcpy(pkt->payload, &ctx->rsp[3], ctx->rsp[2]);

    if (NFC7I2C_NCI_PKT_MT_DATA != ((ctx->rsp[0] & NFC7I2C_NCI_PKT_MT_MASK) >> NFC7I2C_NCI_PKT_MT_SHIFT))
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_pkt_data_trx(nfc7i2c_t *ctx, nfc7i2c_pkt_data_t *pkt)
{
    if (NFC7I2C_OK != nfc7i2c_pkt_data_tx(ctx, pkt))
    {
        return NFC7I2C_ERROR;
    }
    if (NFC7I2C_OK != nfc7i2c_pkt_data_rx(ctx, pkt, NFC7I2C_TIMEOUT_1S))
    {
        return NFC7I2C_ERROR;
    }
    if (ctx->cmd[0] != ctx->rsp[0])
    {
        return NFC7I2C_ERROR;
    }
    return NFC7I2C_OK;
}

err_t nfc7i2c_core_reset(nfc7i2c_t *ctx)
{
    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_CORE;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_CORE_RESET;
    ctx->pkt_ctrl.payload_len = 1;
    ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_CORE_RESET_RESET_CFG;

    if (nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl) != NFC7I2C_OK)
    {
        ESP_LOGE("core reset", "trx failed");
        return NFC7I2C_ERROR;
    }

    if (nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS) == NFC7I2C_OK)
    {
        if ((ctx->pkt_ctrl.msg_type != NFC7I2C_NCI_PKT_MT_CTRL_NTF) ||
            (ctx->pkt_ctrl.gid != NFC7I2C_NCI_GID_CORE) ||
            (ctx->pkt_ctrl.oid != NFC7I2C_NCI_OID_CORE_RESET) ||
            (ctx->pkt_ctrl.payload[2] != NFC7I2C_NCI_CORE_RESET_NTF_NCI_VER_20))
        {
            return NFC7I2C_ERROR;
        }
        memcpy(ctx->fw_version, &ctx->pkt_ctrl.payload[6], 3);
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_core_init(nfc7i2c_t *ctx)
{
    if (NFC7I2C_OK != nfc7i2c_core_reset(ctx))
    {
        ESP_LOGE("CORE", "core reset failed");
        return NFC7I2C_ERROR;
    }

    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_CORE;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_CORE_INIT;
    ctx->pkt_ctrl.payload_len = 2;
    ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_CORE_INIT_FEATURE_DIS;
    ctx->pkt_ctrl.payload[1] = NFC7I2C_NCI_CORE_INIT_FEATURE_DIS;

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        ESP_LOGE("CORE", "TRX failed");
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_core_standby(nfc7i2c_t *ctx, uint8_t mode)
{
    if (mode > NFC7I2C_NCI_CORE_STANDBY_AUTO)
    {
        return NFC7I2C_ERROR;
    }

    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_PROP;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_PROP_SET_PWR_MODE;
    ctx->pkt_ctrl.payload_len = 1;
    ctx->pkt_ctrl.payload[0] = mode;

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_core_total_duration(nfc7i2c_t *ctx)
{
    uint8_t core_cfg_pl[] = NFC7I2C_NCI_CORE_TOTAL_DURATION_510MS;
    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_CORE;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_CORE_SET_CONFIG;
    ctx->pkt_ctrl.payload_len = sizeof(core_cfg_pl);
    memcpy(ctx->pkt_ctrl.payload, core_cfg_pl, ctx->pkt_ctrl.payload_len);

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_core_tag_detector(nfc7i2c_t *ctx)
{
    uint8_t core_cfg_pl[] = NFC7I2C_NCI_CORE_TAG_DETECTOR_DIS;
    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_CORE;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_CORE_SET_CONFIG;
    ctx->pkt_ctrl.payload_len = sizeof(core_cfg_pl);
    memcpy(ctx->pkt_ctrl.payload, core_cfg_pl, ctx->pkt_ctrl.payload_len);

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_core_clock_sel(nfc7i2c_t *ctx)
{
    uint8_t core_cfg_pl[] = NFC7I2C_NCI_CORE_CLOCK_SEL_XTAL;
    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_CORE;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_CORE_SET_CONFIG;
    ctx->pkt_ctrl.payload_len = sizeof(core_cfg_pl);
    memcpy(ctx->pkt_ctrl.payload, core_cfg_pl, ctx->pkt_ctrl.payload_len);

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_core_pmu(nfc7i2c_t *ctx)
{
    uint8_t core_cfg_pl[] = NFC7I2C_NCI_CORE_PMU_IRQ_EN_TVDD_3V3;
    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_CORE;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_CORE_SET_CONFIG;
    ctx->pkt_ctrl.payload_len = sizeof(core_cfg_pl);
    memcpy(ctx->pkt_ctrl.payload, core_cfg_pl, ctx->pkt_ctrl.payload_len);

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_core_rf_config(nfc7i2c_t *ctx)
{
    uint8_t core_cfg_pl[] = NFC7I2C_NCI_CORE_RF_CONF;
    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_CORE;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_CORE_SET_CONFIG;
    ctx->pkt_ctrl.payload_len = sizeof(core_cfg_pl);
    memcpy(ctx->pkt_ctrl.payload, core_cfg_pl, ctx->pkt_ctrl.payload_len);

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_config_settings(nfc7i2c_t *ctx)
{
    if (NFC7I2C_OK != nfc7i2c_core_standby(ctx, NFC7I2C_NCI_CORE_STANDBY_DISABLE))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_core_total_duration(ctx))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_core_tag_detector(ctx))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_core_clock_sel(ctx))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_core_pmu(ctx))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_core_rf_config(ctx))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_OK != nfc7i2c_core_init(ctx))
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_map_rf_interface(nfc7i2c_t *ctx)
{
    uint8_t disc_map[] = {
        NFC7I2C_NCI_RF_PROT_T1T, NFC7I2C_NCI_RF_MAP_POLL_MODE, NFC7I2C_NCI_RF_INTF_FRAME,
        NFC7I2C_NCI_RF_PROT_T2T, NFC7I2C_NCI_RF_MAP_POLL_MODE, NFC7I2C_NCI_RF_INTF_FRAME,
        NFC7I2C_NCI_RF_PROT_T3T, NFC7I2C_NCI_RF_MAP_POLL_MODE, NFC7I2C_NCI_RF_INTF_FRAME,
        NFC7I2C_NCI_RF_PROT_ISODEP, NFC7I2C_NCI_RF_MAP_POLL_MODE, NFC7I2C_NCI_RF_INTF_ISODEP,
        NFC7I2C_NCI_RF_PROT_MIFARE, NFC7I2C_NCI_RF_MAP_POLL_MODE, NFC7I2C_NCI_RF_INTF_TAGCMD};

    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_PROP;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_PROP_ACT;
    ctx->pkt_ctrl.payload_len = 0;

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DISCOVER_MAP;
    ctx->pkt_ctrl.payload_len = sizeof(disc_map) + 1;
    ctx->pkt_ctrl.payload[0] = sizeof(disc_map) / 3;
    memcpy(&ctx->pkt_ctrl.payload[1], disc_map, sizeof(disc_map));

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_start_discovery(nfc7i2c_t *ctx)
{
    uint8_t rf_disc[] = {
        NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_A, NFC7I2C_NCI_RF_EXE_EVERY_DISC_PERIOD,
        NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_B, NFC7I2C_NCI_RF_EXE_EVERY_DISC_PERIOD,
        NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_F, NFC7I2C_NCI_RF_EXE_EVERY_DISC_PERIOD,
        NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_15693, NFC7I2C_NCI_RF_EXE_EVERY_DISC_PERIOD};

    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DISCOVER;
    ctx->pkt_ctrl.payload_len = sizeof(rf_disc) + 1;
    ctx->pkt_ctrl.payload[0] = sizeof(rf_disc) / 2;
    memcpy(&ctx->pkt_ctrl.payload[1], rf_disc, sizeof(rf_disc));

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }

    return NFC7I2C_OK;
}

err_t nfc7i2c_stop_discovery(nfc7i2c_t *ctx)
{
    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DEACTIVATE;
    ctx->pkt_ctrl.payload_len = 1;
    ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_RF_DEACTIVATE_IDLE;

    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }

    return nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_1S);
}

err_t nfc7i2c_wait_discovery(nfc7i2c_t *ctx, nfc7i2c_rf_intf_t *rf_intf)
{
    do
    {
        if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_INFINITE))
        {
            return NFC7I2C_ERROR;
        }
    } while ((NFC7I2C_NCI_PKT_MT_CTRL_NTF != ctx->pkt_ctrl.msg_type) ||
             (NFC7I2C_NCI_GID_RF_MGMT != ctx->pkt_ctrl.gid) ||
             ((NFC7I2C_NCI_OID_RF_INTF_ACTIVATED != ctx->pkt_ctrl.oid) &&
              (NFC7I2C_NCI_OID_RF_DISCOVER != ctx->pkt_ctrl.oid)));

    ctx->next_tag_protocol = NFC7I2C_NCI_RF_PROT_UNDETERMINED;

    if (NFC7I2C_NCI_OID_RF_INTF_ACTIVATED == ctx->pkt_ctrl.oid)
    {
        rf_intf->intf = ctx->pkt_ctrl.payload[1];
        rf_intf->protocol = ctx->pkt_ctrl.payload[2];
        rf_intf->mode_tech = ctx->pkt_ctrl.payload[3];
        rf_intf->more_tags = false;
        nfc7i2c_fill_intf_info(rf_intf, &ctx->pkt_ctrl.payload[7]);
    }
    else
    {
        rf_intf->intf = NFC7I2C_NCI_RF_INTF_UNDETERMINED;
        rf_intf->protocol = ctx->pkt_ctrl.payload[1];
        rf_intf->mode_tech = ctx->pkt_ctrl.payload[2];
        rf_intf->more_tags = true;

        do
        {
            if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS))
            {
                return NFC7I2C_ERROR;
            }
        } while ((NFC7I2C_NCI_PKT_MT_CTRL_NTF != ctx->pkt_ctrl.msg_type) ||
                 (NFC7I2C_NCI_GID_RF_MGMT != ctx->pkt_ctrl.gid) ||
                 (NFC7I2C_NCI_OID_RF_DISCOVER != ctx->pkt_ctrl.oid));

        ctx->next_tag_protocol = ctx->pkt_ctrl.payload[1];
        while (NFC7I2C_NCI_RF_DISC_MORE_NTF_FOLLOW == ctx->pkt_ctrl.payload[ctx->pkt_ctrl.payload_len - 1])
        {
            nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);
        }

        ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
        ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
        ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DISCOVER_SELECT;
        ctx->pkt_ctrl.payload_len = 3;
        ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_RF_CONN_ID_1;
        ctx->pkt_ctrl.payload[1] = rf_intf->protocol;

        if (NFC7I2C_NCI_RF_PROT_ISODEP == rf_intf->protocol)
        {
            ctx->pkt_ctrl.payload[2] = NFC7I2C_NCI_RF_INTF_ISODEP;
        }
        else if (NFC7I2C_NCI_RF_PROT_NFCDEP == rf_intf->protocol)
        {
            ctx->pkt_ctrl.payload[2] = NFC7I2C_NCI_RF_INTF_NFCDEP;
        }
        else if (NFC7I2C_NCI_RF_PROT_MIFARE == rf_intf->protocol)
        {
            ctx->pkt_ctrl.payload[2] = NFC7I2C_NCI_RF_INTF_TAGCMD;
        }
        else
        {
            ctx->pkt_ctrl.payload[2] = NFC7I2C_NCI_RF_INTF_FRAME;
        }

        if (NFC7I2C_OK == nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
        {
            if (NFC7I2C_NCI_STAT_OK == ctx->pkt_ctrl.payload[0])
            {
                nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);
                if ((NFC7I2C_NCI_PKT_MT_CTRL_NTF == ctx->pkt_ctrl.msg_type) &&
                    (NFC7I2C_NCI_GID_RF_MGMT == ctx->pkt_ctrl.gid) &&
                    (NFC7I2C_NCI_OID_RF_INTF_ACTIVATED == ctx->pkt_ctrl.oid))
                {
                    rf_intf->intf = ctx->pkt_ctrl.payload[1];
                    rf_intf->protocol = ctx->pkt_ctrl.payload[2];
                    rf_intf->mode_tech = ctx->pkt_ctrl.payload[3];
                    nfc7i2c_fill_intf_info(rf_intf, &ctx->pkt_ctrl.payload[7]);
                }
            }
        }
    }

    if (NFC7I2C_NCI_RF_INTF_UNDETERMINED == rf_intf->intf)
    {
        rf_intf->protocol = NFC7I2C_NCI_RF_PROT_UNDETERMINED;
    }

    return NFC7I2C_OK;
}

void nfc7i2c_presence_check(nfc7i2c_t *ctx, nfc7i2c_rf_intf_t *rf_intf)
{
    switch (rf_intf->protocol)
    {
    case NFC7I2C_NCI_RF_PROT_T1T:
        do
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            ctx->pkt_data.cid = NFC7I2C_NCI_RF_CONN_ID_STATIC;
            ctx->pkt_data.payload_len = 7;
            ctx->pkt_data.payload[0] = NFC7I2C_T1T_CMD_RID;
            memset(&ctx->pkt_data.payload[1], 0, 6);
            nfc7i2c_pkt_data_trx(ctx, &ctx->pkt_data);
            nfc7i2c_pkt_data_rx(ctx, &ctx->pkt_data, NFC7I2C_TIMEOUT_100MS);
        } while ((NFC7I2C_NCI_RF_CONN_ID_STATIC == ctx->rsp[0]) && (NFC7I2C_NCI_PKT_DATA_RFU == ctx->rsp[1]));
        break;

    case NFC7I2C_NCI_RF_PROT_T2T:
        do
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            ctx->pkt_data.cid = NFC7I2C_NCI_RF_CONN_ID_STATIC;
            ctx->pkt_data.payload_len = 2;
            ctx->pkt_data.payload[0] = NFC7I2C_T2T_CMD_READ;
            ctx->pkt_data.payload[1] = 0;
            nfc7i2c_pkt_data_trx(ctx, &ctx->pkt_data);
            nfc7i2c_pkt_data_rx(ctx, &ctx->pkt_data, NFC7I2C_TIMEOUT_100MS);
        } while ((NFC7I2C_NCI_RF_CONN_ID_STATIC == ctx->rsp[0]) &&
                 (NFC7I2C_NCI_PKT_DATA_RFU == ctx->rsp[1]) &&
                 (17 == ctx->pkt_data.payload_len));
        break;

    case NFC7I2C_NCI_RF_PROT_T3T:
        do
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
            ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
            ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_T3T_POLLING;
            ctx->pkt_ctrl.payload_len = 4;
            ctx->pkt_ctrl.payload[0] = 0xFF;
            ctx->pkt_ctrl.payload[1] = 0xFF;
            ctx->pkt_ctrl.payload[2] = 0x00;
            ctx->pkt_ctrl.payload[3] = 0x01;
            nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl);
            nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);
        } while ((NFC7I2C_NCI_PKT_MT_CTRL_NTF == ctx->pkt_ctrl.msg_type) &&
                 (NFC7I2C_NCI_GID_RF_MGMT == ctx->pkt_ctrl.gid) &&
                 (NFC7I2C_NCI_OID_RF_T3T_POLLING == ctx->pkt_ctrl.oid) &&
                 ((NFC7I2C_NCI_STAT_OK == ctx->pkt_ctrl.payload[0]) ||
                  (ctx->pkt_ctrl.payload[1] > 0)));
        break;

    case NFC7I2C_NCI_RF_PROT_ISODEP:
        do
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
            ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_PROP;
            ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_PROP_RF_PRES_CHECK;
            ctx->pkt_ctrl.payload_len = 0;
            nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl);
            nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);
        } while ((NFC7I2C_NCI_PKT_MT_CTRL_NTF == ctx->pkt_ctrl.msg_type) &&
                 (NFC7I2C_NCI_GID_PROP == ctx->pkt_ctrl.gid) &&
                 (NFC7I2C_NCI_OID_PROP_RF_PRES_CHECK == ctx->pkt_ctrl.oid) &&
                 (1 == ctx->pkt_ctrl.payload_len) &&
                 (NFC7I2C_NCI_STAT_REJECTED == ctx->pkt_ctrl.payload[0]));
        break;

    case NFC7I2C_NCI_RF_PROT_T5T:
        do
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            ctx->pkt_data.cid = NFC7I2C_NCI_RF_CONN_ID_STATIC;
            ctx->pkt_data.payload_len = 11;
            ctx->pkt_data.payload[0] = 0x26;
            ctx->pkt_data.payload[1] = 0x01;
            ctx->pkt_data.payload[2] = 0x40;
            for (uint8_t cnt = 0; cnt < 8; cnt++)
            {
                ctx->pkt_data.payload[cnt + 3] = rf_intf->info.nfc_vpp.id[cnt];
            }
            nfc7i2c_pkt_data_trx(ctx, &ctx->pkt_data);
            if (NFC7I2C_OK != nfc7i2c_pkt_data_rx(ctx, &ctx->pkt_data, NFC7I2C_TIMEOUT_100MS))
            {
                break;
            }
        } while ((NFC7I2C_NCI_RF_CONN_ID_STATIC == ctx->rsp[0]) &&
                 (NFC7I2C_NCI_PKT_DATA_RFU == ctx->rsp[1]) &&
                 (0 == ctx->rsp[ctx->rsp_len - 1]));
        break;

    case NFC7I2C_NCI_RF_PROT_MIFARE:
        do
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
            ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
            ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DEACTIVATE;
            ctx->pkt_ctrl.payload_len = 1;
            ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_RF_DEACTIVATE_SLEEP;
            nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl);
            nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);

            ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
            ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
            ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DISCOVER_SELECT;
            ctx->pkt_ctrl.payload_len = 3;
            ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_RF_CONN_ID_1;
            ctx->pkt_ctrl.payload[1] = NFC7I2C_NCI_RF_PROT_MIFARE;
            ctx->pkt_ctrl.payload[2] = NFC7I2C_NCI_RF_INTF_TAGCMD;
            nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl);
            nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);
        } while ((NFC7I2C_NCI_PKT_MT_CTRL_NTF == ctx->pkt_ctrl.msg_type) &&
                 (NFC7I2C_NCI_GID_RF_MGMT == ctx->pkt_ctrl.gid) &&
                 (NFC7I2C_NCI_OID_RF_INTF_ACTIVATED == ctx->pkt_ctrl.oid));
        break;

    default:
        break;
    }
}

err_t nfc7i2c_reader_act_next(nfc7i2c_t *ctx, nfc7i2c_rf_intf_t *rf_intf)
{
    rf_intf->more_tags = false;
    if (NFC7I2C_NCI_RF_PROT_UNDETERMINED == ctx->next_tag_protocol)
    {
        rf_intf->intf = NFC7I2C_NCI_RF_INTF_UNDETERMINED;
        rf_intf->protocol = NFC7I2C_NCI_RF_PROT_UNDETERMINED;
        return NFC7I2C_ERROR;
    }

    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DEACTIVATE;
    ctx->pkt_ctrl.payload_len = 1;
    ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_RF_DEACTIVATE_SLEEP;
    if (NFC7I2C_OK != nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        return NFC7I2C_ERROR;
    }
    if (NFC7I2C_NCI_STAT_OK != ctx->pkt_ctrl.payload[0])
    {
        return NFC7I2C_ERROR;
    }
    nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);
    if ((NFC7I2C_NCI_PKT_MT_CTRL_NTF != ctx->pkt_ctrl.msg_type) ||
        (NFC7I2C_NCI_GID_RF_MGMT != ctx->pkt_ctrl.gid) ||
        (NFC7I2C_NCI_OID_RF_DEACTIVATE != ctx->pkt_ctrl.oid))
    {
        return NFC7I2C_ERROR;
    }

    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DISCOVER_SELECT;
    ctx->pkt_ctrl.payload_len = 3;
    ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_RF_CONN_ID_2;
    ctx->pkt_ctrl.payload[1] = rf_intf->protocol;

    if (NFC7I2C_NCI_RF_PROT_ISODEP == rf_intf->protocol)
    {
        ctx->pkt_ctrl.payload[2] = NFC7I2C_NCI_RF_INTF_ISODEP;
    }
    else if (NFC7I2C_NCI_RF_PROT_NFCDEP == rf_intf->protocol)
    {
        ctx->pkt_ctrl.payload[2] = NFC7I2C_NCI_RF_INTF_NFCDEP;
    }
    else if (NFC7I2C_NCI_RF_PROT_MIFARE == rf_intf->protocol)
    {
        ctx->pkt_ctrl.payload[2] = NFC7I2C_NCI_RF_INTF_TAGCMD;
    }
    else
    {
        ctx->pkt_ctrl.payload[2] = NFC7I2C_NCI_RF_INTF_FRAME;
    }

    if (NFC7I2C_OK == nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl))
    {
        if (NFC7I2C_NCI_STAT_OK == ctx->pkt_ctrl.payload[0])
        {
            nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);
            if ((NFC7I2C_NCI_PKT_MT_CTRL_NTF == ctx->pkt_ctrl.msg_type) &&
                (NFC7I2C_NCI_GID_RF_MGMT == ctx->pkt_ctrl.gid) &&
                (NFC7I2C_NCI_OID_RF_INTF_ACTIVATED == ctx->pkt_ctrl.oid))
            {
                rf_intf->intf = ctx->pkt_ctrl.payload[1];
                rf_intf->protocol = ctx->pkt_ctrl.payload[2];
                rf_intf->mode_tech = ctx->pkt_ctrl.payload[3];
                nfc7i2c_fill_intf_info(rf_intf, &ctx->pkt_ctrl.payload[7]);
                return NFC7I2C_OK;
            }
        }
    }

    return NFC7I2C_ERROR;
}

err_t nfc7i2c_reader_re_act(nfc7i2c_t *ctx, nfc7i2c_rf_intf_t *rf_intf)
{
    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DEACTIVATE;
    ctx->pkt_ctrl.payload_len = 1;
    ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_RF_DEACTIVATE_SLEEP;
    nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl);
    nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);

    ctx->pkt_ctrl.msg_type = NFC7I2C_NCI_PKT_MT_CTRL_CMD;
    ctx->pkt_ctrl.gid = NFC7I2C_NCI_GID_RF_MGMT;
    ctx->pkt_ctrl.oid = NFC7I2C_NCI_OID_RF_DISCOVER_SELECT;
    ctx->pkt_ctrl.payload_len = 3;
    ctx->pkt_ctrl.payload[0] = NFC7I2C_NCI_RF_CONN_ID_1;
    ctx->pkt_ctrl.payload[1] = rf_intf->protocol;
    ctx->pkt_ctrl.payload[2] = rf_intf->intf;
    nfc7i2c_pkt_ctrl_trx(ctx, &ctx->pkt_ctrl);
    nfc7i2c_pkt_ctrl_rx(ctx, &ctx->pkt_ctrl, NFC7I2C_TIMEOUT_100MS);
    if ((NFC7I2C_NCI_PKT_MT_CTRL_NTF == ctx->pkt_ctrl.msg_type) &&
        (NFC7I2C_NCI_GID_RF_MGMT == ctx->pkt_ctrl.gid) &&
        (NFC7I2C_NCI_OID_RF_INTF_ACTIVATED == ctx->pkt_ctrl.oid))
    {
        return NFC7I2C_OK;
    }
    return NFC7I2C_ERROR;
}

err_t nfc7i2c_reader_tag_cmd(nfc7i2c_t *ctx, nfc7i2c_pkt_data_t *pkt)
{
    pkt->cid = NFC7I2C_NCI_RF_CONN_ID_STATIC;
    nfc7i2c_pkt_data_trx(ctx, pkt);
    nfc7i2c_pkt_data_rx(ctx, pkt, NFC7I2C_TIMEOUT_1S);

    if ((NFC7I2C_NCI_RF_CONN_ID_STATIC == ctx->rsp[0]) &&
        (NFC7I2C_NCI_PKT_DATA_RFU == ctx->rsp[1]))
    {
        return NFC7I2C_OK;
    }

    return NFC7I2C_ERROR;
}
