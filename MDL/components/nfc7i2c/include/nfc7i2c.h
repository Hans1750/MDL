#ifndef NFC7I2C_H
#define NFC7I2C_H


#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define NFC7I2C_NCI_PKT_MT_DATA                 0x00
#define NFC7I2C_NCI_PKT_MT_CTRL_CMD             0x01
#define NFC7I2C_NCI_PKT_MT_CTRL_RSP             0x02
#define NFC7I2C_NCI_PKT_MT_CTRL_NTF             0x03
#define NFC7I2C_NCI_PKT_MT_SHIFT                5
#define NFC7I2C_NCI_PKT_MT_MASK                 0xE0
#define NFC7I2C_NCI_PKT_PBF_COMPLETE            0x00
#define NFC7I2C_NCI_PKT_PBF_NOT_COMPLETE        0x01
#define NFC7I2C_NCI_PKT_PBF_SHIFT               4
#define NFC7I2C_NCI_PKT_PBF_MASK                0x10
#define NFC7I2C_NCI_PKT_CTRL_GID_MASK           0x0F
#define NFC7I2C_NCI_PKT_CTRL_OID_MASK           0x3F
#define NFC7I2C_NCI_PKT_DATA_CID_MASK           0x0F
#define NFC7I2C_NCI_PKT_DATA_RFU                0x00

#define NFC7I2C_NCI_GID_CORE                    0x00
#define NFC7I2C_NCI_GID_RF_MGMT                 0x01
#define NFC7I2C_NCI_GID_PROP                    0x0F

#define NFC7I2C_NCI_OID_CORE_RESET              0x00
#define NFC7I2C_NCI_OID_CORE_INIT               0x01
#define NFC7I2C_NCI_OID_CORE_SET_CONFIG         0x02
#define NFC7I2C_NCI_OID_CORE_GET_CONFIG         0x03
#define NFC7I2C_NCI_OID_CORE_CONN_CREATE        0x04
#define NFC7I2C_NCI_OID_CORE_CONN_CLOSE         0x05
#define NFC7I2C_NCI_OID_CORE_CONN_CREDITS       0x06
#define NFC7I2C_NCI_OID_CORE_GEN_ERR_STAT       0x07
#define NFC7I2C_NCI_OID_CORE_INTF_ERR_STAT      0x08
#define NFC7I2C_NCI_OID_RF_DISCOVER_MAP         0x00
#define NFC7I2C_NCI_OID_RF_SET_ROUTING          0x01
#define NFC7I2C_NCI_OID_RF_GET_ROUTING          0x02
#define NFC7I2C_NCI_OID_RF_DISCOVER             0x03
#define NFC7I2C_NCI_OID_RF_DISCOVER_SELECT      0x04
#define NFC7I2C_NCI_OID_RF_INTF_ACTIVATED       0x05
#define NFC7I2C_NCI_OID_RF_DEACTIVATE           0x06
#define NFC7I2C_NCI_OID_RF_FIELD                0x07
#define NFC7I2C_NCI_OID_RF_T3T_POLLING          0x08
#define NFC7I2C_NCI_OID_RF_EE_ACTION            0x09
#define NFC7I2C_NCI_OID_RF_EE_DISCOVERY_REQ     0x0A
#define NFC7I2C_NCI_OID_RF_PARAMETER_UPDATE     0x0B
#define NFC7I2C_NCI_OID_PROP_SET_PWR_MODE       0x00
#define NFC7I2C_NCI_OID_PROP_ACT                0x02
#define NFC7I2C_NCI_OID_PROP_RF_PRES_CHECK      0x11
#define NFC7I2C_NCI_OID_PROP_RF_LPCD_TRACE      0x13
#define NFC7I2C_NCI_OID_PROP_RF_GET_TRANSIT     0x14
#define NFC7I2C_NCI_OID_PROP_TEST_PRBS          0x30
#define NFC7I2C_NCI_OID_PROP_TEST_ANTENNA       0x3D


#define NFC7I2C_NCI_CORE_RESET_KEEP_CFG         0x00
#define NFC7I2C_NCI_CORE_RESET_RESET_CFG        0x01
#define NFC7I2C_NCI_CORE_RESET_NTF_NCI_VER_20   0x20
#define NFC7I2C_NCI_CORE_INIT_FEATURE_DIS       0x00
#define NFC7I2C_NCI_CORE_STANDBY_DISABLE        0x00
#define NFC7I2C_NCI_CORE_STANDBY_ENABLE         0x01
#define NFC7I2C_NCI_CORE_STANDBY_AUTO           0x02


#define NFC7I2C_NCI_STAT_OK                     0x00
#define NFC7I2C_NCI_STAT_REJECTED               0x01
#define NFC7I2C_NCI_STAT_RF_FRAME_CORRUPTED     0x02
#define NFC7I2C_NCI_STAT_FAILED                 0x03
#define NFC7I2C_NCI_STAT_NOT_INITIALIZED        0x04
#define NFC7I2C_NCI_STAT_SYNTAX_ERROR           0x05
#define NFC7I2C_NCI_STAT_SEMANTIC_ERROR         0x06
#define NFC7I2C_NCI_STAT_UNK_GID                0x07
#define NFC7I2C_NCI_STAT_UNK_OID                0x08
#define NFC7I2C_NCI_STAT_INVALID_PARAM          0x09
#define NFC7I2C_NCI_STAT_MSG_SIZE_EXCEEDED      0x0A
#define NFC7I2C_NCI_STAT_DISC_ALRDY_STARTED     0xA0
#define NFC7I2C_NCI_STAT_DISC_T_ACT_FAILED      0xA1
#define NFC7I2C_NCI_STAT_DISC_TEAR_DOWN         0xA2
#define NFC7I2C_NCI_STAT_RF_TX_ERROR            0xB0
#define NFC7I2C_NCI_STAT_RF_PROTOCOL_ERROR      0xB1
#define NFC7I2C_NCI_STAT_RF_TIMEOUT_ERROR       0xB2


#define NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_A  0x00
#define NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_B  0x01
#define NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_NFC_F  0x02
#define NFC7I2C_NCI_RF_TECH_PASSIVE_POLL_15693  0x06
#define NFC7I2C_NCI_RF_EXE_EVERY_DISC_PERIOD    0x01

/**
 * @brief NFC 7 I2C RF protocols setting.
 * @details Specified settings of RF protocols of NFC 7 I2C Click driver.
 */
#define NFC7I2C_NCI_RF_PROT_UNDETERMINED        0x00
#define NFC7I2C_NCI_RF_PROT_T1T                 0x01
#define NFC7I2C_NCI_RF_PROT_T2T                 0x02
#define NFC7I2C_NCI_RF_PROT_T3T                 0x03
#define NFC7I2C_NCI_RF_PROT_ISODEP              0x04
#define NFC7I2C_NCI_RF_PROT_NFCDEP              0x05
#define NFC7I2C_NCI_RF_PROT_T5T                 0x06
#define NFC7I2C_NCI_RF_PROT_MIFARE              0x80

/**
 * @brief NFC 7 I2C RF interface setting.
 * @details Specified settings of RF interface of NFC 7 I2C Click driver.
 */
#define NFC7I2C_NCI_RF_INTF_UNDETERMINED        0x00
#define NFC7I2C_NCI_RF_INTF_FRAME               0x01
#define NFC7I2C_NCI_RF_INTF_ISODEP              0x02
#define NFC7I2C_NCI_RF_INTF_NFCDEP              0x03
#define NFC7I2C_NCI_RF_INTF_TAGCMD              0x80
#define NFC7I2C_NCI_RF_MAP_POLL_MODE            0x01

/**
 * @brief NFC 7 I2C RF deactivate CMD setting.
 * @details Specified settings of RF deactivate CMD of NFC 7 I2C Click driver.
 */
#define NFC7I2C_NCI_RF_DEACTIVATE_IDLE          0x00
#define NFC7I2C_NCI_RF_DEACTIVATE_SLEEP         0x01
#define NFC7I2C_NCI_RF_DEACTIVATE_SLEEP_AF      0x02
#define NFC7I2C_NCI_RF_DEACTIVATE_DISCOVERY     0x03

/**
 * @brief NFC 7 I2C RF discover CMD setting.
 * @details Specified settings of RF discover CMD of NFC 7 I2C Click driver.
 */
#define NFC7I2C_NCI_RF_CONN_ID_STATIC           0x00
#define NFC7I2C_NCI_RF_CONN_ID_1                0x01
#define NFC7I2C_NCI_RF_CONN_ID_2                0x02
#define NFC7I2C_NCI_RF_DISC_MORE_NTF_FOLLOW     0x02

/**
 * @brief NFC 7 I2C T1T commands setting.
 * @details Specified settings of T1T commands of NFC 7 I2C Click driver.
 */
#define NFC7I2C_T1T_CMD_RID                     0x78
#define NFC7I2C_T1T_CMD_RALL                    0x00
#define NFC7I2C_T1T_CMD_READ                    0x01
#define NFC7I2C_T1T_CMD_WRITE_E                 0x53
#define NFC7I2C_T1T_CMD_WRITE_NE                0x1A
#define NFC7I2C_T1T_CMD_RSEG                    0x10
#define NFC7I2C_T1T_CMD_READ8                   0x02
#define NFC7I2C_T1T_CMD_WRITE_E8                0x54
#define NFC7I2C_T1T_CMD_WRITE_NE8               0x1B

/**
 * @brief NFC 7 I2C T2T commands setting.
 * @details Specified settings of T2T commands of NFC 7 I2C Click driver.
 */
#define NFC7I2C_T2T_CMD_READ                    0x30
#define NFC7I2C_T2T_CMD_WRITE                   0xA2
#define NFC7I2C_T2T_CMD_SECTOR_SELECT           0xC2
#define NFC7I2C_T2T_ACK                         0x0A

/**
 * @brief NFC 7 I2C T4T commands setting.
 * @details Specified settings of T4T commands of NFC 7 I2C Click driver.
 */
#define NFC7I2C_T4T_RSP_COMPLETE_1              0x90
#define NFC7I2C_T4T_RSP_COMPLETE_2              0x00
#define NFC7I2C_T4T_CLA_NO_SECURE               0x00
#define NFC7I2C_T4T_INS_SELECT                  0xA4
#define NFC7I2C_T4T_P1_SELECT_BY_NAME           0x04
#define NFC7I2C_T4T_P2_ONLY_OCCURANCE           0x00
#define NFC7I2C_T4T_LE_RSP_MAY_PRESENT          0x00
#define NFC7I2C_T4T_PPSE_APDU                   "2PAY.SYS.DDF01"

/**
 * @brief NFC 7 I2C ISO15693 commands setting.
 * @details Specified settings of ISO15693 commands of NFC 7 I2C Click driver.
 */
#define NFC7I2C_ISO15693_FLAG_DR_HIGH           0x02
#define NFC7I2C_ISO15693_CMD_READ_SINGLE        0x20
#define NFC7I2C_ISO15693_CMD_WRITE_SINGLE       0x21
#define NFC7I2C_ISO15693_RSP_OK                 0x00

/**
 * @brief NFC 7 I2C MIFARE commands setting.
 * @details Specified settings of MIFARE commands of NFC 7 I2C Click driver.
 */
#define NFC7I2C_MFC_REQ_XCHG_DATA               0x10
#define NFC7I2C_MFC_REQ_SECTOR_SEL              0x32
#define NFC7I2C_MFC_REQ_AUTHENTICATE            0x40
#define NFC7I2C_MFC_KEY_SELECTOR_A_EMB          0x10
#define NFC7I2C_MFC_CMD_READ                    0x30
#define NFC7I2C_MFC_CMD_WRITE                   0xA0
#define NFC7I2C_MFC_ACK                         0x0A

/**
 * @brief NFC 7 I2C core config default setting.
 * @details Specified default setting of core config of NFC 7 I2C Click driver.
 */
#define NFC7I2C_NCI_CORE_TOTAL_DURATION_510MS   { 0x01, 0x00, 0x02, 0xFE, 0x01 }
#define NFC7I2C_NCI_CORE_TAG_DETECTOR_DIS       { 0x01, 0xA0, 0x40, 0x01, 0x00 }
#define NFC7I2C_NCI_CORE_CLOCK_SEL_XTAL         { 0x01, 0xA0, 0x03, 0x01, 0x08 }
#define NFC7I2C_NCI_CORE_PMU_IRQ_EN_TVDD_3V3    { 0x01, 0xA0, 0x0E, 0x0B, 0x11, 0x01, 0x01, 0x01, \
                                                  0x00, 0x00, 0x00, 0x10, 0x00, 0xD0, 0x0C }
#define NFC7I2C_NCI_CORE_RF_CONF                { 0x09, \
                                                  0xA0, 0x0D, 0x03, 0x78, 0x0D, 0x02, \
                                                  0xA0, 0x0D, 0x03, 0x78, 0x14, 0x02, \
                                                  0xA0, 0x0D, 0x06, 0x4C, 0x44, 0x65, 0x09, 0x00, 0x00, \
                                                  0xA0, 0x0D, 0x06, 0x4C, 0x2D, 0x05, 0x35, 0x1E, 0x01, \
                                                  0xA0, 0x0D, 0x06, 0x82, 0x4A, 0x55, 0x07, 0x00, 0x07, \
                                                  0xA0, 0x0D, 0x06, 0x44, 0x44, 0x03, 0x04, 0xC4, 0x00, \
                                                  0xA0, 0x0D, 0x06, 0x46, 0x30, 0x50, 0x00, 0x18, 0x00, \
                                                  0xA0, 0x0D, 0x06, 0x48, 0x30, 0x50, 0x00, 0x18, 0x00, \
                                                  0xA0, 0x0D, 0x06, 0x4A, 0x30, 0x50, 0x00, 0x08, 0x00 }

/**
 * @brief NFC 7 I2C timeout setting.
 * @details Specified setting for timeout of NFC 7 I2C Click driver.
 */
#define NFC7I2C_MAX_NCI_FRAME_SIZE              258
#define NFC7I2C_TIMEOUT_INFINITE                0
#define NFC7I2C_TIMEOUT_100MS                   100
#define NFC7I2C_TIMEOUT_1S                      1000
#define NFC7I2C_TIMEOUT_2S                      2000

#define NFC7I2C_DEVICE_ADDRESS_0                0x7C
#define NFC7I2C_DEVICE_ADDRESS_1                0x29
#define NFC7I2C_DEVICE_ADDRESS_2                0x2A
#define NFC7I2C_DEVICE_ADDRESS_3                0x2B

typedef gpio_num_t pin_name_t;
typedef gpio_num_t digital_out_t;
typedef gpio_num_t digital_in_t;
typedef i2c_port_t i2c_master_t;
typedef esp_err_t err_t;


typedef struct
{
    uint8_t sens_res[2];
    uint8_t nfc_id_len;
    uint8_t nfc_id[10];
    uint8_t sel_res_len;
    uint8_t sel_res[1];
    uint8_t rats_len;
    uint8_t rats[20];

} nfc7i2c_rf_intf_info_app_t;

typedef struct
{
    uint8_t sens_res_len;
    uint8_t sens_res[12];
    uint8_t attrib_res_len;
    uint8_t attrib_res[17];

} nfc7i2c_rf_intf_info_bpp_t;

typedef struct
{
    uint8_t bitrate;
    uint8_t sens_res_len;
    uint8_t sens_res[18];

} nfc7i2c_rf_intf_info_fpp_t;

typedef struct
{
    uint8_t afi;
    uint8_t dsf_id;
    uint8_t id[8];

} nfc7i2c_rf_intf_info_vpp_t;

typedef union
{
    nfc7i2c_rf_intf_info_app_t nfc_app;
    nfc7i2c_rf_intf_info_bpp_t nfc_bpp;
    nfc7i2c_rf_intf_info_fpp_t nfc_fpp;
    nfc7i2c_rf_intf_info_vpp_t nfc_vpp;

} nfc7i2c_rf_intf_info_t;

typedef struct
{
    uint8_t intf;                   /**< RF Interface. */
    uint8_t protocol;               /**< RF Protocol. */
    uint8_t mode_tech;              /**< RF technology and mode. */
    uint8_t more_tags;              /**< More tags discovered flag (true or false). */
    nfc7i2c_rf_intf_info_t info;    /**< Discovered remote device properties information. */

} nfc7i2c_rf_intf_t;

typedef struct
{
    uint8_t msg_type;               /**< Message type field (MT). */
    uint8_t gid;                    /**< Group Identifier (GID). */
    uint8_t oid;                    /**< Opcode Identifier (OID). */
    uint8_t payload_len;            /**< Payload Length (L). */
    uint8_t payload[ NFC7I2C_MAX_NCI_FRAME_SIZE - 3 ];  /**< Payload. */

} nfc7i2c_pkt_ctrl_t;

typedef struct
{
    uint8_t cid;                    /**< Connection Identifier (Conn ID). */
    uint8_t payload_len;            /**< Payload Length (L). */
    uint8_t payload[ NFC7I2C_MAX_NCI_FRAME_SIZE - 3 ];  /**< Payload. */

} nfc7i2c_pkt_data_t;

typedef struct
{
    // Output pins
    digital_out_t ven;              /**< Reset pin (active low). */

    // Input pins
    digital_in_t irq;               /**< Interrupt request pin. */

    // Modules
    i2c_master_t i2c;               /**< I2C driver object. */

    // I2C slave address
    uint8_t slave_address;          /**< Device slave address (used for I2C driver). */

    nfc7i2c_pkt_ctrl_t pkt_ctrl;    /**< NCI Packet control. */
    nfc7i2c_pkt_data_t pkt_data;    /**< NCI Packet data. */
    uint8_t cmd[ NFC7I2C_MAX_NCI_FRAME_SIZE ];  /**< NCI command buffer. */
    uint8_t rsp[ NFC7I2C_MAX_NCI_FRAME_SIZE ];  /**< NCI response buffer. */
    uint16_t rsp_len;               /**< NCI response length. */

    uint8_t fw_version[ 3 ];        /**< FW version (ROM.major.minor). */
    uint8_t next_tag_protocol;      /**< Next discovered tag protocol. */

} nfc7i2c_t;

typedef struct
{
    pin_name_t scl;                 /**< Clock pin descriptor for I2C driver. */
    pin_name_t sda;                 /**< Bidirectional data pin descriptor for I2C driver. */

    pin_name_t ven;                 /**< Reset pin (active low). */
    pin_name_t irq;                 /**< Interrupt request pin. */

    uint32_t   i2c_speed;           /**< I2C serial speed. */
    uint8_t    i2c_address;         /**< I2C slave address. */

    i2c_port_t i2c_port;


} nfc7i2c_cfg_t;

typedef enum
{
    NFC7I2C_OK = 0,
    NFC7I2C_ERROR = -1

} nfc7i2c_return_value_t;


void nfc7i2c_cfg_setup ( nfc7i2c_cfg_t *cfg );

err_t nfc7i2c_init ( nfc7i2c_t *ctx, nfc7i2c_cfg_t *cfg );

err_t nfc7i2c_default_cfg ( nfc7i2c_t *ctx );

void nfc7i2c_enable_device ( nfc7i2c_t *ctx );

void nfc7i2c_disable_device ( nfc7i2c_t *ctx );

void nfc7i2c_reset_device ( nfc7i2c_t *ctx );

uint8_t nfc7i2c_get_irq_pin ( nfc7i2c_t *ctx );

err_t nfc7i2c_tx ( nfc7i2c_t *ctx, uint8_t *data_in, uint16_t len );

err_t nfc7i2c_rx ( nfc7i2c_t *ctx, uint8_t *data_out, uint16_t *len, uint16_t timeout );

err_t nfc7i2c_trx ( nfc7i2c_t *ctx, uint8_t *data_in, uint16_t in_len, uint8_t *data_out, uint16_t *out_len );

err_t nfc7i2c_pkt_ctrl_tx ( nfc7i2c_t *ctx, nfc7i2c_pkt_ctrl_t *pkt );

err_t nfc7i2c_pkt_ctrl_rx ( nfc7i2c_t *ctx, nfc7i2c_pkt_ctrl_t *pkt, uint16_t timeout );

err_t nfc7i2c_pkt_ctrl_trx ( nfc7i2c_t *ctx, nfc7i2c_pkt_ctrl_t *pkt );

err_t nfc7i2c_pkt_data_tx ( nfc7i2c_t *ctx, nfc7i2c_pkt_data_t *pkt );

err_t nfc7i2c_pkt_data_rx ( nfc7i2c_t *ctx, nfc7i2c_pkt_data_t *pkt, uint16_t timeout );

err_t nfc7i2c_pkt_data_trx ( nfc7i2c_t *ctx, nfc7i2c_pkt_data_t *pkt );

err_t nfc7i2c_core_reset ( nfc7i2c_t *ctx );

err_t nfc7i2c_core_init ( nfc7i2c_t *ctx );

err_t nfc7i2c_core_standby ( nfc7i2c_t *ctx, uint8_t mode );

err_t nfc7i2c_core_total_duration ( nfc7i2c_t *ctx );
err_t nfc7i2c_core_tag_detector ( nfc7i2c_t *ctx );

err_t nfc7i2c_core_clock_sel ( nfc7i2c_t *ctx );

err_t nfc7i2c_core_pmu ( nfc7i2c_t *ctx );
err_t nfc7i2c_core_rf_config ( nfc7i2c_t *ctx );
err_t nfc7i2c_config_settings ( nfc7i2c_t *ctx );
err_t nfc7i2c_map_rf_interface ( nfc7i2c_t *ctx );
err_t nfc7i2c_start_discovery ( nfc7i2c_t *ctx );
err_t nfc7i2c_stop_discovery ( nfc7i2c_t *ctx );
err_t nfc7i2c_wait_discovery ( nfc7i2c_t *ctx, nfc7i2c_rf_intf_t *rf_intf );
void nfc7i2c_presence_check ( nfc7i2c_t *ctx, nfc7i2c_rf_intf_t *rf_intf );
err_t nfc7i2c_reader_act_next ( nfc7i2c_t *ctx, nfc7i2c_rf_intf_t *rf_intf );
err_t nfc7i2c_reader_re_act ( nfc7i2c_t *ctx, nfc7i2c_rf_intf_t *rf_intf );
err_t nfc7i2c_reader_tag_cmd ( nfc7i2c_t *ctx, nfc7i2c_pkt_data_t *pkt );
#ifdef __cplusplus
}
#endif

#endif // NFC7I2C_H
