#include "m24sr64y.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

#define m_i2c_transmit(i2c, buf, to) i2c_master_transmit(i2c, buf, sizeof(buf), to)
#define m_i2c_trans_recv(i2c, buf, out, rs, to) i2c_master_transmit_receive(i2c, buf, sizeof(buf), out, rs, to)

// Start an I2C session when RF is active
uint8_t kill_rf_session[] = {0x52};
// Tries to start an I2C session
uint8_t get_i2c_session[] = {0x26};
// Select NDEF app
uint8_t select_ndef_app[] = {0x00, 0xA4, 0x04, 0x00, 0x07, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, 0x00};
// Select NDEF file
uint8_t select_ndef_file[] = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0x00, 0x01};
// Read from NDEF
uint8_t read_length[] = {0x00, 0xB0, 0x00, 0x00, 0x04};

/**
 * Writes checksum of the frame into the last 2 bytes of the buffer.
 */
void _m24sr64y_checksum(uint8_t *data, size_t len)
{
    uint8_t *crcptr = data + len - 2;
    uint16_t crc = 0x6363;

    for (; data < crcptr; data++)
    {
        uint8_t cb = *data;
        cb = (cb ^ (uint8_t)(crc & 0x00FF));
        cb = (cb ^ (cb << 4));
        crc = (crc >> 8) ^ ((uint16_t)cb << 8) ^ ((uint16_t)cb << 3) ^ ((uint16_t)cb >> 4);
    }

    crcptr[0] = crc;
    crcptr[1] = crc >> 8;
}

m24sr64y_t *m24sr64y_create(i2c_master_dev_handle_t device)
{
    m24sr64y_t *this = malloc(sizeof(m24sr64y_t));

    this->dev = device;
    this->state = 0;

    return this;
}

bool m24sr64y_try_session(m24sr64y_t *this)
{
    esp_err_t e = m_i2c_transmit(this->dev, get_i2c_session, -1);
    return (e == ESP_OK);
}

bool m24sr64y_end_session(m24sr64y_t *this)
{
    uint8_t cmdbuf[] = {0xc2, 0, 0};
    uint8_t rcv[4];

    _m24sr64y_checksum(cmdbuf, sizeof(cmdbuf)); // compute frame CRC

    esp_err_t e = m_i2c_transmit(this->dev, cmdbuf, -1);

    // vTaskDelay(1);
    i2c_master_receive(this->dev, rcv, 4, -1);
    return (e == ESP_OK);
}

bool m24sr64y_select_ndef(m24sr64y_t *this)
{
    esp_err_t e;
    uint8_t rcv[5];
    uint8_t cmdbuf[] = {
        0x02 | (this->state ? 1 : 0), // Header
        0x00, 0xA4,                   // Standard Command, Select
        0x04, 0x00,                   // Params
        0x07,                         // 7 bytes of data

        0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, // Select NDEF App

        0x00, // Read len: 0B
        0, 0, // empty space for checksum
    };

    this->state ^= 1; // update block number

    _m24sr64y_checksum(cmdbuf, sizeof(cmdbuf)); // compute frame CRC

    e = m_i2c_transmit(this->dev, cmdbuf, -1);

    if (e != ESP_OK)
    {
        this->state = 0; // reset block number
        return false;
    }

    vTaskDelay(1);

    e = i2c_master_receive(this->dev, rcv, 5, -1);

    if (e != ESP_OK || rcv[1] != 0x90)
        return false;

    return e == ESP_OK;
}

bool m24sr64y_select_file(m24sr64y_t *this, uint16_t id)
{
    esp_err_t e;
    uint8_t rcv[5];
    uint8_t cmdbuf[] = {
        0x02 | (this->state ? 1 : 0),    // Header
        0x00, 0xA4,                      // Standard Command, Select
        0x00, 0x0C,                      // Params: Select files?
        0x02,                            // Lc: 2
        ((id >> 8) & 0xff), (id & 0xff), // Select file
        0, 0,                            // empty space for checksum
    };

    this->state ^= 1; // update block number

    _m24sr64y_checksum(cmdbuf, sizeof(cmdbuf)); // compute frame CRC

    e = m_i2c_transmit(this->dev, cmdbuf, -1);

    if (e != ESP_OK)
    {
        this->state = 0; // reset block number
        return false;
    }

    vTaskDelay(1);

    e = i2c_master_receive(this->dev, rcv, 5, -1);

    if (e != ESP_OK || rcv[1] != 0x90)
        return false;

    return e == ESP_OK;
}

bool m24sr64y_read(m24sr64y_t *this, uint8_t *out, uint16_t offset, uint8_t length)
{
    esp_err_t e;
    uint8_t rcv[256];

    if (length > 246)
        length = 246;

    uint8_t cmdbuf[] = {
        0x02 | (this->state ? 1 : 0),            // Header
        0x00, 0xb0,                              // read
        ((offset >> 8) & 0xff), (offset & 0xff), // offset
        length,                                  // length
        0, 0,                                    // checksum
    };

    this->state ^= 1; // update block number

    _m24sr64y_checksum(cmdbuf, sizeof(cmdbuf));

    e = m_i2c_transmit(this->dev, cmdbuf, -1);

    if (e != ESP_OK)
    {
        this->state = 0; // reset block number
        return false;
    }

    vTaskDelay(1);

    e = i2c_master_receive(this->dev, rcv, length + 5, -1);

    if (e != ESP_OK)
        return false;

    if (rcv[length + 1] != 0x90)
        return false;

    uint8_t *from = rcv + 1;

    while (length > 0)
    {
        *(out++) = *(from++);
        length--;
    }

    return true;
}

bool m24sr64y_write(m24sr64y_t *this, const uint8_t *data, uint16_t offset, uint8_t length)
{
    esp_err_t e;
    uint8_t rcv[5];

    if (length > 246) return false;
    uint8_t cmdsize = 8;
    uint8_t cmdbuf[256] = {
        0x02 | (this->state ? 1 : 0),            // Header
        0x00, 0xd6,                              // read
        ((offset >> 8) & 0xff), (offset & 0xff), // offset
        length,                                  // length
    };

    memcpy(cmdbuf + 6, data, length);
    cmdsize += length;

    this->state ^= 1; // update block number

    _m24sr64y_checksum(cmdbuf, cmdsize);

    e = m_i2c_transmit(this->dev, cmdbuf, -1);

    if (e != ESP_OK)
    {
        this->state = 0; // reset block number
        return false;
    }

    vTaskDelay(1);

    e = i2c_master_receive(this->dev, rcv, 5, -1);

    if (e != ESP_OK || rcv[length + 1] != 0x90)
        return false;

    return true;
}