#include <stdio.h>
#include <inttypes.h>
#include "driver/i2c_master.h"

typedef struct {
    i2c_master_dev_handle_t dev;
    bool state;
} m24sr64y_t;

#define M24SR64Y_FILE_NDEF  0x0001
#define M24SR64Y_FILE_CC    0xE103

m24sr64y_t *m24sr64y_create(i2c_master_dev_handle_t device);

bool m24sr64y_try_session(m24sr64y_t *this);

bool m24sr64y_select_ndef(m24sr64y_t *this);

bool m24sr64y_select_file(m24sr64y_t *this, uint16_t id);

bool m24sr64y_read(m24sr64y_t *this, uint8_t *out, uint16_t offset, uint8_t length);

bool m24sr64y_write(m24sr64y_t *this, const uint8_t *data, uint16_t offset, uint8_t length);

bool m24sr64y_end_session(m24sr64y_t *this);