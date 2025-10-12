/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef SBGC32_I2C_DRV_H
#define SBGC32_I2C_DRV_H

#include <Wire.h>

#include "encoder.h"
#include "motor.h"

#define SBGC32_I2C_DRV_ADDR(n) (0x19 + (n) - 1)

enum sbgc32_i2c_drv_encoder_type {
    SBGC32_I2C_DRV_ENC_TYPE_AS5048A   = 1,
    SBGC32_I2C_DRV_ENC_TYPE_AS5048B   = 2,
    SBGC32_I2C_DRV_ENC_TYPE_AMT203    = 3,
    SBGC32_I2C_DRV_ENC_TYPE_MA3_10BIT = 4,
    SBGC32_I2C_DRV_ENC_TYPE_MA3_12BIT = 5,
    SBGC32_I2C_DRV_ENC_TYPE_ANALOG    = 6,
    SBGC32_I2C_DRV_ENC_TYPE_AS5600    = 7,
    SBGC32_I2C_DRV_ENC_TYPE_AS5050A   = 8,
    SBGC32_I2C_DRV_ENC_TYPE_AS5055A   = 9,
};

typedef struct sbgc32_i2c_drv_s sbgc32_i2c_drv;

sbgc32_i2c_drv *sbgc32_i2c_drv_new(uint8_t addr, TwoWire *i2c, enum sbgc32_i2c_drv_encoder_type typ);
void sbgc32_i2c_drv_free(sbgc32_i2c_drv *dev);
obgc_encoder *sbgc32_i2c_drv_get_encoder(sbgc32_i2c_drv *dev);
obgc_foc_driver *sbgc32_i2c_drv_get_motor_drv(sbgc32_i2c_drv *dev);

#endif /* SBGC32_I2C_DRV_H */
