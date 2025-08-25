/* vim: set ts=4 sw=4 sts=4 et : */
#include <Wire.h>

#include "util.h"

#include "sbgc32_i2c_drv.h"

/* Register definitions from I2C_Drv.h (SBGC32_I2C_Drv Reference Manual.pdf v0.5) */
#define I2C_DRV_REG_ENC_RAW_ANGLE  40
#define I2C_DRV_REG_ENC_ANGLE      41
#define I2C_DRV_REG_ENC_INFO        6
#define I2C_DRV_REG_ENC_ERR_CNTR   42

#define I2C_DRV_REG_SET_POWER_ANGLE 0
#define I2C_DRV_REG_SET_POWER       0
#define I2C_DRV_REG_SET_ANGLE       1
#define I2C_DRV_REG_SET_FORCE_POWER 2
#define I2C_DRV_REG_SET_ENABLE      3

#define I2C_DRV_REG_ENC_TYPE        4
#define I2C_DRV_REG_ENC_CONF        5
#define I2C_DRV_REG_ENC_FLD_OFFSET  7

#define I2C_DRV_REG_DEVICE_ID      39
#define I2C_DRV_REG_FIRMWARE_VER   32
#define I2C_DRV_REG_MCU_ID         33
#define I2C_DRV_REG_I2CS_ERR_CNTR  43

#define I2C_DRV_REG_RESET_MODE      8

struct sbgc32_i2c_drv_s {
    sbgc_encoder enc_obj;
    sbgc_motor motor_obj;
    uint8_t addr;
    TwoWire *i2c;
    bool motor_on;
};

static int32_t sbgc32_i2c_drv_encoder_read(sbgc_encoder *enc) {
    struct sbgc32_i2c_drv_s *dev = container_of(enc, struct sbgc32_i2c_drv_s, enc_obj);
    uint8_t lo;

    if (dev->i2c->requestFrom(dev->addr, (uint8_t) 2, I2C_DRV_REG_ENC_ANGLE, 1, true) != 2) {
        return 0;
    }

    lo = dev->i2c->read();
    return (uint32_t) (dev->i2c->read() << 8) | lo;
}

static void sbgc32_i2c_drv_encoder_free(sbgc_encoder *enc) {
    /* refcount? */
}

sbgc_encoder_class sbgc32_i2c_drv_encoder_class = {
    .read  = sbgc32_i2c_drv_encoder_read,
    .free  = sbgc32_i2c_drv_encoder_free,
    .scale = 0x4000 / 360, /* LSBs per 1deg */
};

static int sbgc32_i2c_drv_motor_init(sbgc_motor *motor) {
    return 0;
}

static void sbgc32_i2c_drv_motor_set(sbgc_motor *motor, float vel) {
    struct sbgc32_i2c_drv_s *dev = container_of(motor, struct sbgc32_i2c_drv_s, motor_obj);
    uint16_t power = 0x1000;
    uint16_t angle = vel * (0x10000 / 2 / M_PI);

    dev->i2c->beginTransmission(dev->addr);
    dev->i2c->write(I2C_DRV_REG_SET_POWER_ANGLE);
    dev->i2c->write((uint8_t) (power >> 0));
    dev->i2c->write((uint8_t) (power >> 8));
    dev->i2c->write((uint8_t) (angle >> 0));
    dev->i2c->write((uint8_t) (angle >> 8));
    dev->i2c->endTransmission();
}

static int sbgc32_i2c_drv_motor_on(sbgc_motor *motor) {
    struct sbgc32_i2c_drv_s *dev = container_of(motor, struct sbgc32_i2c_drv_s, motor_obj);

    if (!dev->motor_obj.ready)
        return -1;

    dev->motor_on = true;

    dev->i2c->beginTransmission(dev->addr);
    dev->i2c->write(I2C_DRV_REG_SET_ENABLE);
    dev->i2c->write((uint8_t) 1);
    dev->i2c->endTransmission();

    return 0;
}

static void sbgc32_i2c_drv_motor_off(sbgc_motor *motor) {
    struct sbgc32_i2c_drv_s *dev = container_of(motor, struct sbgc32_i2c_drv_s, motor_obj);

    dev->i2c->beginTransmission(dev->addr);
    dev->i2c->write(I2C_DRV_REG_SET_ENABLE);
    dev->i2c->write((uint8_t) 0);
    dev->i2c->endTransmission();

    dev->motor_on = false;
}

static void sbgc32_i2c_drv_motor_free(sbgc_motor *motor) {
    sbgc32_i2c_drv_motor_off(motor);
}

sbgc_motor_class sbgc32_i2c_drv_motor_class = {
    .set_velocity = sbgc32_i2c_drv_motor_set,
    .powered_init = sbgc32_i2c_drv_motor_init,
    .on           = sbgc32_i2c_drv_motor_on,
    .off          = sbgc32_i2c_drv_motor_off,
    .free         = sbgc32_i2c_drv_motor_free,
};

sbgc32_i2c_drv *sbgc32_i2c_drv_new(uint8_t addr, TwoWire *i2c, enum sbgc32_i2c_drv_encoder_type typ) {
    struct sbgc32_i2c_drv_s *dev = (struct sbgc32_i2c_drv_s *) malloc(sizeof(struct sbgc32_i2c_drv_s));
    int i;

    memset(dev, 0, sizeof(*dev));
    dev->enc_obj.cls = &sbgc32_i2c_drv_encoder_class;
    dev->motor_obj.cls = &sbgc32_i2c_drv_motor_class;
    dev->i2c = i2c;
    dev->addr = addr;
    dev->motor_on = true; /* For safety assume it's on until sbgc32_i2c_drv_motor_off() */

    /* Check device identity */
    if (dev->i2c->requestFrom(dev->addr, (uint8_t) 1, I2C_DRV_REG_DEVICE_ID, 1, true) != 1 ||
            dev->i2c->read() != 0x14)
        goto err;

    /* Set encoder type */
    dev->i2c->beginTransmission(dev->addr);
    dev->i2c->write(I2C_DRV_REG_ENC_TYPE);
    dev->i2c->write((uint8_t) typ);
    dev->i2c->endTransmission();

    for (i = 0; i < 50; i++) {
        delay(1);

        if (dev->i2c->requestFrom(dev->addr, (uint8_t) 1, I2C_DRV_REG_ENC_TYPE, 1, true) != 1)
            goto err;

        if (dev->i2c->read() == typ)
            break;
    }
    if (i == 50)
        goto err;

    dev->i2c->beginTransmission(dev->addr);
    dev->i2c->write(I2C_DRV_REG_ENC_CONF);
    dev->i2c->write((uint8_t) 5); /* I2C fast mode, LPF factor 1 from 0..3 */
    dev->i2c->write((uint8_t) 0);
    dev->i2c->endTransmission();

    sbgc32_i2c_drv_motor_off(&dev->motor_obj);
    dev->motor_obj.ready = true;

    return dev;

err:
    free(dev);
    return NULL;
}

void sbgc32_i2c_drv_free(sbgc32_i2c_drv *dev) {
    free(dev);
}

sbgc_encoder *sbgc32_i2c_drv_get_encoder(sbgc32_i2c_drv *dev) {
    return &dev->enc_obj;
}

sbgc_motor *sbgc32_i2c_drv_get_motor(sbgc32_i2c_drv *dev) {
    return &dev->motor_obj;
}
