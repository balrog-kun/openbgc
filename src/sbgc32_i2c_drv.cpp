/* vim: set ts=4 sw=4 sts=4 et : */
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
    obgc_encoder enc_obj;
    obgc_foc_driver motor_drv_obj;
    uint8_t addr;
    obgc_i2c *i2c;
};

static int32_t sbgc32_i2c_drv_encoder_read(obgc_encoder *enc) {
    struct sbgc32_i2c_drv_s *dev = container_of(enc, struct sbgc32_i2c_drv_s, enc_obj);
    uint8_t lo;

    if (dev->i2c->requestFrom(dev->addr, (uint8_t) 2, I2C_DRV_REG_ENC_ANGLE, 1, true) != 2) {
        return 0;
    }

    lo = dev->i2c->read();
    return ((uint32_t) (dev->i2c->read() << 8) | lo) << 13;
}

static void sbgc32_i2c_drv_encoder_free(obgc_encoder *enc) {
    /* refcount? */
}

static obgc_encoder_class sbgc32_i2c_drv_encoder_class = {
    .read  = sbgc32_i2c_drv_encoder_read,
    .free  = sbgc32_i2c_drv_encoder_free,
    .scale = (0x4000 << 13) / 360, /* LSBs per 1deg */
};

static void sbgc32_i2c_drv_motor_set_phase_voltage(obgc_foc_driver *motor_drv, float v_q, float v_d, float theta) {
    struct sbgc32_i2c_drv_s *dev = container_of(motor_drv, struct sbgc32_i2c_drv_s, motor_drv_obj);

    uint16_t power;
    uint16_t angle;

    /* Only handle v_q or v_d but not both at once, for now, since there are no users and it will involve complex trig */
    if (v_d != 0.0f) {
        if (v_d < 0.0f)
            theta += 180.0f;
    } else {
        v_d = v_q;
        theta += v_q > 0.0f ? 90.0f : -90.0f;
    }

    power = fabsf(v_d) * 0xffff;
    angle = (uint16_t) (uint32_t) lroundf(theta * (0x10000 / 360.0f));
    /*
     * Instead of lroundf we could cast to int32_t then uint32_t then uint16_t.  The float to int32_t
     * cast is implementation-defined though and likely rounds towards 0 which is not ideal and could
     * cause a minuscule inconsistency if calib_data->sensor_direction is -1.
     */

    dev->i2c->beginTransmission(dev->addr);
    dev->i2c->write(I2C_DRV_REG_SET_POWER_ANGLE);
    dev->i2c->write((uint8_t) (power >> 0));
    dev->i2c->write((uint8_t) (power >> 8));
    dev->i2c->write((uint8_t) (angle >> 0));
    dev->i2c->write((uint8_t) (angle >> 8));
    dev->i2c->endTransmission();
}

static int sbgc32_i2c_drv_motor_on(obgc_foc_driver *motor_drv) {
    struct sbgc32_i2c_drv_s *dev = container_of(motor_drv, struct sbgc32_i2c_drv_s, motor_drv_obj);

    dev->i2c->beginTransmission(dev->addr);
    dev->i2c->write(I2C_DRV_REG_SET_ENABLE);
    dev->i2c->write((uint8_t) 1);
    dev->i2c->endTransmission();

    return 0;
}

static void sbgc32_i2c_drv_motor_off(obgc_foc_driver *motor_drv) {
    struct sbgc32_i2c_drv_s *dev = container_of(motor_drv, struct sbgc32_i2c_drv_s, motor_drv_obj);

    dev->i2c->beginTransmission(dev->addr);
    dev->i2c->write(I2C_DRV_REG_SET_ENABLE);
    dev->i2c->write((uint8_t) 0);
    dev->i2c->endTransmission();
}

static void sbgc32_i2c_drv_motor_free(obgc_foc_driver *motor_drv) {
    sbgc32_i2c_drv_motor_off(motor_drv);
}

static obgc_foc_driver_class sbgc32_i2c_drv_motor_drv_class = {
    .set_phase_voltage = sbgc32_i2c_drv_motor_set_phase_voltage,
    .on                = sbgc32_i2c_drv_motor_on,
    .off               = sbgc32_i2c_drv_motor_off,
    .free              = sbgc32_i2c_drv_motor_free,
};

sbgc32_i2c_drv *sbgc32_i2c_drv_new(uint8_t addr, obgc_i2c *i2c, enum sbgc32_i2c_drv_encoder_type typ) {
    struct sbgc32_i2c_drv_s *dev = (struct sbgc32_i2c_drv_s *) malloc(sizeof(struct sbgc32_i2c_drv_s));
    int i;

    memset(dev, 0, sizeof(*dev));
    dev->enc_obj.cls = &sbgc32_i2c_drv_encoder_class;
    dev->motor_drv_obj.cls = &sbgc32_i2c_drv_motor_drv_class;
    dev->i2c = i2c;
    dev->addr = addr;

    /* Check device identity */
    if (dev->i2c->requestFrom(dev->addr, (uint8_t) 1, I2C_DRV_REG_DEVICE_ID, 1, true) != 1 ||
            dev->i2c->read() != 0x14)
        goto err;

    /* Safety: power off motor ASAP */
    sbgc32_i2c_drv_motor_off(&dev->motor_drv_obj);

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

    return dev;

err:
    free(dev);
    return NULL;
}

void sbgc32_i2c_drv_free(sbgc32_i2c_drv *dev) {
    free(dev);
}

obgc_encoder *sbgc32_i2c_drv_get_encoder(sbgc32_i2c_drv *dev) {
    return &dev->enc_obj;
}

obgc_foc_driver *sbgc32_i2c_drv_get_motor_drv(sbgc32_i2c_drv *dev) {
    return &dev->motor_drv_obj;
}
