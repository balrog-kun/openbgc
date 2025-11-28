/* vim: set ts=4 sw=4 sts=4 et : */
extern "C" {
#include "main.h"
}

#include "encoder-as5600.h"

/* Register definitions */
#define AS5600_REG_ZMCO      0x00
#define AS5600_REG_ZPOS      0x01
#define AS5600_REG_MPOS      0x03
#define AS5600_REG_MANG      0x05
#define AS5600_REG_CONF      0x07
#define AS5600_REG_RAW_ANGLE 0x0c
#define AS5600_REG_ANGLE     0x0e
#define AS5600_REG_STATUS    0x0b
#define AS5600_REG_AGC       0x1a
#define AS5600_REG_MAGNITUDE 0x1b
#define AS5600_REG_BURN      0xff

struct as5600_s {
    obgc_encoder obj;
    uint8_t i2c_addr;
    obgc_i2c *i2c;
    bool i2c_err;
};

static void as5600_free(struct as5600_s *dev) {
    /* TODO: power chip down? */
    free(dev);
}

static int32_t as5600_read(struct as5600_s *dev) {
    uint8_t hi;

    if (dev->i2c->requestFrom(dev->i2c_addr, (uint8_t) 2, AS5600_REG_RAW_ANGLE, 1, true) != 2) {
        dev->i2c_err = 1;
        dev->i2c->error_cnt++;
        return 0;
    }

    hi = dev->i2c->read() & 0xf;
    return ((uint32_t) (hi << 8) | dev->i2c->read()) << 15;
}

static obgc_encoder_class as5600_encoder_class = {
    .read       = (int32_t (*)(obgc_encoder *enc)) as5600_read,
    .free       = (void (*)(obgc_encoder *enc)) as5600_free,
    .scale      = (4096 << 15) / 360, /* LSBs per 1deg, yields a pretty round value */
    .resolution = 360.0 / 4096,
};

obgc_encoder *as5600_new(obgc_i2c *i2c) {
    struct as5600_s *dev = (struct as5600_s *) malloc(sizeof(struct as5600_s));

    memset(dev, 0, sizeof(*dev));
    dev->obj.cls = &as5600_encoder_class;
    dev->i2c = i2c;
    dev->i2c_addr = 0x36;

    /*
     * It sounds form the spec like there's no simple averaging over multiple
     * samples and we don't want to do it on the MCU side, so we're left with
     * the slow filter option which sounds more like an LPF.
     */
    dev->i2c->beginTransmission(dev->i2c_addr);
    if (dev->i2c->write(AS5600_REG_CONF) != 1) {
        dev->i2c->error_cnt++;
        error_print("AS5600 didn't reply");
        return NULL;
    }

    dev->i2c->write(0x04); /* Slowest "slow filter" (16x) 7 LSBs to switch to "fast filter" */
    dev->i2c->write(0x00); /* No hysteresis,  */
    dev->i2c->endTransmission();

    /*
     * TODO: see if we can play with dynamically setting ZPOS/MPOS to a 45 or 90 deg range
     * around the latest value (with hysteresis) to improve ANGLE reg resolution.  The update
     * would take 2+ms but wouldn't have to be done on every cycle.  Compare with our
     * sample rate.
     * Normal resolution is 0.08 deg / LSB but if this works we can get it down to the promised
     * 0.015 Max. RMS Output Noise specified for the slowest filtering option?
     *
     * For now, before we do this, we can use the extra precision that the chip has but
     * can't give us, to enable the fast-filter option which we do above.
     */

    return &dev->obj;
}

const char *as5600_get_error(obgc_encoder *enc) {
    struct as5600_s *dev = (struct as5600_s *) enc;
    uint8_t b;

    if (dev->i2c_err) {
        dev->i2c_err = 0;
        return "I2C error";
    }

    if (dev->i2c->requestFrom(dev->i2c_addr, (uint8_t) 1, AS5600_REG_STATUS, 1, true) != 1)
        return "I2C error";

    if (!(b & (1 << 5)))
        return "Magnet not detected";

    if (b & (1 << 4))
        return "AGC maximum gain overflow, magnet too weak";

    if (b & (1 << 3))
        return "AGC minimum gain overflow, magnet too strong";

    return NULL;
}
