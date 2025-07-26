/* vim: set ts=4 sw=4 sts=4 et : */
#include <Wire.h>

#include "imu-mpu6050.h"

/* Register definitions */
#define MPU6050_REG_PWR_MGMT_1   0x6b
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1a
#define MPU6050_REG_GYRO_CONFIG  0x1b
#define MPU6050_REG_ACCEL_CONFIG 0x1c
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_DATA_START   0x3b

struct mpu6050_s {
    sbgc_imu obj;
    uint8_t i2c_addr;
    TwoWire *i2c;
    int16_t last_temp;
};

extern sbgc_imu_class mpu6050_imu_class;

sbgc_imu *sbgc_mpu6050_new(uint8_t i2c_addr, TwoWire *i2c) {
    struct mpu6050_s *dev = (struct mpu6050_s *) malloc(sizeof(struct mpu6050_s));

    dev->obj.cls = &mpu6050_imu_class;
    dev->i2c = i2c;
    dev->i2c_addr = i2c_addr;

    /* Check device identity */
    if (dev->i2c->requestFrom(dev->i2c_addr, (uint8_t) 1, MPU6050_REG_WHO_AM_I, 1, true) != 1 ||
            dev->i2c->read() != 0x68) {
        free(dev);
        return NULL;
    }

    /* Initialization sequence */
    static uint8_t init_sequence[] = {
        /* Values 0-5 all work and it looks like other factors dominate gyro stddev, hard to see any dependency */
        MPU6050_REG_PWR_MGMT_1,   0x01, /* X-gyro as PLL clock source */
        MPU6050_REG_SMPLRT_DIV,   0x00,
        MPU6050_REG_CONFIG,       0x02, /* DLPF_CFG 0 = no LPF, 6 = slowest filter */
        MPU6050_REG_ACCEL_CONFIG, 0x00,
        MPU6050_REG_GYRO_CONFIG,  0x00
    };

    for (uint8_t i = 0; i < sizeof(init_sequence); i += 2) {
        dev->i2c->beginTransmission(dev->i2c_addr);
        dev->i2c->write(init_sequence[i]);
        dev->i2c->write(init_sequence[i + 1]);
        dev->i2c->endTransmission();
    }

    /* TODO: for the gyro we probably want to enable the FIFO mode, read all the samples available, integrate/average them and report the result, clear the fifo */

    return &dev->obj;
}

static void mpu6050_free(struct mpu6050_s *dev) {
    /* TODO: power chip down? */
    free(dev);
}

static void mpu6050_read_main(struct mpu6050_s *dev, int32_t *accel, int32_t *gyro) {
    uint8_t buffer[14];

    if (dev->i2c->requestFrom(dev->i2c_addr, (uint8_t) 14, MPU6050_REG_DATA_START, 1, true) != 14) {
        /* TODO: report error */
        return;
    }

    for (uint8_t i = 0; i < 14; i++)
        buffer[i] = dev->i2c->read();

    /* Accelerometer data */
    if (accel) {
        accel[0] = (int32_t) (int16_t) (buffer[0] << 8 | buffer[1]) << 2;
        accel[1] = (int32_t) (int16_t) (buffer[2] << 8 | buffer[3]) << 2;
        accel[2] = (int32_t) (int16_t) (buffer[4] << 8 | buffer[5]) << 2;
    }

    /* Temperature data */
    dev->last_temp = (int16_t) (buffer[6] << 8 | buffer[7]);

    /* Gyroscope data */
    if (gyro) {
        gyro[0] = (int16_t) (buffer[8] << 8 | buffer[9]);
        gyro[1] = (int16_t) (buffer[10] << 8 | buffer[11]);
        gyro[2] = (int16_t) (buffer[12] << 8 | buffer[13]);
    }
}

static void mpu6050_read_temp(struct mpu6050_s *dev, int32_t *temp) {
    *temp = ((int32_t) dev->last_temp << 16) / 340 + (int32_t) (36.53f * 65536);
}

sbgc_imu_class mpu6050_imu_class = {
    .read_main   = (void (*)(sbgc_imu *imu, int32_t *accel_out, int32_t *gyro_out)) mpu6050_read_main,
    .read_temp   = (void (*)(sbgc_imu *imu, int32_t *temp_out)) mpu6050_read_temp,
    .free        = (void (*)(sbgc_imu *imu)) mpu6050_free,
    .accel_scale = 65536, /* LSBs per 1g */
    .gyro_scale  = 131,   /* LSBs per 1deg/s */
};

void mpu6050_set_clksrc(sbgc_imu *imu, uint8_t val) {
    struct mpu6050_s *dev = (struct mpu6050_s *) imu;

    dev->i2c->beginTransmission(dev->i2c_addr);
    dev->i2c->write(MPU6050_REG_PWR_MGMT_1);
    dev->i2c->write(val);
    dev->i2c->endTransmission();
}

void mpu6050_set_srate(sbgc_imu *imu, uint8_t smplrt_div, uint8_t dlpf_cfg) {
    struct mpu6050_s *dev = (struct mpu6050_s *) imu;

    dev->i2c->beginTransmission(dev->i2c_addr);
    dev->i2c->write(MPU6050_REG_SMPLRT_DIV);
    dev->i2c->write(smplrt_div);
    dev->i2c->endTransmission();
    dev->i2c->beginTransmission(dev->i2c_addr);
    dev->i2c->write(MPU6050_REG_CONFIG);
    dev->i2c->write(dlpf_cfg & 7);
    dev->i2c->endTransmission();
}
