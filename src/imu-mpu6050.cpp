/* vim: set ts=4 sw=4 sts=4 et : */
#include "imu-mpu6050.h"

/* Register definitions */
#define MPU6050_REG_PWR_MGMT_1   0x6b
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1a
#define MPU6050_REG_GYRO_CONFIG  0x1b
#define MPU6050_REG_ACCEL_CONFIG 0x1c
#define MPU6050_REG_FIFO_EN      0x23
#define MPU6050_REG_USER_CTRL    0x6a
#define MPU6050_REG_FIFO_COUNT   0x72
#define MPU6050_REG_FIFO_R_W     0x74
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_DATA_START   0x3b

struct mpu6050_s {
    obgc_imu obj;
    uint8_t i2c_addr;
    obgc_i2c *i2c;
    int16_t last_temp;
};

static void mpu6050_free(struct mpu6050_s *dev) {
    dev->i2c->beginTransmission(dev->i2c_addr);
    dev->i2c->write(MPU6050_REG_PWR_MGMT_1);
    dev->i2c->write(0x87); /* DEVICE_RESET and stop the clk */
    dev->i2c->endTransmission();
    free(dev);
}

static void mpu6050_read_main(struct mpu6050_s *dev, int32_t *accel, int32_t *gyro) {
#ifdef USE_FIFO
    uint16_t byte_cnt = 0;
    uint8_t sample_cnt;
    int32_t gyro_total[3] = {};

    uint8_t reg_data_size = 8;
# define FIFO_SAMPLE_SIZE 6
    bool fifo_failsafe = false;
#else
    uint8_t reg_data_size = 14;
#endif

    uint8_t buffer[14];

#ifdef USE_FIFO
# ifdef BLOCK_UNTIL_NEW_DATA
    while (gyro && byte_cnt < FIFO_SAMPLE_SIZE) {
# else
    if (gyro) {
# endif
        if (dev->i2c->requestFrom(dev->i2c_addr, (uint8_t) 2, MPU6050_REG_FIFO_COUNT, 1, true) != 2) {
            /* TODO: report error */
            return;
        }

        byte_cnt = (uint16_t) dev->i2c->read() << 8;
        byte_cnt |= dev->i2c->read();
    }

    sample_cnt = byte_cnt / FIFO_SAMPLE_SIZE;
    if (byte_cnt > sample_cnt * FIFO_SAMPLE_SIZE) {
        /* This may happen for two reasons, separately or both at once:
         *
         * If the FIFO has overflowed (FIFO_COUNT returned 1024), there will be
         * 1024 % FIFO_SAMPLE_SIZE bytes at the beginning of the FIFO belonging to a
         * partially overwritten sample and we'll need to discard these bytes to align with
         * the initial bytes of the rest of the samples in the FIFO.
         *
         * Separately it seems that FIFO_COUNT is not updated atomically and even without an
         * overflow may return a value not divisible by FIFO_SAMPLE_SIZE (and not even
         * divisble by 2).  In that case we must not discard any bytes from the beginning
         * because the samples are not offset/misaligned -- the partial sample is
         * (momentarily) at the end of the FIFO.  But we need to leave the last
         * FIFO_COUNT % FIFO_SAMPLE_SIZE bytes in the FIFO for a future read (or wait and
         * retry).
         *
         * The real question, though, is whether the MPU60x0's internal FIFO start (think of
         * a pointer into a ring buffer) is updated atomically -- it probably isn't.  And if
         * it isn't then we have no reliable strategy to handle the overflowed FIFO.  On
         * overflow, FIFO_COUNT will always read 1024 but the samples inside the FIFO may be
         * offset from the beginning by a value that's neither 1024 % FIFO_SAMPLE_SIZE nor 0,
         * and we cannot know.
         *
         * Conceivably, even if the FIFO hasn't overflowed at the time we read FIFO_COUNT,
         * it may have been in the process of writing the sample that will cause the oveflow
         * by the time we read FIFO_R_W and we're also screwed.
         *
         * Consequently handle this scenario by emptying the FIFO and returning only the
         * last sample from the DATA_START area.  Separately do check whether the gyro
         * readings are nonsense in all the places where we expect we may have an overflow,
         * which is in the calibration code.
         */
        byte_cnt = sample_cnt * FIFO_SAMPLE_SIZE;
    }

    if (byte_cnt >= 1024 - FIFO_SAMPLE_SIZE || !sample_cnt) {
        reg_data_size += FIFO_SAMPLE_SIZE;
        fifo_failsafe = true;

        /* Recover by emptying the FIFO.
         *
         * FIFO_RESET in USER_CTRL register might also work but in tests it didn't seem to help in
         * this scenario, neither with nor without the wait for FIFO_RESET to become 0.
         */
        while (byte_cnt) {
            uint8_t byte_cnt_chunk = min((int) byte_cnt, 255);

            byte_cnt -= byte_cnt_chunk;
            if (dev->i2c->requestFrom(dev->i2c_addr, byte_cnt_chunk, MPU6050_REG_FIFO_R_W, 1, true) !=
                    byte_cnt_chunk) {
                /* TODO: report error */
                return;
            }

            if (byte_cnt)
                continue;

            /* Re-read FIFO_COUNT.  If we've read the number of bytes it reported before,
             * it should have gone done to 0.  If it hasn't gone up by a single byte over
             * the time it took us to read the buffer then the MPU60x0 probably wasn't in
             * the middle of an update and we can assume the FIFO start is aligned with
             * the beginning of a sample.  Fingers crossed.
             */
            if (dev->i2c->requestFrom(dev->i2c_addr, (uint8_t) 2, MPU6050_REG_FIFO_COUNT, 1, true) != 2) {
                /* TODO: report error */
                return;
            }

            byte_cnt = (uint16_t) dev->i2c->read() << 8;
            byte_cnt |= dev->i2c->read();
        }
    }
#endif

    if (dev->i2c->requestFrom(dev->i2c_addr, reg_data_size, MPU6050_REG_DATA_START, 1, true) != reg_data_size) {
        /* TODO: report error */
        return;
    }

    for (uint8_t i = 0; i < reg_data_size; i++)
        buffer[i] = dev->i2c->read();

    /* Accelerometer data */
    if (accel) {
        accel[0] = (int32_t) (int16_t) (buffer[0] << 8 | buffer[1]) << 2;
        accel[1] = (int32_t) (int16_t) (buffer[2] << 8 | buffer[3]) << 2;
        accel[2] = (int32_t) (int16_t) (buffer[4] << 8 | buffer[5]) << 2;
    }

    /* Temperature data */
    dev->last_temp = (int16_t) (buffer[6] << 8 | buffer[7]);

    if (!gyro)
        return;

    /* Gyroscope data */
#ifdef USE_FIFO
    if (fifo_failsafe)
        /* Return previous reading if no data and non-blocking.  Not ideal.  TODO: error reporting */
        goto failsafe;

    for (uint8_t i = 0; i < sample_cnt;) {
        uint8_t sample_cnt_chunk = min((int) sample_cnt - i, 255 / FIFO_SAMPLE_SIZE);

        i += sample_cnt_chunk;
        byte_cnt = sample_cnt_chunk * FIFO_SAMPLE_SIZE;

        if (dev->i2c->requestFrom(dev->i2c_addr, byte_cnt, MPU6050_REG_FIFO_R_W, 1, true) != byte_cnt) {
            /* TODO: report error */
            return;
        }

        while (sample_cnt_chunk--) {
            buffer[0] = dev->i2c->read();
            gyro_total[0] += (int16_t) (buffer[0] << 8 | dev->i2c->read());
            buffer[0] = dev->i2c->read();
            gyro_total[1] += (int16_t) (buffer[0] << 8 | dev->i2c->read());
            buffer[0] = dev->i2c->read();
            gyro_total[2] += (int16_t) (buffer[0] << 8 | dev->i2c->read());
        }
    }

    {
        uint16_t c = sample_cnt / 2; /* Compensate for the rounding down when dividing by sample_cnt */
        if (gyro_total[0] > 0)
            gyro_total[0] += c;
        else
            gyro_total[0] -= c;
        if (gyro_total[1] > 0)
            gyro_total[1] += c;
        else
            gyro_total[1] -= c;
        if (gyro_total[2] > 0)
            gyro_total[2] += c;
        else
            gyro_total[2] -= c;
    }

    /* Note: could shift this and mpu6050_imu_class.gyro_scale left by a few bits to avoid
     * the rounding errors even more.  However even at current scale the noise seems to be
     * way over 1 LSB.
     */
    gyro[0] = gyro_total[0] / sample_cnt;
    gyro[1] = gyro_total[1] / sample_cnt;
    gyro[2] = gyro_total[2] / sample_cnt;
    return;

failsafe:
#endif
    gyro[0] = (int16_t) (buffer[8] << 8 | buffer[9]);
    gyro[1] = (int16_t) (buffer[10] << 8 | buffer[11]);
    gyro[2] = (int16_t) (buffer[12] << 8 | buffer[13]);
}

static void mpu6050_read_temp(struct mpu6050_s *dev, int32_t *temp) {
    *temp = ((int32_t) dev->last_temp << 16) / 340 + (int32_t) (36.53f * 65536);
}

static obgc_imu_class mpu6050_imu_class = {
    .read_main   = (void (*)(obgc_imu *imu, int32_t *accel_out, int32_t *gyro_out)) mpu6050_read_main,
    .read_temp   = (void (*)(obgc_imu *imu, int32_t *temp_out)) mpu6050_read_temp,
    .free        = (void (*)(obgc_imu *imu)) mpu6050_free,
    .accel_scale = 65536, /* LSBs per 1g */
    .gyro_scale  = 131,   /* LSBs per 1deg/s */
};

obgc_imu *mpu6050_new(uint8_t i2c_addr, obgc_i2c *i2c) {
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

#define USE_FIFO
#define BLOCK_UNTIL_NEW_DATA

    /* Initialization sequence */
    static uint8_t init_sequence[] = {
        /* Values 0-5 all work and it looks like other factors dominate gyro stddev, hard to see any dependency */
        MPU6050_REG_PWR_MGMT_1,   0x01, /* X-gyro as PLL clock source */
        MPU6050_REG_SMPLRT_DIV,   0x00, /* 1kHz gyro sample rate (8kHz without DLPF) */
        MPU6050_REG_CONFIG,       0x02, /* DLPF_CFG 0 = no LPF, 6 = slowest filter */
        MPU6050_REG_ACCEL_CONFIG, 0x00,
        MPU6050_REG_GYRO_CONFIG,  0x00,
#ifdef USE_FIFO
        MPU6050_REG_USER_CTRL,    0x04, /* Reset FIFO */
        MPU6050_REG_FIFO_EN,      0x70, /* Enable FIFO for gyro readings */
        MPU6050_REG_USER_CTRL,    0x40, /* Enable FIFO in general */
#endif
    };

    for (uint8_t i = 0; i < sizeof(init_sequence); i += 2) {
        dev->i2c->beginTransmission(dev->i2c_addr);
        dev->i2c->write(init_sequence[i]);
        dev->i2c->write(init_sequence[i + 1]);
        dev->i2c->endTransmission();
    }

    /* TODO: do we need to wait for FIFO_RESET to clear befor setting FIFO_EN?  Do we even want to do it here
     * rather than lazily in read_main()? */

    /* TODO: do we need the MPU60x0 revision vs. accelerometer resolution logic as seen in Android and cleanflight?
     * https://github.com/cleanflight/cleanflight/blob/master/src/main/drivers/accgyro/accgyro_mpu.c#L78 */

    return &dev->obj;
}

void mpu6050_set_clksrc(obgc_imu *imu, uint8_t val) {
    struct mpu6050_s *dev = (struct mpu6050_s *) imu;

    dev->i2c->beginTransmission(dev->i2c_addr);
    dev->i2c->write(MPU6050_REG_PWR_MGMT_1);
    dev->i2c->write(val);
    dev->i2c->endTransmission();
}

void mpu6050_set_srate(obgc_imu *imu, uint8_t smplrt_div, uint8_t dlpf_cfg) {
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
