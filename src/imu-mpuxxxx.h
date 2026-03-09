/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MPUXXXX_H
#define MPUXXXX_H

#include <stdint.h>

#include "imu.h"
#include "i2c.h"

#define MPUXXXX_DEFAULT_ADDR 0x68
#define MPUXXXX_ALT_ADDR 0x69

obgc_imu *mpu6050_new(uint8_t i2c_addr, obgc_i2c *i2c);
obgc_imu *mpu9250_new(uint8_t i2c_addr, obgc_i2c *i2c);

/* Performance / diagnostic settings */
void mpuxxxx_set_clksrc(obgc_imu *imu, uint8_t val);
void mpuxxxx_set_srate(obgc_imu *imu, uint8_t smplrt_div, uint8_t dlpf_cfg);

#endif /* MPUXXXX_H */
