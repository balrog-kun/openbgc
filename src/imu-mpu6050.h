/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <Wire.h>

#include "imu.h"

#define MPU6050_DEFAULT_ADDR 0x68
#define MPU6050_ALT_ADDR 0x68

obgc_imu *mpu6050_new(uint8_t i2c_addr, TwoWire *i2c);

/* Performance / diagnostic settings */
void mpu6050_set_clksrc(obgc_imu *imu, uint8_t val);
void mpu6050_set_srate(obgc_imu *imu, uint8_t smplrt_div, uint8_t dlpf_cfg);

#endif /* MPU6050_H */
