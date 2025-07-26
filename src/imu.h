/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>

struct sbgc_imu_class_s;

typedef struct sbgc_imu_s {
    struct sbgc_imu_class_s *cls;
} sbgc_imu;

typedef struct sbgc_imu_class_s {
    void (*read_main)(sbgc_imu *imu, int32_t *accel_out, int32_t *gyro_out);
    void (*read_temp)(sbgc_imu *imu, int32_t *temp_out);
    void (*free)(sbgc_imu *imu);
    uint32_t accel_scale;
    uint32_t gyro_scale;
} sbgc_imu_class;

#endif /* IMU_H */
