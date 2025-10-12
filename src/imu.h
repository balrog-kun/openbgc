/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>

struct obgc_imu_class_s;

typedef struct obgc_imu_s {
    struct obgc_imu_class_s *cls;
} obgc_imu;

typedef struct obgc_imu_class_s {
    void (*read_main)(obgc_imu *imu, int32_t *accel_out, int32_t *gyro_out);
    void (*read_temp)(obgc_imu *imu, int32_t *temp_out);
    void (*free)(obgc_imu *imu);
    uint32_t accel_scale;
    uint32_t gyro_scale;
} obgc_imu_class;

#endif /* IMU_H */
