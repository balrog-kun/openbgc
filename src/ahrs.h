/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef AHRS_H
#define AHRS_H

#include <stdint.h>

#include "imu.h"

/* TODO: can probably drop this since we'll be autocalibrating IMU orientations anyway,
 * we work with native or calibrated coordinates, no need for an intermediate coordinate
 * system.
 */
typedef enum {
    SBGC_IMU_X       = 1,
    SBGC_IMU_Y       = 2,
    SBGC_IMU_Z       = 3,
    SBGC_IMU_MINUS_X = -1,
    SBGC_IMU_MINUS_Y = -2,
    SBGC_IMU_MINUS_Z = -3,
} sbgc_imu_axis;

typedef struct obgc_ahrs_s {
    obgc_imu *imu;
    struct obgc_ahrs_config_s *config;
    float q[4];           /* Quaternion (w, x, y, z) */
    float velocity_vec[3];
    float beta;           /* Should map to SBGC gyro trust param? used differently with Mahony, user setting ignored */
    uint32_t last_update; /* in micros */
    uint8_t axis_map[3];  /* 0:X, 1:Y, 2:Z (Deprecated) */
    int8_t axis_sign[3];  /* 1 or -1 (Deprecated) */
    float gyro_stddev;
    float gyro_lpf[3];
    float error_integral[3];
    float encoder_step;
    const float *encoder_q; /* If set, gets consumed by update(), calibrate() or reset_orientation() */
    void (*debug_print)(const char *);
    uint16_t debug_cnt;
    float gyr_contrib;
    float acc_contrib;
    float acc_error_avg;
    float acc_error_max;
    float enc_contrib;
    float enc_error_avg;
    float enc_error_max;
} obgc_ahrs;

/* Note: changes here may need a STORAGE_CONFIG_VERSION bump in storage.h */
struct obgc_ahrs_config_s {
    float acc_bias[3];
    float acc_sensitivity[3][3];
    float gyro_bias[3];
    float gyro_sensitivity[3][3];
    float acc_kp;
    float enc_kp;
    float ki;
    bool calibrate_on_start;
};

/*
 * TODO: make the calibration non-blocking, only save a flag in calibrate(), do the actual maths in update()
 * When ready and some time has passed set a ready flag.  The main loop should look at ready flags from all
 * components and only enable motors when all are ready, disable them again when something becomes not ready
 */

obgc_ahrs *ahrs_new(obgc_imu *imu, sbgc_imu_axis axis_top, sbgc_imu_axis axis_right,
        struct obgc_ahrs_config_s *config, float encoder_step);
void ahrs_free(obgc_ahrs *ahrs);
void ahrs_calibrate(obgc_ahrs *ahrs);
void ahrs_reset_orientation(obgc_ahrs *agrs);
void ahrs_update(obgc_ahrs *ahrs);
void ahrs_set_encoder_q(obgc_ahrs *ahrs, const float *encoder_q);
void ahrs_set_debug(obgc_ahrs *ahrs, void (*fn)(const char *));

#endif /* AHRS_H */
