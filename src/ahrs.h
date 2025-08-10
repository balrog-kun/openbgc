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

typedef struct sbgc_ahrs_s {
    sbgc_imu *imu;
    float q[4];           /* Quaternion (w, x, y, z) */
    float beta;           /* Should map to SBGC gyro trust param? used differently with Mahony, user setting ignored */
    float acc_kp;
    float enc_kp;
    uint32_t last_update; /* in micros */
    uint8_t axis_map[3];  /* 0:X, 1:Y, 2:Z */
    int8_t axis_sign[3];  /* 1 or -1 */
    float gyro_bias[3];
    float gyro_stddev;
    float gyro_lpf[3];
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
} sbgc_ahrs;

/*
 * TODO: make the calibration non-blocking, only save a flag in calibrate(), do the actual maths in update()
 * When ready and some time has passed set a ready flag.  The main loop should look at ready flags from all
 * components and only enable motors when all are ready, disable them again when something becomes not ready
 */

sbgc_ahrs *ahrs_new(sbgc_imu *imu, sbgc_imu_axis axis_top, sbgc_imu_axis axis_right);
void ahrs_free(sbgc_ahrs *ahrs);
void ahrs_calibrate(sbgc_ahrs *ahrs);
void ahrs_reset_orientation(sbgc_ahrs *agrs);
void ahrs_update(sbgc_ahrs *ahrs);
void ahrs_set_encoder_q(sbgc_ahrs *ahrs, const float *encoder_q);
void ahrs_set_weights(sbgc_ahrs *ahrs, float beta, float acc_kp, float enc_kp, float encoder_step);
void ahrs_set_debug(sbgc_ahrs *ahrs, void (*fn)(const char *));

#endif /* AHRS_H */
