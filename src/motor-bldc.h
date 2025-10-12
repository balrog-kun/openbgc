/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MOTOR_BLDC_H
#define MOTOR_BLDC_H

#include "encoder.h"
#include "motor.h"

typedef enum sbgc_motor_bldc_param_e {
    BLDC_PARAM_KP, /* roughly [V / (deg/s)], should be [A / (deg/s)] */
    BLDC_PARAM_KI, /* eventually should be same as Kp by time? */
    BLDC_PARAM_KD, /* [s] */
    BLDC_PARAM_KI_FALLOFF,
    BLDC_PARAM_K_DRAG,
    BLDC_PARAM_K_COULOMB,
    BLDC_PARAM_K_STICTION,
    BLDC_PARAM_V_MAX, /* eventually should be in [V] */

    __BLDC_PARAM_MAX
} sbgc_motor_bldc_param;

sbgc_motor *sbgc_motor_bldc_new(sbgc_encoder *enc, sbgc_foc_driver *driver,
        const struct sbgc_motor_calib_data_s *calib_data);
void sbgc_motor_bldc_set_param(sbgc_motor *motor, sbgc_motor_bldc_param param,
        float val);

#endif /* MOTOR_BLDC_H */
