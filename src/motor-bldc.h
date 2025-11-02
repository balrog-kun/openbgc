/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MOTOR_BLDC_H
#define MOTOR_BLDC_H

#include "encoder.h"
#include "motor.h"

extern obgc_motor_class motor_bldc_class;

typedef enum obgc_motor_bldc_param_e {
    BLDC_PARAM_KP, /* roughly [V / (deg/s)], should be [A / (deg/s)] */
    BLDC_PARAM_KI, /* eventually should be same as Kp by time? */
    BLDC_PARAM_KD, /* [s] */
    BLDC_PARAM_KI_FALLOFF,
    BLDC_PARAM_K_DRAG,
    BLDC_PARAM_K_COULOMB,
    BLDC_PARAM_K_STICTION,
    BLDC_PARAM_V_MAX, /* eventually should be in [V] */

    __BLDC_PARAM_MAX
} obgc_motor_bldc_param;

obgc_motor *motor_bldc_new(obgc_encoder *enc, obgc_foc_driver *driver,
        const struct obgc_motor_calib_data_s *calib_data);
void motor_bldc_set_param(obgc_motor *motor, obgc_motor_bldc_param param, float val);
void motor_bldc_override_cur_omega(obgc_motor *motor, float val);

#endif /* MOTOR_BLDC_H */
