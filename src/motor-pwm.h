/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "encoder.h"
#include "motor.h"

obgc_foc_driver *motor_drv_3pwm_new(int pin_uh, int pin_vh, int pin_wh, int pin_en);

obgc_motor *motor_3pwm_new(int pin_uh, int pin_vh, int pin_wh, int pin_en, obgc_encoder *enc,
        const struct obgc_motor_calib_data_s *calib_data);

#endif /* MOTOR_PWM_H */
