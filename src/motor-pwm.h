/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "encoder.h"
#include "motor.h"

sbgc_foc_driver *sbgc_motor_drv_3pwm_new(int pin_uh, int pin_vh, int pin_wh, int pin_en);

sbgc_motor *sbgc_motor_3pwm_new(int pin_uh, int pin_vh, int pin_wh, int pin_en, sbgc_encoder *enc,
        const struct sbgc_motor_calib_data_s *calib_data);

#endif /* MOTOR_PWM_H */
