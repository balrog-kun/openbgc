/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "encoder.h"
#include "motor.h"

sbgc_motor *sbgc_motor_pwm_new(int pin_uh, int pin_vh, int pin_wh, int pin_en, int pairs, sbgc_encoder *enc);

#endif /* MOTOR_PWM_H */
