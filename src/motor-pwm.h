/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "encoder.h"
#include "motor.h"

struct motor_pwm_calib_data_s {
    float zero_electric_offset;
    int8_t sensor_direction;
};

sbgc_motor *sbgc_motor_pwm_new(int pin_uh, int pin_vh, int pin_wh, int pin_en, int pairs, sbgc_encoder *enc,
        const struct motor_pwm_calib_data_s *calib_data);
bool sbgc_motor_pwm_recalibrate(sbgc_motor *motor);
bool sbgc_motor_pwm_get_calibration(sbgc_motor *motor, struct motor_pwm_calib_data_s *out_data);

#endif /* MOTOR_PWM_H */
