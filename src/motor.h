/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

struct sbgc_motor_class_s;

typedef struct sbgc_motor_s {
    struct sbgc_motor_class_s *cls;
    bool ready;
} sbgc_motor;

typedef struct sbgc_bldc_with_encoder_calib_data_s {
    uint16_t pole_pairs;
    float zero_electric_offset;
    int8_t sensor_direction;
} sbgc_bldc_with_encoder_calib_data;

typedef struct sbgc_motor_calib_data_s {
    union {
        struct sbgc_bldc_with_encoder_calib_data_s bldc_with_encoder;
    };
} sbgc_motor_calib_data;

typedef struct sbgc_motor_class_s {
    void (*set_velocity)(sbgc_motor *motor, float omega);
    int (*powered_init)(sbgc_motor *motor);
    int (*on)(sbgc_motor *motor);
    void (*off)(sbgc_motor *motor);
    void (*free)(sbgc_motor *motor);
    int (*recalibrate)(sbgc_motor *motor);
    int (*get_calibration)(sbgc_motor *motor, sbgc_motor_calib_data *out_data);
} sbgc_motor_class;

struct sbgc_foc_driver_class_s;

typedef struct sbgc_foc_driver_s {
    struct sbgc_foc_driver_class_s *cls;
} sbgc_foc_driver;

typedef struct sbgc_foc_driver_class_s {
    void (*set_phase_voltage)(sbgc_foc_driver *drv, float v_q, float v_d, float theta);
    int (*on)(sbgc_foc_driver *drv);
    void (*off)(sbgc_foc_driver *drv);
    void (*free)(sbgc_foc_driver *drv);
} sbgc_foc_driver_class;

#endif /* MOTOR_H */
