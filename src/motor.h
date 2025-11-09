/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

struct obgc_motor_class_s;
struct obgc_motor_pid_params_s;

typedef struct obgc_motor_s {
    struct obgc_motor_class_s *cls;
    bool ready;
    struct obgc_motor_pid_params_s *pid_params;
} obgc_motor;

typedef struct obgc_bldc_with_encoder_calib_data_s {
    uint16_t pole_pairs;
    float zero_electric_offset;
    int8_t sensor_direction;
} obgc_bldc_with_encoder_calib_data;

/* Note: changes here may need a STORAGE_CONFIG_VERSION bump in storage.h */
typedef struct obgc_motor_calib_data_s {
    union {
        struct obgc_bldc_with_encoder_calib_data_s bldc_with_encoder;
    };
} obgc_motor_calib_data;

/* Note: changes here may need a STORAGE_CONFIG_VERSION bump in storage.h */
typedef struct obgc_motor_pid_params_s {
    float kp, ki, kd;
    float ki_falloff;
    float v_max;
    /* The below would have been more flexible but at the end of the day the loop code
     * would be traversing the list and copying the parameters so even more memory use.
     */
#if 0
    struct {
        uint8_t key;
        float val;
    } param[10]; /* 0-key-terminated if not full */
#endif
} obgc_motor_pid_params;

typedef struct obgc_motor_class_s {
    void (*set_velocity)(obgc_motor *motor, float omega);
    int (*powered_init)(obgc_motor *motor);
    int (*on)(obgc_motor *motor);
    void (*off)(obgc_motor *motor);
    void (*free)(obgc_motor *motor);
    int (*recalibrate)(obgc_motor *motor);
    int (*get_calibration)(obgc_motor *motor, obgc_motor_calib_data *out_data);
    void (*override_cur_velocity)(obgc_motor *motor, float omega);
    /* TODO: void (*set_torque_threshold)(motor, float threshold, void (*torque_callback)(motor, data), void *data) for manual override */
} obgc_motor_class;

struct obgc_foc_driver_class_s;

typedef struct obgc_foc_driver_s {
    struct obgc_foc_driver_class_s *cls;
} obgc_foc_driver;

typedef struct obgc_foc_driver_class_s {
    void (*set_phase_voltage)(obgc_foc_driver *drv, float v_q, float v_d, float theta);
    int (*on)(obgc_foc_driver *drv);
    void (*off)(obgc_foc_driver *drv);
    void (*free)(obgc_foc_driver *drv);
    /* TODO: void beep_on(drv) */
    /* TODO: void beep_off(drv) */
} obgc_foc_driver_class;

#endif /* MOTOR_H */
