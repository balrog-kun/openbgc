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
    struct obgc_motor_pid_stats_s {
        float tracking[3];
        float tracking_dev[3];
    } pid_stats;
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
    float kp_trust, ki_falloff;
    float v_max;
    float kdrag;
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
    void (*set_calibration)(obgc_motor *motor, const obgc_motor_calib_data *data);
    void (*override_cur_velocity)(obgc_motor *motor, float omega);
    void (*set_external_torque)(obgc_motor *motor, float minus_delta_v);
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
    /* .passive_brake() may be issued in any state.  Use .off() to disable.  */
    void (*passive_brake)(obgc_foc_driver *drv);
    void (*free)(obgc_foc_driver *drv);
    void (*beep)(obgc_foc_driver *drv, uint8_t volume, uint16_t freq, uint8_t duration_ms);
} obgc_foc_driver_class;

#endif /* MOTOR_H */
