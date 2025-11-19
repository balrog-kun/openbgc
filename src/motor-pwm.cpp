/* vim: set ts=4 sw=4 sts=4 et : */
#include <SimpleFOC.h>
extern "C" {
#include "main.h"
#include "moremath.h"
#include "util.h"

#include "motor-pwm.h"
}

struct motor_pwm_s;

class SFOCEncoder : public Sensor {
public:
    obgc_encoder *enc;
    SFOCEncoder() {}
    SFOCEncoder(obgc_encoder *_enc) : enc(_enc) {}
    float getSensorAngle() { return enc->reading_rad; }
};

struct motor_pwm_s {
    obgc_motor motor_obj;
    obgc_foc_driver drv_obj;
    /*
     * TODO: allocate the objects statically here instead of using a pointer+new+delete.
     * In testing doing motor->sfoc_driver = BLDCDriver3PWM(...) in motor_3pwm_new()
     * was causing crashes in init() though, may a compiler problem or something else.
     */
    BLDCMotor *sfoc_motor;
    BLDCDriver3PWM *sfoc_driver;
    SFOCEncoder *sfoc_encoder;
    struct main_loop_cb_s loop_cb;
    bool on;
};

static int motor_pwm_init(struct obgc_motor_s *motor_obj) {
    struct motor_pwm_s *motor = container_of(motor_obj, struct motor_pwm_s, motor_obj);
    int ret;

    if (motor->motor_obj.ready)
        return 0;

    motor->sfoc_motor->enable();
    delay(5);
    ret = motor->sfoc_motor->initFOC(); /* These things can actually fail so handle errors */
    motor->sfoc_motor->disable();
    if (!ret)
        return -1;

    motor->motor_obj.ready = true;
    return 0;
}

static void motor_pwm_set(struct obgc_motor_s *motor_obj, float vel) {
    struct motor_pwm_s *motor = container_of(motor_obj, struct motor_pwm_s, motor_obj);
    motor->sfoc_motor->target = vel;
}

static int motor_pwm_on(struct obgc_motor_s *motor_obj) {
    struct motor_pwm_s *motor = container_of(motor_obj, struct motor_pwm_s, motor_obj);

    if (!motor->motor_obj.ready)
        return -1;

    motor->on = true;
    motor->sfoc_motor->enable();
    return 0;
}

static void motor_pwm_off(struct obgc_motor_s *motor_obj) {
    struct motor_pwm_s *motor = container_of(motor_obj, struct motor_pwm_s, motor_obj);

    if (motor->sfoc_motor->driver)
        motor->sfoc_motor->disable();
    else /* Fallback if the constructor didn't finish */
        motor->sfoc_driver->disable();
    motor->on = false;
}

static void motor_pwm_free(struct obgc_motor_s *motor_obj) {
    struct motor_pwm_s *motor = container_of(motor_obj, struct motor_pwm_s, motor_obj);

    if (motor->on)
        motor_pwm_off(motor_obj);
    main_loop_cb_remove(&motor->loop_cb);
    delete motor->sfoc_motor;
    delete motor->sfoc_encoder; /* May be NULL */
    delete motor->sfoc_driver;
    free(motor);
}

static int motor_pwm_recalibrate(struct obgc_motor_s *motor_obj) {
    struct motor_pwm_s *motor = container_of(motor_obj, struct motor_pwm_s, motor_obj);
    int ret;

    if (!IN_SET(motor->sfoc_motor->motor_status,
            FOCMotorStatus::motor_ready, FOCMotorStatus::motor_calib_failed, FOCMotorStatus::motor_uncalibrated))
        return -1;

    if (motor->on) /* Must be off */
        return -1;

    motor->sfoc_motor->zero_electric_angle = NOT_SET;
    motor->sfoc_motor->sensor_direction = Direction::UNKNOWN;

    motor->sfoc_motor->enable();
    delay(5);
    ret = motor->sfoc_motor->initFOC();
    motor->sfoc_motor->disable();

    motor->motor_obj.ready = !!ret;
    return ret ? 0 : 1;
}

static int motor_pwm_get_calibration(struct obgc_motor_s *motor_obj, struct obgc_motor_calib_data_s *out_data) {
    struct motor_pwm_s *motor = container_of(motor_obj, struct motor_pwm_s, motor_obj);

    if (motor->sfoc_motor->motor_status != FOCMotorStatus::motor_ready)
        return -1;

    out_data->bldc_with_encoder.pole_pairs = motor->sfoc_motor->pole_pairs;
    out_data->bldc_with_encoder.zero_electric_offset = motor->sfoc_motor->zero_electric_angle;
    out_data->bldc_with_encoder.sensor_direction = motor->sfoc_motor->sensor_direction;
    return 0;
}

static obgc_motor_class motor_pwm_class = {
    .set_velocity    = motor_pwm_set,
    .powered_init    = motor_pwm_init,
    .on              = motor_pwm_on,
    .off             = motor_pwm_off,
    .free            = motor_pwm_free,
    .recalibrate     = motor_pwm_recalibrate,
    .get_calibration = motor_pwm_get_calibration,
};

static void motor_drv_pwm_set_phase_voltage(struct obgc_foc_driver_s *drv_obj, float v_q, float v_d, float theta) {
    struct motor_pwm_s *motor = container_of(drv_obj, struct motor_pwm_s, drv_obj);

    motor->sfoc_motor->setPhaseVoltage(v_q * motor->sfoc_driver->voltage_power_supply,
            v_d * motor->sfoc_driver->voltage_power_supply, theta * D2R);
}

static int motor_drv_pwm_on(struct obgc_foc_driver_s *drv_obj) {
    struct motor_pwm_s *motor = container_of(drv_obj, struct motor_pwm_s, drv_obj);

    motor->sfoc_motor->driver->enable();
    return 0;
}

static void motor_drv_pwm_off(struct obgc_foc_driver_s *drv_obj) {
    struct motor_pwm_s *motor = container_of(drv_obj, struct motor_pwm_s, drv_obj);

    motor->sfoc_motor->driver->disable();
}

static void motor_drv_pwm_free(struct obgc_foc_driver_s *drv_obj) {
    struct motor_pwm_s *motor = container_of(drv_obj, struct motor_pwm_s, drv_obj);

    motor_pwm_free(&motor->motor_obj);
}

static obgc_foc_driver_class motor_drv_pwm_class = {
    .set_phase_voltage = motor_drv_pwm_set_phase_voltage,
    .on                = motor_drv_pwm_on,
    .off               = motor_drv_pwm_off,
    .free              = motor_drv_pwm_free,
};

static void motor_pwm_loop(struct motor_pwm_s *motor) {
    if (!motor->motor_obj.ready || !motor->on)
        return;

    motor->sfoc_motor->loopFOC();
    motor->sfoc_motor->move();
}

struct motor_pwm_s *sfoc_3pwm_new(bool is_motor, int pin_uh, int pin_vh, int pin_wh, int pin_en,
        obgc_encoder *enc, const struct obgc_motor_calib_data_s *calib_data) {
    struct motor_pwm_s *motor = (struct motor_pwm_s *) malloc(sizeof(struct motor_pwm_s));

    memset(motor, 0, sizeof(*motor));
    motor->motor_obj.cls = &motor_pwm_class;
    motor->drv_obj.cls = &motor_drv_pwm_class;
    motor->on = true; /* For safety assume it's on until motor_pwm_off() */

    if (is_motor)
        motor->sfoc_encoder = new SFOCEncoder(enc);

    motor->sfoc_driver = new BLDCDriver3PWM(pin_uh, pin_vh, pin_wh, pin_en);
    motor->sfoc_driver->voltage_power_supply = 12; /* TODO */
    if (!motor->sfoc_driver->init())
        goto error;

    motor->sfoc_motor = new BLDCMotor(calib_data ? calib_data->bldc_with_encoder.pole_pairs : 11);
    motor->sfoc_motor->linkDriver(motor->sfoc_driver);

    if (is_motor) {
        motor->sfoc_motor->linkSensor(motor->sfoc_encoder);
        motor->sfoc_motor->controller = MotionControlType::velocity;
        motor->sfoc_motor->voltage_limit = 12;
        motor->sfoc_motor->voltage_sensor_align = 5;
        motor->sfoc_motor->velocity_limit = 60 * D2R;
        motor->sfoc_motor->PID_velocity.P = 0.5;
        motor->sfoc_motor->PID_velocity.I = 0.2;
        motor->sfoc_motor->PID_velocity.D = 0.001;
        motor->sfoc_motor->LPF_velocity.Tf = 0.0f;

        if (calib_data) {
            motor->sfoc_motor->zero_electric_angle = calib_data->bldc_with_encoder.zero_electric_offset;
            motor->sfoc_motor->sensor_direction = (enum Direction) calib_data->bldc_with_encoder.sensor_direction;
        }

        if (!motor->sfoc_motor->init())
            goto error;
    }

    motor->sfoc_motor->modulation_centered = 1;
    motor_pwm_class.off(&motor->motor_obj);

    /* TODO: eventually switch to torque control so we can let the user override position by hand like the official firmware does?
     * maybe we just need to monitor the torque but can still use velocity control loop */

    if (is_motor) {
        motor->loop_cb.cb = (void (*)(void *)) motor_pwm_loop;
        motor->loop_cb.data = motor;
        main_loop_cb_add(&motor->loop_cb);
    }

    return motor;

error:
    motor_pwm_class.free(&motor->motor_obj);
    return NULL;
}

obgc_motor *motor_3pwm_new(int pin_uh, int pin_vh, int pin_wh, int pin_en,
        obgc_encoder *enc, const struct obgc_motor_calib_data_s *calib_data) {
    struct motor_pwm_s *motor = sfoc_3pwm_new(true, pin_uh, pin_vh, pin_wh, pin_en, enc, calib_data);

    return motor ? &motor->motor_obj : NULL;
}

obgc_foc_driver *motor_drv_3pwm_new(int pin_uh, int pin_vh, int pin_wh, int pin_en) {
    struct motor_pwm_s *motor = sfoc_3pwm_new(false, pin_uh, pin_vh, pin_wh, pin_en, NULL, NULL);

    return motor ? &motor->drv_obj : NULL;
}
