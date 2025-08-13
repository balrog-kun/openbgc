/* vim: set ts=4 sw=4 sts=4 et : */
#include <SimpleFOC.h>
extern "C" {
#include "main.h"
#include "moremath.h"

#include "motor-pwm.h"
}

struct motor_pwm_s;

class SFOCEncoder : public Sensor {
public:
    sbgc_encoder *enc;
    SFOCEncoder() {}
    SFOCEncoder(sbgc_encoder *_enc) : enc(_enc) {}
    /* TODO: possibly just use what we've already read in main? also pre-multiply */
    float getSensorAngle() { return (float) enc->cls->read(enc) * (D2R / enc->cls->scale); }
};

struct motor_pwm_s {
    sbgc_motor obj;
    /*
     * TODO: allocate the objects statically here instead of using a pointer+new+delete.
     * In testing doing motor->sfoc_driver = BLDCDriver3PWM(...) in sbgc_motor_pwm_new()
     * was causing crashes in init() though, may a compiler problem or something else.
     */
    BLDCMotor *sfoc_motor;
    BLDCDriver3PWM *sfoc_driver;
    SFOCEncoder *sfoc_encoder;
    struct main_loop_cb_s loop_cb;
    bool on;
};

extern sbgc_motor_class motor_pwm_class;

static void motor_pwm_loop(struct motor_pwm_s *motor) {
    if (!motor->obj.ready || !motor->on)
        return;

    motor->sfoc_motor->loopFOC();
    motor->sfoc_motor->move();
}

sbgc_motor *sbgc_motor_pwm_new(int pin_uh, int pin_vh, int pin_wh, int pin_en, int pairs, sbgc_encoder *enc) {
    struct motor_pwm_s *motor = (struct motor_pwm_s *) malloc(sizeof(struct motor_pwm_s));

    memset(motor, 0, sizeof(*motor));
    motor->obj.cls = &motor_pwm_class;
    motor->on = true; /* For safety assume it's on until motor_pwm_off() */

    motor->sfoc_encoder = new SFOCEncoder(enc);

    motor->sfoc_driver = new BLDCDriver3PWM(pin_uh, pin_vh, pin_wh, pin_en);
    motor->sfoc_driver->voltage_power_supply = 12; /* TODO */
    if (motor->sfoc_driver->init()) /* FIXME: inverted in 2.2.1 */
        goto error;

    motor->sfoc_motor = new BLDCMotor(11);
    motor->sfoc_motor->linkDriver(motor->sfoc_driver);
    motor->sfoc_motor->linkSensor(motor->sfoc_encoder);
    motor->sfoc_motor->voltage_limit = 3;
    motor->sfoc_motor->velocity_limit = 60 * D2R;
    motor->sfoc_motor->PID_velocity.P = 0.2;
    motor->sfoc_motor->PID_velocity.I = 2;
    motor->sfoc_motor->PID_velocity.D = 0.1;
    motor->sfoc_motor->LPF_velocity.Tf = 0.001;
    motor->sfoc_motor->init();
    motor_pwm_class.off(&motor->obj);

    /* TODO: eventually switch to torque control so we can let the user override position by hand like the official firmware does?
     * maybe we just need to monitor the torque but can still use velocity control loop */
    /* TODO: save zero_electric_offset and sensor_direction for re-use */

    motor->loop_cb.cb = (void (*)(void *)) motor_pwm_loop;
    motor->loop_cb.data = motor;
    main_loop_cb_add(&motor->loop_cb);

    return &motor->obj;

error:
    motor_pwm_class.free(&motor->obj);
    return NULL;
}

static int motor_pwm_init(struct motor_pwm_s *motor) {
    int ret;

    if (motor->obj.ready)
        return 0;

    motor->sfoc_motor->enable();
    delay(5);
    ret = motor->sfoc_motor->initFOC(); /* These things can actually fail so handle errors */
    motor->sfoc_motor->disable();
    if (!ret)
        return -1;

    motor->obj.ready = true;
    return 0;
}

static void motor_pwm_set(struct motor_pwm_s *motor, float vel) {
    motor->sfoc_motor->target = vel;
}

static int motor_pwm_on(struct motor_pwm_s *motor) {
    if (!motor->obj.ready)
        return -1;

    motor->on = true;
    motor->sfoc_motor->enable();
    return 0;
}

static void motor_pwm_off(struct motor_pwm_s *motor) {
    if (motor->sfoc_motor->driver)
        motor->sfoc_motor->disable();
    else
        motor->sfoc_driver->disable();
    motor->on = false;
}

static void motor_pwm_free(struct motor_pwm_s *motor) {
    if (motor->on)
        motor_pwm_off(motor);
    main_loop_cb_remove(&motor->loop_cb);
    delete motor->sfoc_motor;
    delete motor->sfoc_encoder;
    delete motor->sfoc_driver;
    free(motor);
}

sbgc_motor_class motor_pwm_class = {
    .set_velocity = (void (*)(sbgc_motor *, float)) motor_pwm_set,
    .powered_init = (int (*)(sbgc_motor *)) motor_pwm_init,
    .on           = (int (*)(sbgc_motor *)) motor_pwm_on,
    .off          = (void (*)(sbgc_motor *)) motor_pwm_off,
    .free         = (void (*)(sbgc_motor *)) motor_pwm_free,
};
