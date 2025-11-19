/* vim: set ts=4 sw=4 sts=4 et : */
#include <SimpleFOC.h>
#include <drivers/hardware_specific/stm32/stm32_mcu.h>
#include <drivers/hardware_specific/stm32/stm32_timerutils.h>
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

    bool beeping;
    bool flipped;
    uint32_t orig_psc0;
    uint32_t orig_arr0;
    unsigned long beep_end_ts;
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

    if (motor->beeping)
        return;

    motor->sfoc_motor->setPhaseVoltage(v_q * motor->sfoc_driver->voltage_power_supply,
            v_d * motor->sfoc_driver->voltage_power_supply, theta * D2R);
}

static int motor_drv_pwm_on(struct obgc_foc_driver_s *drv_obj) {
    struct motor_pwm_s *motor = container_of(drv_obj, struct motor_pwm_s, drv_obj);

    motor->on = true;

    if (motor->beeping)
        return 0;

    motor->sfoc_motor->driver->enable();
    return 0;
}

static void motor_drv_pwm_off(struct obgc_foc_driver_s *drv_obj) {
    struct motor_pwm_s *motor = container_of(drv_obj, struct motor_pwm_s, drv_obj);

    motor->on = false;

    if (motor->beeping)
        return;

    motor->sfoc_motor->driver->disable();
}

static void motor_drv_pwm_free(struct obgc_foc_driver_s *drv_obj) {
    struct motor_pwm_s *motor = container_of(drv_obj, struct motor_pwm_s, drv_obj);

    motor_pwm_free(&motor->motor_obj);
}

static void motor_drv_stm32_flip_polarity(struct motor_pwm_s *motor) {
    STM32DriverParams *stm32_params = (STM32DriverParams *) motor->sfoc_driver->params;

    /* Invert the polarity of all 3 PWM channels to pull the rotor the other direction just
     * in case the pull is actually causing noticeable acceleration, or in case the rotor
     * angle is such that the currents are not causing any pull.
     *
     * Right now we keep two of the outputs LOW and one HIGH for a short time at the
     * beginning of each cycle, of each square wave period, then LOW for the rest of
     * the cycle.  So for a short time there's current flowing through 2 of the windings
     * while the third is shorted, no current.  The rest of the time all windings are
     * shorted so we're probably passively braking.  Then in the next cycle it's the
     * same situation except the currents are flowing in the opposite directions.
     *
     * What we should be doing is perhaps this:
     * 1. have the currents flow in the same way as now, for the same short period of
     *    time in each cycle,
     * 2. then within the same cycle have them flowing in the opposite directions for
     *    the same small amount of time,
     * 3. put all outputs in high-impedance state for the rest of the cycle, which would
     *    require playing with PWM on the ENx pins which we're currently not doing.
     *    (Unless the user requested passive braking, in that case the windings should
     *    be shorted for the rest of the cycle like now)
     *
     * Another difficult and probably not worth it improvement would be to try to, if
     * the motor is on and being drived, keep driving it roughly at the same requested
     * torque while emitting the square wave.  Both these ideas are left as a TODO.
     */
    motor->flipped ^= 1;

    LL_TIM_OC_SetPolarity(stm32_params->timers_handle[0]->Instance, stm32_params->channels[0],
            !LL_TIM_OC_GetPolarity(stm32_params->timers_handle[0]->Instance, stm32_params->channels[0]));
    LL_TIM_OC_SetPolarity(stm32_params->timers_handle[1]->Instance, stm32_params->channels[1],
            !LL_TIM_OC_GetPolarity(stm32_params->timers_handle[1]->Instance, stm32_params->channels[1]));
    LL_TIM_OC_SetPolarity(stm32_params->timers_handle[2]->Instance, stm32_params->channels[2],
            !LL_TIM_OC_GetPolarity(stm32_params->timers_handle[2]->Instance, stm32_params->channels[2]));
}

static void motor_drv_stm32_beep_loop(struct motor_pwm_s *motor) {
    STM32DriverParams *stm32_params = (STM32DriverParams *) motor->sfoc_driver->params;

    if ((signed long) (unsigned long) (millis() - motor->beep_end_ts) < 0) {
        motor_drv_stm32_flip_polarity(motor);
        return;
    }

    /* Restore everything */
    LL_TIM_SetPrescaler(stm32_params->timers_handle[0]->Instance, motor->orig_psc0);
    LL_TIM_SetAutoReload(stm32_params->timers_handle[0]->Instance, motor->orig_arr0);

    if (motor->flipped)
        motor_drv_stm32_flip_polarity(motor);

    if (motor->on)
        motor->sfoc_driver->enable();
    else
        motor->sfoc_driver->disable();

    motor->beeping = false;
    main_loop_cb_remove(&motor->loop_cb);
}

static void motor_drv_stm32_beep(struct obgc_foc_driver_s *drv_obj, uint8_t volume, uint16_t freq,
        uint8_t duration_ms) {
    struct motor_pwm_s *motor = container_of(drv_obj, struct motor_pwm_s, drv_obj);
    STM32DriverParams *stm32_params = (STM32DriverParams *) motor->sfoc_driver->params;
    uint32_t duty0;

    if (motor->beeping)
        return;

    motor->beeping = true;
    motor->flipped = false;
    /* Reprogramming the timers without SimpleFOC knowing is a bit of a hack but still
     * offloading the rest of the timer logic to SimpleFOC and keeping it all within the
     * contraints of an embedded MCU are probably worth it.
     */
    motor->orig_psc0 = stm32_params->timers_handle[0]->Instance->PSC;
    motor->orig_arr0 = stm32_params->timers_handle[0]->Instance->ARR;

    motor->sfoc_driver->enable(); /* Sets all duty cycles to 0 */

    stm32_setClockAndARR(stm32_params->timers_handle[0], freq);

    duty0 = (uint32_t) volume * (stm32_params->timers_handle[0]->Instance->ARR + 1) / 200;
    stm32_setPwm(stm32_params->timers_handle[0], stm32_params->channels[0], duty0);

    motor->beep_end_ts = millis() + duration_ms;
    motor->loop_cb.cb = (void (*)(void *)) motor_drv_stm32_beep_loop;
    motor->loop_cb.data = motor;
    main_loop_cb_add(&motor->loop_cb);
}

static obgc_foc_driver_class motor_drv_pwm_class = {
    .set_phase_voltage = motor_drv_pwm_set_phase_voltage,
    .on                = motor_drv_pwm_on,
    .off               = motor_drv_pwm_off,
    .free              = motor_drv_pwm_free,
    .beep              = motor_drv_stm32_beep,
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
