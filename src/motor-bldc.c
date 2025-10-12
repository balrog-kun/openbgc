/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h> /* For micros(), delayMicroseconds() */

#include "main.h"
#include "moremath.h"
#include "util.h"

#include "motor-bldc.h"

struct motor_bldc_s {
    sbgc_motor obj;
    sbgc_motor *driver;
    sbgc_encoder *enc;
    struct main_loop_cb_s loop_cb;
    bool on;
    struct sbgc_bldc_with_encoder_calib_data_s calib_data;
    float electric_scale;
    float target_omega;
    float v_max;
    float kp, ki, kd; /* Set ki == 0 if running an outer (angle) PID on top of this to avoid redundancy? */
    float i, i_falloff;
    float prev_theta;
    float prev_omega;
    uint32_t prev_ts;
    float kdrag/*B*/, kcoulomb, kstiction;
};

/*
 * Assumed motor maths:
 *   theta (angle) += omega * dt
 *   omega (angular velocity) += a * dt
 *   a (angular acceleration) = (torque_motor - friction_torques) / J (moment of inertia) - frame_angular_acceleration?
 *   torque_motor = i_q * Kt (motor constant? does it account for cogging though?)
 *   i_q = (v_q - ...) / R
 *
 *   specifically: v = R * i + L * ... + e
 *   specifically: torque_motor = J * a + B * omega + torque_friction
 *   J = moment of inertia aka rotational inertia, is different for each axis + may change to some extent because of the rotation in other axes of the camera mass
 *       and inner arm masses
 *   B = viscous friction (drag?) coefficient = torque_viscous / omega
 *   torque_friction = (coulomb aka dry friction torque when |omega| > 0 else stiction torque) * sign(omega)    (both torques being negative)
 *   stiction is supposed to be > coulomb friction (in absolute terms) but not by a lot.
 *
 * omega is our input, v_q our output.
 *
 * We get desired a from omega, but as soon as we try to get torque, we hit unknown factors like J, Kt, R, so we use a user-configurable PID that estimates v_q from a
 * We could probably calculate frame_a from the AHRS+axes though.
 */

static void motor_bldc_update_theta(struct motor_bldc_s *motor) {
    motor->prev_theta = (float) motor->enc->cls->read(motor->enc) / motor->enc->cls->scale; /* TODO: only read once in main loop? and convert the division to mult */
    motor->prev_ts = micros();
}

static int motor_bldc_init(struct motor_bldc_s *motor) {
    int ret;

    if (motor->obj.ready)
        return 0;

    return motor->obj.cls->recalibrate(&motor->obj);
}

static void motor_bldc_set(struct motor_bldc_s *motor, float vel) {
    motor->target_omega = vel;
}

static int motor_bldc_on(struct motor_bldc_s *motor) {
    int ret;

    if (motor->on)
        return 0;

    if ((ret = motor->driver->cls->on(motor->driver)) != 0)
        return ret;

    motor->on = true;
    motor_bldc_update_theta(motor);
    motor->prev_omega = 0;
    motor->i = 0;
    return 0;
}

static void motor_bldc_off(struct motor_bldc_s *motor) {
    motor->driver->cls->off(motor->driver);
    motor->on = false;
}

static void motor_bldc_free(struct motor_bldc_s *motor) {
    if (motor->on)
        motor_bldc_off(motor);
    main_loop_cb_remove(&motor->loop_cb);
    free(motor);
}

static int motor_bldc_recalibrate(struct motor_bldc_s *motor) {
    int ret, i;
    float mech0, mech1, diff, pairs;
    int8_t dir;

    if (motor->on) /* Must be off */
        return -1;

    motor->calib_data.pole_pairs = -1;
    motor->calib_data.zero_electric_offset = -1;
    motor->calib_data.sensor_direction = 0;

    if ((ret = motor->driver->cls->on(motor->driver)) != 0)
        return ret;

    /*
     * Similar to what SimpleFOC does but with pole_pairs autodetect.
     *
     * First send stator voltage vectors in a sequence sweeping from 0 to 360 degrees
     * so that whatever the initial rotor angle is, we'll attract it at some point of
     * the sequence and rotate it to a known electrical angle at the end of the sequence.
     * Then save the encoder (mechanical) angle matching that electrical angle.
     *
     * We'll be driving the motor by setting v_d == 1 which is not efficient but we
     * have no better method without calibration data.  The whole sequence is about 4.5
     * seconds, hopefully we're not risking overheating in this amount of time.
     */
#define CALIB_START_OFFSET 270 /* SimpleFOC starts at 270deg, seems to work just as poorly as 0 */
    for (i = 1; i <= 360; i += 1) {
        motor->driver->cls->set_phase_voltage(motor->driver, 0.0f, motor->v_max, i + CALIB_START_OFFSET);
        delayMicroseconds(2000000 / 360);
    }

    delay(1000); /* Wait for the rotor to settle */

    motor_bldc_update_theta(motor);
    mech1 = motor->prev_theta;

    /* Rotate one full electrical rotation the other direction */
    for (i = 1; i <= 360; i += 1) {
        motor->driver->cls->set_phase_voltage(motor->driver, 0.0f, motor->v_max, 360 - i + CALIB_START_OFFSET);
        delayMicroseconds(2000000 / 360);
    }

    delay(1000); /* Wait for the rotor to settle */

    motor_bldc_update_theta(motor);
    mech0 = motor->prev_theta;
    motor->driver->cls->off(motor->driver);

    diff = mech1 - mech0;
    if (diff >= 180.0f)
        diff -= 360.0f;
    else if (diff <= -180.f)
        diff += 360.0f;

    if (diff > 0) /* Assuming pole pairs > 1 */
        dir = 1;
    else {
        dir = -1;
        diff = -diff;
    }

    if (diff < 0.5f)
        return -2; /* Too little motion seen by encoder (assuming pole pairs < ~720) */

    pairs = 360.0f / diff;
    if (fabsf(pairs - roundf(pairs)) > 0.1f)
        return -3; /* Electrical angle not close enough to a multiple of mechanical angle distance */

    motor->calib_data.pole_pairs = roundf(pairs);

    mech1 -= CALIB_START_OFFSET / motor->calib_data.pole_pairs * dir;
    mech1 += (360.0f / motor->calib_data.pole_pairs - diff) * 0.5f * dir; /* Try to improve accuracy */

    /* 0 mechanical angle expressed as electrical angle */
    motor->calib_data.zero_electric_offset = (dir == 1 ? 360.0f : 0.0f) - fmodf(mech1 * motor->calib_data.pole_pairs, 360.0f) * dir;
    motor->calib_data.sensor_direction = dir;
    motor->electric_scale = (float) motor->calib_data.pole_pairs * dir;
    motor->obj.ready = true;
    return 0;
}

static int motor_bldc_get_calibration(struct motor_bldc_s *motor, struct sbgc_motor_calib_data_s *out_data) {
    if (!motor->obj.ready)
        return -1;

    memcpy(&out_data->bldc_with_encoder, &motor->calib_data, sizeof(motor->calib_data));
    return 0;
}

sbgc_motor_class motor_bldc_class = {
    .set_velocity    = (void (*)(sbgc_motor *, float)) motor_bldc_set,
    .powered_init    = (int (*)(sbgc_motor *)) motor_bldc_init,
    .on              = (int (*)(sbgc_motor *)) motor_bldc_on,
    .off             = (void (*)(sbgc_motor *)) motor_bldc_off,
    .free            = (void (*)(sbgc_motor *)) motor_bldc_free,
    .recalibrate     = (int (*)(sbgc_motor *)) motor_bldc_recalibrate,
    .get_calibration = (int (*)(sbgc_motor *, sbgc_motor_calib_data *)) motor_bldc_get_calibration,
};

static void motor_bldc_loop(struct motor_bldc_s *motor) {
    float prev_theta, theta, dtheta, invdt, omega, accel, error, torque, vq, vd;
    uint32_t prev_ts;

    if (!motor->obj.ready || !motor->on)
        return;

    /* Angles mechanical until further notice */
    prev_theta = motor->prev_theta;
    prev_ts = motor->prev_ts;

    motor_bldc_update_theta(motor);
    dtheta = motor->prev_theta - prev_theta;
    if (dtheta > 180.0f)
        dtheta -= 360.0f;
    else if (dtheta <= -180.f)
        dtheta += 360.0f;

    invdt = 1000000.0f / (motor->prev_ts - prev_ts); /* TODO: zero check */
    omega = dtheta * invdt;
    accel = (omega - motor->prev_omega) * invdt;
    motor->prev_omega = omega;

    error = motor->target_omega - (omega + accel * motor->kd);
    /* Basic PID (note we applied Kd before Kp so Kd is strictly in time units) */
    torque = error * motor->kp + motor->i * motor->ki;
    motor->i = motor->i * (1.0f - motor->i_falloff) + error; /* TODO: (1 - falloff) ^ dt? error * dt? */
    /* Friction torque */
    torque += omega * motor->kdrag; /* TODO: maybe should factor this into error */
    if (omega > 0.01f)
        torque += motor->kcoulomb;
    else if (omega < -0.01f)
        torque -= motor->kcoulomb;
    else if (error > 0.01f)
        torque += motor->kstiction;
    else if (error < 0.01f)
        torque -= motor->kstiction;

    /* TODO: get frame acceleration from AHRS and add some amount of torque to counter that.
     * Actually, since the frame acceleration will have already acted on the motor, probably just
     * subtract it from the omega so that it's not factored into the error or the integral.
     * Note this is a little moot because at non-zero middle angle, the effect of frame
     * acceleration will be split between the two outer motors by some ratio and we'll never
     * get this fully right.
     */

    /* TODO: once/if we ever have all of this fully right (optimally with auto-calibration),
     * we want to estimate the actual acceleration expected within dt from now, so we can feed
     * that back into the AHRS.
     */

    /* Convert to electric angle */
    theta = motor->prev_theta * motor->electric_scale + motor->calib_data.zero_electric_offset;

    /* Calculate output voltages.
     *
     * Since we expect low angular velocities in our use cases, don't bother with non-zero
     * direct voltages for now.
     */
    vq = constrain(torque / 12.0f, -motor->v_max, motor->v_max); /* TODO: divide vq by current VBAT and multiply by winding resistance */
    vd = 0.0f;

    /* TODO: take a temperature sensor as input to init() or if NULL, estimate temperature from vq, VBAT, resistance, dissipation rate.
     * Reduce v_max proportionally to temperature if over threshold.  */

    motor->driver->cls->set_phase_voltage(motor->driver, vq * motor->calib_data.sensor_direction, vd, theta);
}

sbgc_motor *sbgc_motor_bldc_new(sbgc_encoder *enc, sbgc_motor *driver,
        const struct sbgc_motor_calib_data_s *calib_data) {
    struct motor_bldc_s *motor = (struct motor_bldc_s *) malloc(sizeof(struct motor_bldc_s));

    memset(motor, 0, sizeof(*motor));
    motor->obj.cls = &motor_bldc_class;

    motor->enc = enc;
    motor->driver = driver;

    if (calib_data) {
        memcpy(&motor->calib_data, &calib_data->bldc_with_encoder, sizeof(motor->calib_data));
        motor->electric_scale = (float) motor->calib_data.pole_pairs * motor->calib_data.sensor_direction;
        motor->obj.ready = true;
    }

    motor->loop_cb.cb = (void (*)(void *)) motor_bldc_loop;
    motor->loop_cb.data = motor;
    main_loop_cb_add(&motor->loop_cb);

    return &motor->obj;

error:
    motor_bldc_class.free(&motor->obj);
    return NULL;
}

void sbgc_motor_bldc_set_param(sbgc_motor *motor, sbgc_motor_bldc_param param,
        float val) {
    struct motor_bldc_s *bldc = (struct motor_bldc_s *) motor;

    switch (param) {
    case BLDC_PARAM_KP:
        bldc->kp = val;
        break;
    case BLDC_PARAM_KI:
        bldc->ki = val;
        break;
    case BLDC_PARAM_KD:
        bldc->kd = val;
        break;
    case BLDC_PARAM_KI_FALLOFF:
        bldc->i_falloff = val;
        break;
    case BLDC_PARAM_K_DRAG:
        bldc->kdrag = val;
        break;
    case BLDC_PARAM_K_COULOMB:
        bldc->kcoulomb = val;
        break;
    case BLDC_PARAM_K_STICTION:
        bldc->kstiction = val;
        break;
    case BLDC_PARAM_V_MAX:
        bldc->v_max = val;
        break;
    }
}
