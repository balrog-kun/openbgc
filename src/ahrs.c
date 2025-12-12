/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h> /* For delay(), micros() */
#include <stdio.h>
#include <math.h>

#include "moremath.h"
#include "util.h"
#include "main.h"

#include "ahrs.h"

/*
 * Put the calculation of reference gravity vector in a static function to make the
 * algorithms themselves independent of the world frame axis convention (ENU, NED, NWU).
 *
 * We hardcode ENU here.  Throughout this file, the state quaternion is assumed to be
 * the "passive" quaternion representing the orientation of the local/body frame within
 * the global/world frame.  In other words it represents the rotation operation that
 * converts the global frame to local frame or, alternatively, the rotation that
 * converts a vector's representation in the local frame to its representation in the
 * global frame.  Quaternion components order is w, x, y, z.  Important to state both
 * these conventions as there are many possibel conventions in use out there.
 *
 * The world frame axes convention affects mainly this function, quaternion_{to,from}_euler,
 * the axis_top/axis_right parameters, ahrs_init_q_with_a.
 */
static void get_ref_gravity(const float *q, float *g) {
    /* vector_rotate_by_quaternion([0, 0, 1], quaternion_conj(ahrs->q)) */
    g[0] = 2.0f * (q[3] * q[1] - q[0] * q[2]);
    g[1] = 2.0f * (q[3] * q[2] + q[0] * q[1]);
    g[2] = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
}

#define DBG_PRINT(fmt, ...) { \
        if (ahrs->debug_print && !(ahrs->debug_cnt & 255)) { \
            char buf[200]; \
            sprintf(buf, fmt, __VA_ARGS__); \
            ahrs->debug_print(buf); \
        } \
    }

static void madgwick_update(obgc_ahrs *ahrs, float *gyr, float *acc, float dt) {
    float q1 = ahrs->q[0], q2 = ahrs->q[1], q3 = ahrs->q[2], q4 = ahrs->q[3];
    float beta, norm;
    float s[3];
    float gradient1, gradient2, gradient3, gradient4;
    float q_dot1, q_dot2, q_dot3, q_dot4;

    /* Normalize accelerometer measurement */
    norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (norm == 0.0f) {
        /* Weightless?  Keep going with gyro data only */
        beta = 0.0f;
    } else {
        norm = 1.0f / norm;
        beta = ahrs->beta;
    }
    DBG_PRINT("magdwick: recip norm %.2f", norm);

    /* Objective function for gravity: reference_acc - actual_acc in local frame */
    get_ref_gravity(ahrs->q, s);
    s[0] -= acc[0] * norm;
    s[1] -= acc[1] * norm;
    s[2] -= acc[2] * norm;

    /* Gradient for gravity Jacobian */
    gradient1 = -2.0f * q3 * s[0] + 2.0f * q2 * s[1];
    gradient2 =  2.0f * q4 * s[0] + 2.0f * q1 * s[1] - 4 * q2 * s[2];
    gradient3 = -2.0f * q1 * s[0] + 2.0f * q4 * s[1] - 4 * q3 * s[2];
    gradient4 =  2.0f * q2 * s[0] + 2.0f * q3 * s[1];

    /* TODO: once we have mahony working, try to get this one working too, including with the encoder inputs.
     * see around https://arxiv.org/pdf/1711.02508 section 6.1 about how the jacobians are calculated?
     * actually, easier info on reference vector vs. sensor reading, the gradients and its use is in
     * https://ahrs.readthedocs.io/en/latest/filters/madgwick.html */
    norm = beta / sqrtf(gradient1 * gradient1 + gradient2 * gradient2 + gradient3 * gradient3 + gradient4 * gradient4);
    DBG_PRINT("magdwick: gradient recip norm %.2f", norm);

    /* Apply feedback */
    q_dot1 = -0.5f * (q2 * gyr[0] + q3 * gyr[1] + q4 * gyr[2]) - norm * gradient1;
    q_dot2 =  0.5f * (q1 * gyr[0] + q3 * gyr[2] - q4 * gyr[1]) - norm * gradient2;
    q_dot3 =  0.5f * (q1 * gyr[1] - q2 * gyr[2] + q4 * gyr[0]) - norm * gradient3;
    q_dot4 =  0.5f * (q1 * gyr[2] + q2 * gyr[1] - q3 * gyr[0]) - norm * gradient4;

    /* Integrate quaternion */
    q1 += q_dot1 * dt;
    q2 += q_dot2 * dt;
    q3 += q_dot3 * dt;
    q4 += q_dot4 * dt;

    /* Normalize quaternion */
    norm = 1.0f / sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    ahrs->q[0] = q1 * norm;
    ahrs->q[1] = q2 * norm;
    ahrs->q[2] = q3 * norm;
    ahrs->q[3] = q4 * norm;

    /* Only every now and then, print norm for debug */
    DBG_PRINT("magdwick: q recip norm %.2f dt %f", norm, dt);
}

static void mahony_update(obgc_ahrs *ahrs, float *gyr, float *acc, float dt) {
    float omega[3];
    float error[3] = {}, *error_scaled = omega;
    float mbeta;
    float norm;

    /*
     * At very low angular velocities, as gyro readings become dominated by noise reduce the gyro
     * weight and use an increasingly slow low-pass filter.  As soon as angular velocity increases
     * crank the weight back up.
     *
     * The hope is that this will better track very slow movements like in time lapse or astrophotograpy
     * use cases, where the tracking speeds can remain constant for long periods and it's important
     * to capture the speed correctly more than it is to point at exactly the request coordinates.
     * (Ok, both things are important.)
     *
     * This tracking relies on the encoder readings (even if low resolution) to start correcting
     * the LPF-filtered angular speed once we're close to an angle that marks the step from one
     * encoder value to the next one, separately in each axis.  At that point we'll hopefully zero
     * in on the exact angular velocity over time.  This can also be zero velocity where we just
     * want to be very steady rather than micro-drift and wander from one side to the other of the
     * setpoint angle.
     */
#   define BETA_DROP_RATE 0.85f
#   define BETA_MIN       0.1f
    norm = vector_norm(gyr);
    if (norm > ahrs->gyro_stddev * 4)
        ahrs->beta = 1.0f;
    else if (norm > ahrs->gyro_stddev * 2) {
        if (ahrs->beta < BETA_DROP_RATE)
            ahrs->beta *= 1 / BETA_DROP_RATE;
    } else if (ahrs->beta > BETA_MIN / BETA_DROP_RATE)
        ahrs->beta *= BETA_DROP_RATE;
    else
        ahrs->beta = BETA_MIN;

    mbeta = 1.0f - ahrs->beta;
    omega[0] = ahrs->gyro_lpf[0] * mbeta + gyr[0] * ahrs->beta;
    omega[1] = ahrs->gyro_lpf[1] * mbeta + gyr[1] * ahrs->beta;
    omega[2] = ahrs->gyro_lpf[2] * mbeta + gyr[2] * ahrs->beta;

    memcpy(ahrs->gyro_lpf, omega, 3 * sizeof(float));

#define APPLY_GYRO_FIRST
#ifdef APPLY_GYRO_FIRST
    /*
     * Apply the new weighted gyro data before calculating errors from other sources.
     * This is a little wasteful but hopefully worth it.
     */
    vector_mult_scalar(omega, dt / 2);
    quaternion_mult_add_xyz(ahrs->q, omega);
    quaternion_normalize(ahrs->q);

    ahrs->gyr_contrib += vector_norm(omega);
    memset(error_scaled, 0, 3 * sizeof(float));
#else
    ahrs->gyr_contrib += vector_norm(omega) * dt; /* omega components are multiplied later */
#endif

    /* See if we have any useful values from other sensors to contrast the gyro based angles and limit drift */

    norm = vector_norm(acc);
    if (norm > 0.1f && norm < 1.9f) { /* acc is already scaled to 1g unit */
        float g_local[3];
        float acc_error[3], sin_angle;

        get_ref_gravity(ahrs->q, g_local);
        vector_cross(acc, g_local, acc_error); /* Rotation vector from reference to estimated */
        vector_mult_scalar(acc_error, 1.0f / norm);
        sin_angle = vector_norm(acc_error);
        vector_add(error, acc_error);
        vector_mult_scalar(acc_error, ahrs->acc_kp);
        vector_add(error_scaled, acc_error);

        ahrs->acc_contrib += sin_angle * ahrs->acc_kp * dt;
        ahrs->acc_error_avg += sin_angle;
        if (sin_angle > ahrs->acc_error_max)
            ahrs->acc_error_max = sin_angle;
    }

    if (ahrs->encoder_q) {
        float conj_q[4] = INIT_CONJ_Q(ahrs->q);
        float enc_error_q[4], enc_error[3];

        /* With the acceleration data we had local vectors so we needed the rotation
         * from reference vector towards estimated.  Here we have global inputs so we
         * need the opposite: estimated (q) to reference (encoder_q) orientation, but
         * with output in body-local reference frame again:
         *   q x enc_error_q = encoder_q
         * Left-multiply by conj(q):
         *   enc_error_q = conj(q) x encoder_q
         */
        quaternion_mult_to(conj_q, steal_ptr(ahrs->encoder_q), enc_error_q);
        /* _to_rotvec() + vector_norm() in one step.  Still does atan2f().  We can afford
         * that but could also have used the small-angle approximation in the spirit of
         * the Mahony/Magdwick filters.
         */
        quaternion_to_axis_angle(enc_error_q, enc_error, &norm);

        /* Don't correct if within deadband */
        /* TODO: probably also need to take into account encoder stddev due to noise.  On
         * the Pilotfly H2 the stddev is luckily way less than resolution */
#       define MIN_ENC_ERROR (1.5f * ahrs->encoder_step)
        if (norm >= MIN_ENC_ERROR) {
            /* Note the amount of correction here scales with the error angle with a
             * discontinuity at +/-Pi, but for the accelerometer it scales with
             * sin(angle), no discontinuity.
             */
            vector_mult_scalar(enc_error, norm - MIN_ENC_ERROR);
            vector_add(error, enc_error);
            vector_mult_scalar(enc_error, ahrs->enc_kp);
            vector_add(error_scaled, enc_error);

            ahrs->enc_contrib += (norm - MIN_ENC_ERROR) * ahrs->enc_kp * dt;
            ahrs->enc_error_avg += norm;
            if (norm > ahrs->enc_error_max)
                ahrs->enc_error_max = norm;
        }
    }

    /* Finally apply the error and rotate again */
    vector_mult_scalar(error_scaled, dt / 2);
    quaternion_mult_add_xyz(ahrs->q, error_scaled);
    quaternion_normalize(ahrs->q);

    /* TODO: calc the error integral too? if so we need to remove omega from error_scaled (or use "error"?) */
    /*
     * if so, apply the spin_rate_limit as in https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/imu.c
     *   but does the lpf kindof cover the same needs as the integral?
     *
     *   also can we autocalibrate not only the offset but scale variations??
     *   we'd have to be sure we know the sign. if sign is different, we skip this cycle?
     *   in any case, we shouldn't be calculating it only based on the single latest reading but on the lpf of gyro and lpf or error data
     */

    /* TODO: do we want to consider the error magnitude in updating beta, too? */
    /* TODO: do we want to update ahrs->gyro_bias using the error * ahrs->ki? */
}

static void velocity_update(obgc_ahrs *ahrs, float *gyr, float dt) {
    /* Should we try to calculate new_q * prev_q^-1 / dt instead? */
    memcpy(ahrs->velocity_vec, gyr, 3 * sizeof(float));
    vector_rotate_by_quaternion(ahrs->velocity_vec, ahrs->q); /* New and old ahrs->q are probably equally good/bad here */
}

static void remap_axes(obgc_ahrs *ahrs, float *vec) {
    float input[3] = { vec[0], vec[1], vec[2] };

    vec[0] = ahrs->axis_sign[0] == 1 ? input[ahrs->axis_map[0]] : -input[ahrs->axis_map[0]];
    vec[1] = ahrs->axis_sign[1] == 1 ? input[ahrs->axis_map[1]] : -input[ahrs->axis_map[1]];
    vec[2] = ahrs->axis_sign[2] == 1 ? input[ahrs->axis_map[2]] : -input[ahrs->axis_map[2]];
}

static void compute_y_axis(uint8_t x_map, uint8_t *y_map, uint8_t z_map,
        int8_t x_sign, int8_t *y_sign, int8_t z_sign) {
    if ((z_map + 3 - x_map) % 3 == 2) {
        *y_map = (x_map + 1) % 3;
        *y_sign = x_sign * z_sign;
    } else if ((z_map + 3 - x_map) % 3 == 1) {
        *y_map = (x_map + 2) % 3;
        *y_sign = -x_sign * z_sign;
    } else {
        /* Bad configuration */
    }
}

obgc_ahrs *ahrs_new(obgc_imu *imu, sbgc_imu_axis axis_top, sbgc_imu_axis axis_right) {
    obgc_ahrs *ahrs = malloc(sizeof(obgc_ahrs));

    memset(ahrs, 0, sizeof(obgc_ahrs));
    ahrs->imu = imu;

    /* Convert axis_right to X mapping */
    ahrs->axis_map[0] = abs(axis_right) - 1;
    ahrs->axis_sign[0] = axis_right < 0 ? -1 : 1;

    /* Convert axis_top to Z mapping */
    ahrs->axis_map[2] = abs(axis_top) - 1;
    ahrs->axis_sign[2] = axis_top < 0 ? -1 : 1;

    /* Compute Y mapping */
    compute_y_axis(ahrs->axis_map[0], &ahrs->axis_map[1], ahrs->axis_map[2],
            ahrs->axis_sign[0], &ahrs->axis_sign[1], ahrs->axis_sign[2]);

    /* Initialize quaternion */
    ahrs->q[0] = 1.0f;
    ahrs->q[1] = 0.0f;
    ahrs->q[2] = 0.0f;
    ahrs->q[3] = 0.0f;

    ahrs->last_update = micros();

    return ahrs;
}

void ahrs_free(obgc_ahrs *ahrs) {
    free(ahrs);
}

/* TODO: add parameter to keep yaw from before or from encoder readings */
static void ahrs_init_q_with_a(obgc_ahrs *ahrs, const float *a) {
    float lensq = vector_normsq(a);

    /* Initialize quaternion */
    if (!ahrs->encoder_q && lensq > 0.01f) {
        /*
         * At rest and with the body frame aligned with the ENU reference frame (no rotation), the
         * acceleration as measured by the accelerometer should be in the Z axis (up).  Given
         * only the accelerometer reading, no yaw information (assume 0) and assuming body is still
         * at rest, get the pitch (Y axis rotation) and roll (X axis rotation) angle of the body
         * that would give this accelerometer reading.  Initialize the AHRS quaternion from these
         * angles.  TODO: recheck if this works if roll is over 90 deg (pitch can't be >90 deg by
         * definition and this is guaranteed here by the always positive atan2 x parameter).  The
         * gimbal physically may have arbitrary limits on either axis but these two angles are
         * only used to initialize the quaternion, they don't have to map to rotation angles on
         * the physical joints.
         */
        float angles[3] = {
            0.0f, /* yaw */
            atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2])), /* pitch */
            atan2f(a[1], a[2]), /* roll */
        };

        quaternion_from_euler(angles, ahrs->q);
    } else if (ahrs->encoder_q) {
        memcpy(ahrs->q, ahrs->encoder_q, sizeof(ahrs->q));
        ahrs->encoder_q = NULL;
    } else {
        ahrs->q[0] = 1.0f;
        ahrs->q[1] = 0.0f;
        ahrs->q[2] = 0.0f;
        ahrs->q[3] = 0.0f;
    }
}

void ahrs_calibrate(obgc_ahrs *ahrs) {
#define SAMPLES_NUM 512
    int32_t acc_raw[3], gyr_raw[SAMPLES_NUM][3];
    int64_t sum[3] = { 0, 0, 0 };
    float dev[3] = { 0.0f, 0.0f, 0.0f };
    float acc[3];
    unsigned int i;
    float factor = M_PI / (180.0f * ahrs->imu->cls->gyro_scale);

    for (i = 0; i < SAMPLES_NUM; i++) {
        /* Read sensor data */
        ahrs->imu->cls->read_main(ahrs->imu, acc_raw, gyr_raw[i]);

        /* Sum integers for accuracy */
        sum[0] += gyr_raw[i][0];
        sum[1] += gyr_raw[i][1];
        sum[2] += gyr_raw[i][2];
        delay(10); /* Use main_loop_sleep();? */
    }

    ahrs->gyro_bias[0] = (float) sum[0] * (factor / SAMPLES_NUM);
    ahrs->gyro_bias[1] = (float) sum[1] * (factor / SAMPLES_NUM);
    ahrs->gyro_bias[2] = (float) sum[2] * (factor / SAMPLES_NUM);

    for (i = 0; i < SAMPLES_NUM; i++) {
        float diff[3] = {
            gyr_raw[i][0] * factor - ahrs->gyro_bias[0],
            gyr_raw[i][1] * factor - ahrs->gyro_bias[1],
            gyr_raw[i][2] * factor - ahrs->gyro_bias[2],
        };

        /* Summing floats won't give us the best accuracy but we're good with approx values here */
        dev[0] += diff[0] * diff[0];
        dev[1] += diff[1] * diff[1];
        dev[2] += diff[2] * diff[2];
    }

    ahrs->gyro_stddev = sqrtf((dev[0] + dev[1] + dev[2]) / (SAMPLES_NUM * 3));
    /*
     * TODO: with Magdwick we can get the beta directly from this, instead of from user configuration?
     *
     * "beta = sqrt{frac{3}{4}}bar{omega}_beta
     *
     * where bar{omega}_beta is the estimated mean zero gyroscope measurement error of each axis."
     */

    acc[0] = (float) acc_raw[0] / ahrs->imu->cls->accel_scale;
    acc[1] = (float) acc_raw[1] / ahrs->imu->cls->accel_scale;
    acc[2] = (float) acc_raw[2] / ahrs->imu->cls->accel_scale;
    remap_axes(ahrs, acc);
    ahrs_init_q_with_a(ahrs, acc);

    ahrs->gyro_lpf[0] = 0.0f;
    ahrs->gyro_lpf[1] = 0.0f;
    ahrs->gyro_lpf[2] = 0.0f;
    ahrs->beta = 1.0f;

    ahrs->last_update = micros();

    if (ahrs->debug_print) {
        char output[256];
        float ypr[3];

        quaternion_to_euler(ahrs->q, ypr);
        sprintf(output, "Calib reset to (non-remapped): bias = (%.3f, %.3f, %.3f), stddev = (%.3f, %.3f, %.3f), "
                "ypr = (%.2f, %.2f, %.2f), acc was (%.2f, %.2f, %.2f)",
                ahrs->gyro_bias[0] * R2D, ahrs->gyro_bias[1] * R2D, ahrs->gyro_bias[2] * R2D,
                sqrtf(dev[0] / SAMPLES_NUM) * R2D, sqrtf(dev[1] / SAMPLES_NUM) * R2D, sqrtf(dev[2] / SAMPLES_NUM) * R2D,
                ypr[0] * R2D, ypr[1] * R2D, ypr[2] * R2D, acc[0], acc[1], acc[2]);
        ahrs->debug_print(output);
    }
}

void ahrs_reset_orientation(obgc_ahrs *ahrs) {
    int32_t acc_raw[3], gyr_raw[3];
    float acc[3], gyr[3];
    float factor = M_PI / (180.0f * ahrs->imu->cls->gyro_scale);

    /* Read sensor data */
    ahrs->imu->cls->read_main(ahrs->imu, acc_raw, gyr_raw);

    /* Convert to physical units */
    acc[0] = (float) acc_raw[0] / ahrs->imu->cls->accel_scale;
    acc[1] = (float) acc_raw[1] / ahrs->imu->cls->accel_scale;
    acc[2] = (float) acc_raw[2] / ahrs->imu->cls->accel_scale;
    gyr[0] = gyr_raw[0] * factor - ahrs->gyro_bias[0];
    gyr[1] = gyr_raw[1] * factor - ahrs->gyro_bias[1];
    gyr[2] = gyr_raw[2] * factor - ahrs->gyro_bias[2];

    /* Remap axes */
    remap_axes(ahrs, acc);
    remap_axes(ahrs, gyr);

    ahrs_init_q_with_a(ahrs, acc);

    velocity_update(ahrs, gyr, 1.0f);

    memcpy(ahrs->gyro_lpf, gyr, 3 * sizeof(float));
    ahrs->beta = 1.0f;

    ahrs->last_update = micros();

    if (ahrs->debug_print) {
        char output[256];
        float ypr[3];

        quaternion_to_euler(ahrs->q, ypr);
        sprintf(output, "Reset ypr = (%.2f, %.2f, %.2f), rates were (%.2f, %.2f, %.2f), acc (%.2f, %.2f, %.2f)",
                ypr[0] * R2D, ypr[1] * R2D, ypr[2] * R2D, gyr[0] * R2D, gyr[1] * R2D, gyr[2] * R2D,
                acc[0], acc[1], acc[2]);
        ahrs->debug_print(output);
    }
}

void ahrs_update(obgc_ahrs *ahrs) {
    int32_t acc_raw[3], gyr_raw[3];
    float acc[3], gyr[3];
    float dt;
    float factor = M_PI / (180.0f * ahrs->imu->cls->gyro_scale);
    uint32_t now = micros();

    /* Calculate time delta */
    dt = (now - ahrs->last_update) * 1e-6f;
    ahrs->last_update = now;

    /* TODO: wait for movement to stop when forcing reset based on accelerometer? maybe move to main loop */
    if (dt > 0.1f) {
        ahrs_reset_orientation(ahrs);
        return;
    }

    /* Read sensor data */
    ahrs->imu->cls->read_main(ahrs->imu, acc_raw, gyr_raw);
    PERF_SAVE_TS;

    if (ahrs->debug_print) {
        if (abs(gyr_raw[0]) > 0x7f00)
            ahrs->debug_print("X OF");
        if (abs(gyr_raw[1]) > 0x7f00)
            ahrs->debug_print("Y OF");
        if (abs(gyr_raw[2]) > 0x7f00)
            ahrs->debug_print("Z OF");
    }

    /* Convert to physical units */
    acc[0] = (float) acc_raw[0] / ahrs->imu->cls->accel_scale;
    acc[1] = (float) acc_raw[1] / ahrs->imu->cls->accel_scale;
    acc[2] = (float) acc_raw[2] / ahrs->imu->cls->accel_scale;
    gyr[0] = gyr_raw[0] * factor - ahrs->gyro_bias[0];
    gyr[1] = gyr_raw[1] * factor - ahrs->gyro_bias[1];
    gyr[2] = gyr_raw[2] * factor - ahrs->gyro_bias[2];

    /* Remap axes */
    remap_axes(ahrs, acc);
    remap_axes(ahrs, gyr);

    velocity_update(ahrs, gyr, dt);

    /*
     * TODO: online-recalibrate gyro_bias? basic version would be just a very slow LPF on gyro
     * readings alone.  But better accumulate how much we're correcting with accelerometers again with
     * slow LPF.  Even better instead/also accumulate how much we're correcting with encoder inputs
     * once we have them and only when we actually do correct with encoders (they're lower-res so we
     * only use them when we have two imus and the difference between imu-calculated angles and encoders
     * is out of the allowed deadband), but in those cases we can give the encoders higher trust so
     * faster LPF?.  Ignore encoders when close to gimbal lock etc. and note the axes may not aligh
     * very well so we have to be clever which gyro_bias value we're correcting.
     * When using the accelerometer corrections amount, better act on the decomposed values, see
     * below about the decomposition.
     * When using the encoders and two IMU's, decide which IMU is to blame for the error, how?
     */
    /* TODO: 1. during times of movement, correlate accelerations between the two IMUs,
     * calculate likely acceleration reading component that's due to motion (i.e. remove expected gravity),
     * then translate this to the ref frame of the other IMU and ignore that part of accelerometer reading.
     * Maybe calc the motion component as a weighted avg from the two IMU's non-gravity acceleration?
     *
     * in times of no movement, trust the accelerometer more.. basically the lower the gyro reading the more
     * we can trust the accelerometer?? hmm not sure
     *
     * calculate the expected centrifugal force on main IMU due to rotation, this will give us approx position of the
     * sensor relative to COG, use it to remove the centrifugal force from accel reading once we have some
     * level of trust in the calculate values.  Only on the main IMU?? yeah
     * we can apply a similar mechanism for turning motion of the whole gimbal, e.g. when operator's whole body is
     * turning.  But, instead of calibrating the centrifugal force over time we use the momentary acceleration
     * on the frame IMU like 1., should we correlate it with gyro rates? nah, not necessary
     */
    /* Think about how we can integrate the encoder readings... only apply them to the filter when the angles from
     * encoders differ by at least E (based on encoder resolution) from the angles calculated from the two
     * quaternions, and not in gimbal lock?  In all axes? yeah probably,
     * and do it gradually with the square of that error??
     */
    /* And this, IMPORTANT for getting good resolution: once we have some trust in the how much action we get from
     * the force we apply to our motors, use what we apply to our motors as an estimate of how much we turned!!
     * This may be a good way to get good accuracy in tripod mode.
     */
    /*
     * Decomposition:
     *   decompose gyro readings on main IMU into:
     *     * gyro readings on frame IMU (whole gimbal rotation), mapped since axes are rotated,
     *     * previous angular momentum+forces we applied on the motors, mapped since axes are rotated, maybe even gimbal-locked,
     *     * other slip in the motors maybe due to inertia and smooth rolling axes, again mapped,
     *     * error.
     *
     *   decompose accelerometer readings on both IMUs into:
     *     * G,
     *     * what we expect is due to change in G due to rotation (based on gyro reading over last period? half of it?)
     *     * what we expect is centrifugal force from just camera rotation, based on the above gyro decomposition,
     *       and based on online centrifugal calibration? i.e. IMU distance from COG of the camera+the end of the arm,
     *       ideally also apply the same for the two other parts of the arm?
     *     * what we expect is centrifugal force from the whole gimbal rotation, now apply on both AHRS instances and we can
     *       use the gyro readings from the frame IMU (mapped) like in the gyro decomposition above,
     *     * what we expect is acceleration from the change in whole gimbal rotation (delta-g{x,y,z} * arm), now apply on
     *        both AHRS instances and we can use the gyro readings from the frame IMU (mapped)??,
     *     * what we expect is due to whole frame acceleration, common to both IMUs and now we can try to correlate
     *       between the two IMUs,
     *     * error.
     */

    /* Update filter */
    PERF_SAVE_TS;
    //madgwick_update(ahrs, gyr, acc, dt);
    mahony_update(ahrs, gyr, acc, dt);
    PERF_SAVE_TS;

    if (ahrs->debug_print && !(ahrs->debug_cnt++ & 1023)) {
        char output[256];
        float ypr[3];

        quaternion_to_euler(ahrs->q, ypr);
        sprintf(output, "New ypr = (%.2f, %.2f, %.2f), rates were (%.3f, %.3f, %.3f), acc (%.2f, %.2f, %.2f)"
                " beta %.2f gyr_contrib %.2f acc_contrib %.2f errmax %.1f enc_contrib %.2f errmax %.1f dt %ius"
                " gyr %.2f lpf %.2f acc_ypr (*, %.2f, %.2f)",
                ypr[0] * R2D, ypr[1] * R2D, ypr[2] * R2D, gyr[0] * R2D, gyr[1] * R2D, gyr[2] * R2D,
                acc[0], acc[1], acc[2], ahrs->beta, ahrs->gyr_contrib * R2D,
                ahrs->acc_contrib * R2D, asinf(ahrs->acc_error_max) * R2D,
                ahrs->enc_contrib * R2D, ahrs->enc_error_max * R2D,
                (int) (dt * 1e6f), vector_norm(gyr), vector_norm(ahrs->gyro_lpf),
                atan2f(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2])) * R2D, atan2f(acc[1], acc[2]) * R2D);
        ahrs->acc_contrib = ahrs->acc_error_avg = ahrs->acc_error_max = 0.0f;
        ahrs->enc_contrib = ahrs->enc_error_avg = ahrs->enc_error_max = 0.0f;
        ahrs->gyr_contrib = 0.0f;
        ahrs->debug_print(output);
    }
}

void ahrs_set_encoder_q(obgc_ahrs *ahrs, const float *encoder_q) {
    ahrs->encoder_q = encoder_q;
}

void ahrs_set_weights(obgc_ahrs *ahrs, float beta, float acc_kp, float enc_kp, float encoder_step) {
    ahrs->beta = beta;
    ahrs->acc_kp = acc_kp;
    ahrs->enc_kp = enc_kp;
    ahrs->encoder_step = encoder_step;
}

void ahrs_set_debug(obgc_ahrs *ahrs, void (*fn)(const char *)) {
    ahrs->debug_print = fn;
}
