/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h> /* For max(), stdint, stdbool, string etc. */
#include <math.h>

#include "ahrs.h"
#include "moremath.h"
#include "encoder.h"
#include "motor.h"
#include "motor-bldc.h"
#include "main.h"
#include "axes.h"

#include "control.h"

static void control_calc_target(struct control_data_s *control,
        float *out_target_q, float *out_target_ypr) {
    float frame_rel_q[4], frame_ypr[3], target_rel_q[4];
    int i;

    /*
     * We go through Euler/Tait-Bryan angles here to handle the three axes simultaneously.  This cannot be
     * very numerically stable.. Ideally we'd only rely on quaternion maths but it gets too complicated if we
     * want to support pitch and roll follow.  (does it?)
     *
     * However we could easily operate on the sines and cosines cutting out the atanf()s in quaternion_to_euler()
     * and the sinf/cosf()s in quaternion_from_euler() (TODO).
     *
     * For only yaw follow, we'd do ref = frame_q x forward_vec x conj(home_frame_q), flatten it, calculate
     * rotation from ref to forward_vec to use the resulting quaternion to rotate home_q.  If ref was too close
     * to vertical, we'd repeat the same calc with [-forward_vec[1], forward_vec[0], 0] instead and seamlessly
     * switch between one of these two methods.
     */
    quaternion_mult_to(control->frame_q, control->conj_aligned_home_frame_q, frame_rel_q);
    quaternion_to_euler(frame_rel_q, frame_ypr);

    static int cnt;////
    for (i = 0; i < 3; i++) {
        bool follow = control->settings->follow[i];
        float rc;

        if (control->sbgc_api_override_mode[i] != SBGC_API_OVERRIDE_NONE) {
            if ((uint32_t) (now - control->sbgc_api_override_ts) > 5000000) /* TODO: is the timeout configurable? */
                control->sbgc_api_override_mode[i] = SBGC_API_OVERRIDE_NONE;
        }

        if (control->sbgc_api_override_mode[i] != SBGC_API_OVERRIDE_NONE &&
                control->sbgc_api_follow_override[i])
            follow = false;
            /* TODO: any change to a speed mode (whether RC or SBGC serial API) should perhaps
             * preserve the previous offset so we may need to add or subtract frame_ypr[i].
             */

        switch (control->sbgc_api_override_mode[i]) {
        case SBGC_API_OVERRIDE_NONE:
        case SBGC_API_OVERRIDE_RC:
            if (control->sbgc_api_override_mode[i] == SBGC_API_OVERRIDE_NONE)
                rc = control->rc_ypr_readings[i];
            else
                rc = control->sbgc_api_ypr_offsets[i];

            rc *= control->settings->rc_gain * 0.01f * D2R;

            if (control->settings->rc_mode_angle)
                control->target_ypr_offsets[i] = rc;
            else
                control->target_ypr_offsets[i] += rc * control->dt;

            break;
        case SBGC_API_OVERRIDE_SPEED:
            control->target_ypr_offsets[i] += control->sbgc_api_ypr_speeds[i] * control->dt;
            break;
        case SBGC_API_OVERRIDE_ANGLE:
            control->target_ypr_offsets[i] = control->sbgc_api_ypr_offsets[i];
            break;
        }

        out_target_ypr[i] = (follow ? frame_ypr[i] : (i ? 0 : control->forward_az)) +
            control->target_ypr_offsets[i];
    }

    /* TODO: handle various corner cases, there may be cases where the yaw from quaternion_to_euler is meaningless,
     * detect those and just keep main_ypr in those cases, maybe add hysteresis.
     * TODO: some sbgc_api_override_mode values need to override keep_yaw.
     */
    if (control->settings->keep_yaw) {
        float conj_home_q[4] = INIT_CONJ_Q(control->aligned_home_q);
        float main_rel_q[4], main_ypr[3], diff;
        quaternion_mult_to(control->main_ahrs->q, conj_home_q, main_rel_q);
        quaternion_to_euler(main_rel_q, main_ypr);
        diff = angle_normalize_pi(out_target_ypr[0] - main_ypr[0]);
        if (fabsf(diff) > M_PI / 2)
            out_target_ypr[0] += M_PI;
    }

    quaternion_from_euler(out_target_ypr, target_rel_q);
    /* TODO: If following all, shortcut to multiply by frame_rel_q, perhaps use the non-aligned_* versions.
     * If following none, don't multiply at all, use .home_q.  */
    quaternion_mult_to(target_rel_q, control->aligned_home_q, out_target_q);

    /* TODO: do we want to further complicate this by adding 1-motor and/or 2-motor modes?
     * That would probably mean an euler angles priority setting that tells us, with two motors, which two axes we want to stabilize
     * and which one we give up on.  We may want this kind of setting anyway for non-orthogonal axes where some combinations of
     * Yaw+Pitch+Roll may be out of reach, and specially with travel limits on some joints.  */
}

static float control_apply_velocity_limits(struct control_data_s *control,
        const float *delta_axis, float delta_angle) {
    float new_v, max_v, current_v, v_vec[3], perp_vec[3];

    /*
     * We now have the whole angle we want to travel and the current camera angular velocity in
     * control->main_ahrs->velocity_vec.  Apply simple acceleration and velocity limits to the
     * magnitudes of these vectors, not to the 3D vectors because that would be too complicated to
     * later map to the per-joint velocities.  But at least calculate the current_v scalar not
     * as the length of velocity vec (vector_norm(control->main_ahrs->velocity_vec)) but as the
     * length of its projection onto delta_axis so that it's negative if we're currently
     * travelling in the opposite direction.
     *
     * TODO: force control->settings->ahrs_velocity_kp of 1 every first iteration after control enable.
     */
    vector_weighted_sum(control->main_ahrs->velocity_vec, -control->settings->ahrs_velocity_kp,
            control->velocity_vec, 1.0f - control->settings->ahrs_velocity_kp, v_vec);
    current_v = vector_dot(v_vec, delta_axis);

    if (!isfinite(current_v)) {
        error_print("Bad state, resetting velocity_vec");
        current_v = 0.0f;
        memset(control->velocity_vec, 0, 3 * sizeof(float));
        memset(v_vec, 0, 3 * sizeof(float));
    }

    /*
     * Based on current velocity and max allowed acceleration/deceleration decide if we're
     * close enough to target that we need to be decelerating already so that we're at standstill
     * when we're there.
     *
     * start_velocity = current_v;
     * end_velocity = 0;
     * average_velocity = (start_velocity + end_velocity) / 2;
     * deceleration_time = abs(start_velocity - end_velocity) / max_acceleration;
     * deceleration_distance = average_velocity * decelration_time;
     *
     * deceleration_distance = current_v^2 / 2 / max_acceleration;
     *
     * We could compare that against delta_angle but instead we translate it to:
     * max_velocity = sqrt(2 * whole_distance * max_acceleration);
     * ...if we're close or above that, we need to be decelrating already, if we're below we can
     * keep accelerating up to this value and the user configured max velocity, whichever is lower.
     *
     * Obviously this is flawed because the new velocity takes some time to take effect and
     * another period for it to feed back to us through the sensors, plus we set the velocities
     * in steps.  So shorten the distance by some time * current velocity.
     */
#define BUFFER_STEPS 3
    max_v = sqrtf(2 * max(delta_angle - BUFFER_STEPS * control->dt * current_v, 0) *
            control->settings->max_accel);

    /* TODO: filter current_v and use the filter response time in place of dt */
    if (delta_angle <= BUFFER_STEPS * control->settings->max_accel * control->dt * control->dt)
        new_v = delta_angle / (BUFFER_STEPS * control->dt);
    else if (current_v >= max_v)
        /*
         * Two options here:
         *   * prioritize control->settings->max_accel, set
         *     current_v - control->settings->max_accel * control->dt.
         *     The downside is we may overshoot if for whatever reason we're already
         *     travelling too fast.  The reason could be a new user command or a sudden
         *     movement.
         *   * prioritize target, set max_v, the downside could be a deceleration above the
         *     allowed limit.
         * So try to prioritize target but spread the deceleration evenly over the travel
         * distance left.
         *
         * From the earlier calc we also get:
         *   deceleration_required = current_v^2 / 2 / decelration_distance;
         */
        new_v = current_v - (0.5f * current_v * current_v / max(delta_angle, M_PI / 20)) * control->dt;
    else
        new_v = min(control->settings->max_vel, current_v + control->settings->max_accel * control->dt);

    vector_weighted_sum(v_vec, 1, delta_axis, -current_v, perp_vec);
    vector_weighted_sum(delta_axis, new_v, perp_vec,
            max((vector_norm(perp_vec) - control->settings->max_accel * control->dt), 0),
            control->velocity_vec);

    return new_v;
}

static void control_calc_path_step_orientation(struct control_data_s *control, const float *delta_axis,
        float delta_angle, float *out_joint_deltas) {
    float conj_frame_q[4] = INIT_CONJ_Q(control->frame_q);
    float step_delta_vec[3];

    /* Set actual delta to the maximum we're allowed in unit time without exceeding max_accel or max_vel */
    control_apply_velocity_limits(control, delta_axis, delta_angle);

    /* Recalculate delta_q based on what we want to see one step from now (actually skip going back to quaternion) */
    memcpy(step_delta_vec, control->velocity_vec, 3 * sizeof(float));
    vector_mult_scalar(step_delta_vec, control->dt);

    /* Convert the global step_delta_vec to frame_q-local then to per-joint delta angles.
     * Do the same with current velocities from the IMU gyros while we're there.
     *
     * TODO: axes_precalc_rel_q() and axes_calibrate() start from the frame IMU frame of
     * reference and go through the joints from outer to inner.  Switch to using the main IMU as
     * reference and work our way through the joints from there to reduce the accumulation of
     * rounding errors in encoder angles and maybe even simplify the maths.  Note this will
     * probably make the maths even less obvious, is it worth it?
     */
    vector_rotate_by_quaternion(step_delta_vec, conj_frame_q);
    memcpy(out_joint_deltas, step_delta_vec, 3 * sizeof(float));
    vector_mult_matrix(out_joint_deltas, control->axes->jacobian_pinv);
}

static void control_calc_path_step_joint_target(struct control_data_s *control,
        const float *angles_target, const float *angles_current, bool force_direction,
        float *out_joint_deltas) {
    float angles_diff[3];
    float delta_axis[3], new_v;
    int i;

    if (force_direction) {
        for (i = 0; i < 3; i++)
            angles_diff[i] = angles_target[i] - angles_current[i];
    } else {
        /* Default to whichever direction to target angle is shorter */
        for (i = 0; i < 3; i++)
            angles_diff[i] = angle_normalize_pi(angles_target[i] - angles_current[i]);

        /* But if that crosses a hard stop / limiter, may have to take the other way */
        axes_apply_limits_full(control->axes, control->settings->limit_margin,
                angles_current, angles_diff);
    }

    memcpy(delta_axis, angles_diff, 3 * sizeof(float));
    vector_mult_matrix_t(delta_axis, control->axes->jacobian_t); /* Multiply by J^T^T == J */
    control->delta_angle = vector_norm(delta_axis);              /* Override the value from control_step */
    vector_mult_scalar(delta_axis, 1.0f / control->delta_angle); /* Normalize to get just the axis */
    vector_rotate_by_quaternion(delta_axis, control->frame_q);   /* Go back to global frame */

    /* Get the maximum velocity magnitude we're allowed in dt time without exceeding max_accel or max_vel */
    new_v = control_apply_velocity_limits(control, delta_axis, control->delta_angle);

    memcpy(out_joint_deltas, angles_diff, 3 * sizeof(float));
    vector_mult_scalar(out_joint_deltas, control->dt * new_v / control->delta_angle);
}

static void control_calc_path_step_joint(struct control_data_s *control, const float *target_q,
        const float *angles_current, float *out_joint_deltas) {
    float conj_frame_q[4] = INIT_CONJ_Q(control->frame_q);
    float target_rel_q[4];
    float angles_target[3];

    quaternion_mult_to(conj_frame_q, target_q, target_rel_q);
    axes_q_to_angles_universal(control->axes, target_rel_q, control->settings->home_angles,
            angles_target);

    control_calc_path_step_joint_target(control, angles_target, angles_current, false,
            out_joint_deltas);
}

static void control_calc_path_step_euler(struct control_data_s *control, const float *target_ypr,
        float *out_joint_deltas) {
    float conj_aligned_home_q[4] = INIT_CONJ_Q(control->aligned_home_q);
    float conj_main_q[4] = INIT_CONJ_Q(control->main_ahrs->q);
    float current_rel_q[4], current_ypr[3], ypr_dist, factor;
    float step_rel_q[4], step_q[4], step_ypr[3], delta_q[4];
    float delta_axis[3], delta_angle;
    int i;

    quaternion_mult_to(control->main_ahrs->q, conj_aligned_home_q, current_rel_q);
    quaternion_to_euler(current_rel_q, current_ypr);

    /* Use the rotation between current and 0.1deg away from current orientation to calculate
     * the axis, but use the actual Euler angle difference as rotation angle, for velocity control.
     */
    ypr_dist = vector_dist(current_ypr, target_ypr);
    factor = 0.1f * D2R / ypr_dist; /* step = current + (target - current) * factor */
    vector_weighted_sum(current_ypr, 1.0f - factor, target_ypr, factor, step_ypr);
    quaternion_from_euler(step_ypr, step_rel_q);
    quaternion_mult_to(step_rel_q, control->aligned_home_q, step_q);
    quaternion_mult_to(step_q, conj_main_q, delta_q);
    quaternion_to_axis_angle(delta_q, delta_axis, &delta_angle);

    control_calc_path_step_orientation(control, delta_axis, ypr_dist, out_joint_deltas);
}

static void control_calc_path_step_park(struct control_data_s *control,
        const float *angles_current, float *out_joint_deltas) {
    const float *angles_target = control->settings->have_parking ?
        control->settings->park_angles : control->settings->home_angles;

    control_calc_path_step_joint_target(control, angles_target, angles_current, false,
            out_joint_deltas);
    /* TODO: possibly call back to power off when delta_angle below threshold */
}

/* Could also implement this in motor-bldc.c but here may be a little easier */
static void control_calc_path_step_limit_search(struct control_data_s *control,
        const float *angles_current, float *out_joint_deltas) {
#define MIN_DIFF (0.1f * D2R) /* Should use encoder resolution+stddev instead */
#define MIN_TIME_US 1000000
    float diff1, diff2, diff3;
    int i;
    __auto_type search = &control->limit_search;
    uint8_t num = search->axis;
    float angles_target[3];

    for (i = 0; i < 3; i++)
        out_joint_deltas[i] = 0.0f;

    switch (search->step) {
    case 0: /* Start */
        search->start_angle = angles_current[num];
        search->last_angle = angles_current[num];
        search->last_angle_ts = now;
        search->step++;
        break;
    case 1: /* limit_min search */
        out_joint_deltas[num] = control->settings->limit_search_v;
        diff1 = angle_normalize_pi(angles_current[num] - search->last_angle);
        diff2 = angle_normalize_pi(angles_current[num] - search->start_angle);
        diff3 = angle_normalize_pi(search->last_angle - search->start_angle);

        if (diff1 > MIN_DIFF) {
            if (diff2 >= 0 && diff3 < 0) {
                error_print("No limit on this axis");
                main_beep();
                control->axes->has_limits[num] = false;
                /* We should be close to start_angle so nothing else to do here */
                search->step = 5; /* Next axis */
                break;
            }

            search->last_angle = angles_current[num];
            search->last_angle_ts = now;
        } else if (now - search->last_angle_ts > MIN_TIME_US) {
            /* Found the limit? */
            main_beep();
            control->axes->limit_min[num] = search->last_angle;
            search->step++;
            break;
        }

        break;
    case 2: /* Move back to start_angle at normal velocity */
    case 4:
        if (angle_normalize_pi(angles_current[num] - search->start_angle) < MIN_DIFF) {
            search->step++;
            break;
        }

        for (i = 0; i < 3; i++)
            angles_target[i] = angles_current[i];
        angles_target[num] = search->start_angle;

        if (search->step == 2) {
            /* Since we got here at positive velocity, the target delta must be negative so we
             * go back the same way.
             */
            if (angles_target[num] > angles_current[num])
                angles_target[num] -= 2 * M_PI;
        } else {
            /* Since we got here at negative velocity, the target delta must be positive so we
             * go back the same way.
             */
            if (angles_target[num] < angles_current[num])
                angles_target[num] += 2 * M_PI;
        }

        control_calc_path_step_joint_target(control, angles_target, angles_current, true,
                out_joint_deltas);
        break;
    case 3: /* limit_max search */
        out_joint_deltas[num] = -control->settings->limit_search_v;
        diff1 = angle_normalize_pi(angles_current[num] - search->last_angle);
        diff2 = angle_normalize_pi(angles_current[num] - control->axes->limit_min[num]);
        diff3 = angle_normalize_pi(search->last_angle - control->axes->limit_min[num]);

        if (diff1 < -MIN_DIFF) {
            if (diff2 <= 0 && diff3 > 0) {
                /* Something is wrong, don't set has_limits, exit */
                error_print("Problem, found limit_min but not limit_max");
                main_beep();
                search->step = 5;
                search->axis = 3;
                break;
            }

            search->last_angle = angles_current[num];
            search->last_angle_ts = now;
        } else if (now - search->last_angle_ts > MIN_TIME_US) {
            /* Found the limit? */
            main_beep();
            control->axes->limit_max[num] = search->last_angle;
            control->axes->has_limits[num] = true;
            search->step++;
            break;
        }

        break;
    case 5: /* Move on to the next axis */
        search->step = 0;
        search->axis++;

        if (search->axis >= 3) {
            search->axis = 0;
            control->path_type = control->settings->default_path;
        }
    }
}

void control_step(struct control_data_s *control) {
    float conj_main_q[4] = INIT_CONJ_Q(control->main_ahrs->q);
    float target_q[4], delta_q[4];
    float delta_axis[3];
    float joint_velocities_target[3];
    float joint_angles_current[3], joint_step_deltas[3], joint_extra_torque[3] = {};
    float target_ypr[3];
    int i, j;

    for (i = 0; i < 3; i++) {
        int num = control->axes->axis_to_encoder[i];

        joint_angles_current[i] = control->encoders[num]->reading_rad * control->axes->encoder_scale[num];
    }

    /* Where do we want to go */
    control_calc_target(control, target_q, target_ypr);
    quaternion_mult_to(target_q, conj_main_q, delta_q); /* Global delta to target_q */
    quaternion_to_axis_angle(delta_q, delta_axis, &control->delta_angle);

    /* How we want to get there: least joint movement if > 5deg, least camera movement if <= 5deg (cheaper) */
    switch (control->path_type) {
    path_short:
    case CONTROL_PATH_SHORT:
        control_calc_path_step_orientation(control, delta_axis, control->delta_angle,
                joint_step_deltas);

        axes_apply_limits_step(control->axes, control->settings->limit_margin,
                joint_angles_current, joint_step_deltas);
        break;

    case CONTROL_PATH_INTERPOLATE_EULER:
        if (control->delta_angle <= 5 * D2R)
            goto path_short;

        control_calc_path_step_euler(control, target_ypr, joint_step_deltas);

        axes_apply_limits_step(control->axes, control->settings->limit_margin,
                joint_angles_current, joint_step_deltas);
        break;

    default:
    case CONTROL_PATH_INTERPOLATE_JOINT:
        if (control->delta_angle <= 5 * D2R)
            goto path_short;

        control_calc_path_step_joint(control, target_q, joint_angles_current, joint_step_deltas);
        break;

    case CONTROL_PATH_PARK:
        control_calc_path_step_park(control, joint_angles_current, joint_step_deltas);
        break;

    case CONTROL_PATH_LIMIT_SEARCH:
        control_calc_path_step_limit_search(control, joint_angles_current, joint_step_deltas);
        break;
    }

    /* Divide the deltas by dt to get velocities, go from rad/dt to °/unit time */
    for (i = 0; i < 3; i++)
        joint_velocities_target[i] = joint_step_deltas[i] * (R2D / control->dt);

    /* Process the coupling between pairs of axes for at least two reasons:
     * 1. to tell the motor PIDs to anticipate an external force => acceleration.  When two
     *    axes are aligned and we command one of the motors to exert a torque on its axis, the
     *    inertia of the payload and the structure will exert unexpected torque in the opposite
     *    direction on the other axis.  So the PID of the other motor needs to know to apply
     *    the same torque in the same direction as the first motor just to counteract this
     *    external (to it) reaction force, i.e. without expecting its extra torque to produce
     *    actual acceleration, *only* counteract the reaction torque.  We're effectively
     *    spending ~2x the energy we would without aligned axes, to produce the same amount
     *    of acceleration.
     * 2. TODO: to anticipate Coriolis effect.  We have rotating bodies mounted on top
     *    of other rotating bodies (rotating frame of reference) and there's bound to be an
     *    effect.
     */
    for (i = 0; i < 3; i++)
        for (j = i + 1; j < 3; j++) {
            float delta_v_i = joint_velocities_target[i] - joint_velocities_current[i];
            float delta_v_j = joint_velocities_target[j] - joint_velocities_current[j];
            float alignment = vector_dot(control->axes->jacobian_t[i], control->axes->jacobian_t[j]);

            joint_extra_torque[i] += alignment * delta_v_j;
            joint_extra_torque[j] += alignment * delta_v_i;

            /* Instead of calculating the delta_v's, we might add motor API method to pass
             * the reference of another coupled motor together with the alignment value.  Let
             * the motors' feedback loops talk to each other and figure it out before sending
             * new values to their drivers.  This would be better than the naive thing we do
             * here because it would include the friction modelling (including cogging in the
             * future) reducing the effect of the torques, and the integral+derivative terms
             * we don't know about here, which do change the outputs.  Although if we trust
             * our PIDs, we should be assuming the commanded delta_v is the delta_v we get,
             * precisely thanks to the friction modelling, the I, the D terms, etc.
             * Still the PID may add a delay if max voltage is limited for example or to
             * smooth things out if available feedback is too noisy.
             */
        }

    /* Request new values from motors directly */
    for (i = 0; i < 3; i++) {
        int num = control->axes->axis_to_encoder[i];
        struct obgc_motor_s *motor = control->motors[num];
        /* TODO: any downside if we make the scales always positive and invert the axes in axes_calibrate? */

        /* Note we could merge the three calls into one "torque" value but that would ask for
         * the whole PID logic to be moved here and we don't want that, let it do its job.
         */
        motor->cls->set_velocity(motor, joint_velocities_target[i] / control->axes->encoder_scale[num]);

        if (motor->cls->set_external_torque)
            motor->cls->set_external_torque(motor,
                    joint_extra_torque[i] / control->axes->encoder_scale[num]);
    }
}

void control_update_joint_vel(struct control_data_s *control) {
    float conj_frame_q[4] = INIT_CONJ_Q(control->frame_q);
    int i;

    memcpy(control->joint_velocities, control->main_ahrs->velocity_vec, 3 * sizeof(float));
    vector_rotate_by_quaternion(control->joint_velocities, conj_frame_q);
    vector_mult_matrix(control->joint_velocities, control->axes->jacobian_pinv);
    vector_mult_scalar(control->joint_velocities, R2D);

    for (i = 0; i < 3; i++) {
        int num = control->axes->axis_to_encoder[i];
        struct obgc_motor_s *motor = control->motors[num];

        if (motor->cls->override_cur_velocity)
            motor->cls->override_cur_velocity(motor,
                    control->joint_velocities[i] / control->axes->encoder_scale[num]);
    }
}
