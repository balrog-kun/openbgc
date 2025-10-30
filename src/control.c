/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h> /* For millis() */
#include <math.h>

#include "ahrs.h"
#include "moremath.h"
#include "encoder.h"
#include "motor.h"
#include "main.h"
#include "axes.h"

#include "control.h"

void control_step(struct control_data_s *control) {
    float conj_main_q[4] = INIT_CONJ_Q(control->main_ahrs->q);
    float conj_frame_q[4] = INIT_CONJ_Q(control->frame_q);
    float conj_frame_home_q[4] = INIT_CONJ_Q(control->home_frame_q);
    float frame_rel_q[4], frame_rel_align_q[4], frame_ypr[3], target_ypr[3], target_rel_q[4], target_q[4];
    float delta_q[4], delta_angle, delta_axis[3], current_v, tmp_q[4];
    float v_vec[3], max_v, new_v, perp_vec[3], step_delta_vec[3];
    float joint_angles_current[3], joint_angles_to_target[3];
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
    quaternion_mult_to(control->frame_q, conj_frame_home_q, frame_rel_q);
    quaternion_rotate_z_to(frame_rel_q, control->forward_vec[1], control->forward_vec[0], frame_rel_align_q);
    quaternion_to_euler(frame_rel_align_q, frame_ypr);
    frame_ypr[0] -= control->forward_az;

    for (i = 0; i < 3; i++)
        target_ypr[i] = control->follow[i] ? frame_ypr[i] : 0;

    /* TODO: handle various corner cases, there may be cases where the yaw from quaternion_to_euler is meaningless,
     * detect those and just keep main_ypr in those cases, maybe add hysteresis */
    if (control->keep_yaw) {
        float conj_home_q[4] = INIT_CONJ_Q(control->home_q);
        float main_rel_q[4], main_ypr[3], diff;
        quaternion_mult_to(control->main_ahrs->q, conj_home_q, main_rel_q);
        quaternion_to_euler(frame_rel_align_q, main_ypr);
        diff = angle_normalize_pi(target_ypr[0] - main_ypr[0]);
        if (fabsf(diff) > M_PI / 2)
            target_ypr[0] += M_PI;
    }

    quaternion_from_euler(target_ypr, target_rel_q);
    /* TODO: If following all, shortcut to multiply by frame_rel_q.  If following none, don't multiply at all.  */
    quaternion_mult_to(target_rel_q, control->home_q, target_q);

    /* Global delta to target_q */
    quaternion_mult_to(target_q, conj_main_q, delta_q);
    quaternion_to_axis_angle(delta_q, delta_axis, &delta_angle);

    /*
     * We now have the whole angle we want to travel and the current camera angular velocity in
     * control->main_ahrs->velocity_vec.  Apply simple acceleration and velocity limits to the
     * magnitudes of these vectors, not to the 3D vectors because that would be too complicated to
     * later map to the per-joint velocities.  But at least calculate the current_v scalar not
     * as the length of velocity vec (vector_norm(control->main_ahrs->velocity_vec)) but as the
     * length of its projection onto delta_axis so that it's negative if we're currently
     * travelling in the opposite direction.
     *
     * TODO: force control->ahrs_velocity_kp of 1 every first iteration after control enable.
     */
    vector_weighted_sum(control->main_ahrs->velocity_vec, -control->ahrs_velocity_kp,
            control->velocity_vec, 1.0f - control->ahrs_velocity_kp, v_vec);
    current_v = vector_dot(v_vec, delta_axis);

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
    max_v = sqrtf(2 * max(delta_angle - BUFFER_STEPS * control->dt * current_v, 0) * control->max_accel);

    /* TODO: filter current_v and use the filter response time in place of dt */
    if (delta_angle <= BUFFER_STEPS * control->max_accel * control->dt * control->dt)
        new_v = delta_angle / (BUFFER_STEPS * control->dt);
    else if (current_v >= max_v)
        /*
         * Two options here:
         *   * prioritize control->max_accel, set current_v - control->max_accel * control->dt.
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
        new_v = current_v - (0.5f * current_v * current_v / delta_angle) * control->dt;
    else
        new_v = min(control->max_vel, current_v + control->max_accel * control->dt);

    /* Recalculate delta_q based on what we want to see one step from now (actually skip going back to quaternion) */
    vector_weighted_sum(v_vec, 1, delta_axis, -current_v, perp_vec);
    vector_weighted_sum(delta_axis, new_v, perp_vec,
            max((vector_norm(perp_vec) - control->max_accel * control->dt), 0),
            control->velocity_vec);
    memcpy(step_delta_vec, control->velocity_vec, 3 * sizeof(float));
    vector_mult_scalar(step_delta_vec, control->dt);

    /* TODO: add gradual transition to the vertical check.  Switch the ref vector to pitch axis when roll axis is too vertical */
    /* TODO: do we want to further complicate this by adding 1-motor and/or 2-motor modes?
     * That would probably mean an euler angles priority setting that tells us, with two motors, which two axes we want to stabilize
     * and which one we give up on.  We may want this kind of setting anyway for non-orthogonal axes where some combinations of
     * Yaw+Pitch+Roll may be out of reach, and specially with travel limits on some joints.  */

    for (i = 0; i < 3; i++) {
        int num = control->axes->axis_to_encoder[i];

        joint_angles_current[i] = control->encoders[num]->reading_rad * control->axes->encoder_scale[num];
    }

    /* Convert the global step_delta_vec to frame_q-local then to per-joint delta angles */
    vector_rotate_by_quaternion(step_delta_vec, conj_frame_q);
    axes_rotvec_to_step(control->axes, step_delta_vec, joint_angles_current, 0.0f, joint_angles_to_target);

    /* Divide the deltas by dt to get velocities and request these directly from motors.
     *
     * Note we just went from velocities (change /s) to deltas (change per step) but that's ok,
     * our limits are in unit time terms while axes_rotvec_to_step() has to operate on actual angles
     * because for if the target orientation is farther away it might in theory take a completely
     * different trajectory (although it doesn't now).
     */
    for (i = 0; i < 3; i++) {
        int num = control->axes->axis_to_encoder[i];
        struct obgc_motor_s *motor = control->motors[num];
        /* TODO: any downside if we make the scales always positive and invert the axes in axes_calibrate? */
        float delta = joint_angles_to_target[i] / control->axes->encoder_scale[num];

        motor->cls->set_velocity(motor, delta * (R2D / control->dt));
    }
}
