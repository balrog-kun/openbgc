/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h> /* For millis() */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "ahrs.h"
#include "moremath.h"
#include "encoder.h"
#include "main.h"

#include "axes.h"

static void get_enc(struct axes_calibrate_data_s *data, float *enc) {
    int i;

    for (i = 0; i < 3; i++)
        if (data->encoders[i])
            enc[i] = data->encoders[i]->reading_rad;
}

static void get_q(struct axes_calibrate_data_s *data, float *q, int naxis) {
    int i;

    /*
     * Get the main IMU orientation wrt. to the frame IMU, i.e. the quaternion describing a rotation from the frame IMU to the main IMU.
     * (if no frame IMU then fall back to world frame)
     */
    if (data->frame_ahrs) {
        float conj_frame_q[4] = INIT_CONJ_Q(data->frame_ahrs->q);

        /* Compose the main IMU rotiation with the reverse of the frame IMU rotation */
        quaternion_mult_to(conj_frame_q, data->main_ahrs->q, q);
    } else {
        /* Nothing to rotate */
        memcpy(q, data->main_ahrs->q, 4 * sizeof(float));
    }

    /*
     * Take into account the angles from the encoders that we've already calibrated.  Go from
     * outer to inner axes because normally the rotations are applied in the reverse order to
     * go from frame IMU quaternion to main IMU quaternion.  Since we're undoing the rotations
     * we need to apply the inverse rotations in the normal order (reverse the reverse.)
     */
    for (i = 0; i < naxis; i++) {
        float angle2, sin_angle2;
        int num = data->out->axis_to_encoder[i];
        float q_main[4], q_joint[4];

        if (num < 0 || !data->encoders[num])
            return;

        angle2 = data->encoders[num]->reading_rad * data->out->encoder_scale[num] * 0.5f;
        sin_angle2 = sinf(angle2);

        memcpy(q_main, q, sizeof(q_main));
        q_joint[0] = -cosf(angle2); /* - to directly produce the conjugate */
        memcpy(q_joint + 1, data->out->axes[i], 3 * sizeof(float));
        vector_mult_scalar(q_joint + 1, sin_angle2);
        quaternion_mult_to(q_joint, q_main, q);
        quaternion_normalize(q);
    }
}

/*
 * Ask the user to rotate the gimbal joints in specific ways and calculate the axis vectors for the
 * joints, their corresponding encoder numbers, encoder offsets (positions at encoder 0 angle) and
 * encoder scale factors.
 *
 * After this is done, with the encoders and/or IMUs we can have a quite complete picture of the
 * positions of each element of the gimbal.  The chain of orientations and the rotations that take
 * us from one to the next orientation is this:
 *
 *                         World/global frame (identity quaternion)
 *                     < rotation by q_frame, from data->frame_ahrs->q >
 *                            Frame (gimbal handle) IMU orientation
 *      < rotation by angle[0] (from encoder out->axis_to_encoder[0]) around out->axis[0] >
 *                  Gimbal outer arm orientation, q_frame if angle[0] == 0
 *      < rotation by angle[1] (from encoder out->axis_to_encoder[1]) around out->axis[1] >
 *               Gimbal middle arm orientation, q_frame if angle[0:2] == 0, 0
 *      < rotation by angle[2] (from encoder out->axis_to_encoder[2]) around out->axis[2] >
 *              Gimbal inner arm orientation, q_frame if angle[0:3] == 0, 0, 0
 *                          < rotation by out->main_imu_mount_q >
 *                         Main IMU orientation, data->main_ahrs->q
 *          < rotation by another user-calibrated quaternion, out of scope here >
 *                                Camera home orientation
 *
 * Each rotation within <...> is within the local reference frame of the previous orientation.
 *
 * Note that the IMU orientations represent actual current physical orientations of the IMU chips
 * up to a very small factory error.  The gimbal handle and arm orientations on the other hand
 * have no physical meaning, they're each the orientation of an arbitrary element of the
 * corresponding part of the gimbal.  But we likely don't care about their physical orientations,
 * only relative relations.  Similarly the camera home orientation may not be related to any
 * physical part of the camera, only the relative home orientation that the user is wants to
 * maintain or in relation to which they want to operate.  The camera, the main IMU and the
 * inner arm are assumed to be rigidly connected and their orientations differ by constant
 * amounts.
 *
 * In theory this can be easily changed for more than 3 joints.
 */
int axes_calibrate(struct axes_calibrate_data_s *data) {
    float prev_enc[3];
    float conj_prev_q[4];
    int naxis = 0, nsamples, enc_nsamples[3];
    float enc_corr_accum[3];
    float enc_corr_max[3];
    float enc_scale_accum[3];
    float axis_accum[3];
    int i;

    char msg[200];
    bool reinit = true;
    bool reinit_wait;
    unsigned long reinit_move_ts;

    data->print("1. Rotate the outer joint manually over full range of motion or at\r\n");
    data->print("   least 30 degs each direction, while keeping the other two joints'\r\n");
    if (data->frame_ahrs) {
        data->print("   angles fixed.\r\n");
    } else {
        data->print("   angles fixed.  Without frame IMU, keep the frame/base absolutely\r\n");
        data->print("   static like a on tripod.\r\n");
    }

    while (naxis < 3) {
        float q[4], diff_q[4], halfangle, angle, axis[3], inv_sin_half, enc[3];
        float corr_axis, corr_enc[3];
        int best = -1;
        bool accepted;

        if (reinit) {
            reinit = false;
            reinit_wait = true;
            reinit_move_ts = millis();

            get_q(data, conj_prev_q, naxis);
            conj_prev_q[0] = -conj_prev_q[0];

            /* Wait for the ahrs-detected delta in relative q to stay < 0.5deg for a 2s period */
            data->print("   Waiting for IMUs to report no movement...\r");

            memset(enc_corr_accum, 0, sizeof(enc_corr_accum));
            memset(enc_corr_max, 0, sizeof(enc_corr_max));
            memset(enc_scale_accum, 0, sizeof(enc_scale_accum));
            memset(enc_nsamples, 0, sizeof(enc_nsamples));
            memset(axis_accum, 0, sizeof(axis_accum));
            nsamples = 0;
        }

        main_loop_sleep();
        ahrs_update(data->main_ahrs);

        if (data->frame_ahrs)
            ahrs_update(data->frame_ahrs);

        for (i = 0; i < 3; i++)
            encoder_update(data->encoders[i]);

        /* Get new orientation, calc difference from prev as q x conj(prev_q) */
        get_q(data, q, naxis);
        quaternion_mult_to(q, conj_prev_q, diff_q); /* We could delay the other terms calc but this is not a hot path */

        if (fabsf(diff_q[0]) > 0.999f)
            halfangle = 0.0f;
        else
            halfangle = acosf(diff_q[0]); /* 0 - 1 * M_PI range */
        angle = 2 * halfangle;            /* 0 - 2 * M_PI range */
        if (angle > M_PI)
            angle -= 2 * M_PI; /* Now discontinuous at diff_q[0] == 0 */

        if (reinit_wait) {
            unsigned long now = millis();

            if (fabsf(angle) > 0.5f * M_PI / 180) {
                reinit_move_ts = now;
                memcpy(conj_prev_q, q, sizeof(q));
                conj_prev_q[0] = -conj_prev_q[0];
                continue;
            }

            if (now - reinit_move_ts < 2000)
                continue;

            reinit_wait = false;
            data->print("   Start now                                      \r");

            get_enc(data, prev_enc);
            memcpy(conj_prev_q, q, sizeof(q));
            conj_prev_q[0] = -conj_prev_q[0];
            continue;
        }

        /* Do nothing until angle difference of 10 degrees or more */
        if (fabsf(angle) < 10.0f * M_PI / 180)
            continue;

        get_enc(data, enc);

        memcpy(axis, diff_q + 1, 3 * sizeof(float));
        /*
         * We can normalize axis by dividing by abs(sin(halfangle)), len(x, y, z) or sqrt(1 - w^2).
         * FTR sqrtf() takes about half the time of a sinf() on STMF3 / F4, and 5-6 multiplies time.
         * sqrt(1 - w^2) is the most likely to produce a nan but we don't expect diff_q to have
         * lost normalization by a lot.  w would be close to +/-1.0f if angle is close to 0 which
         * we have confirmed it isn't, or if all of q has drifted towards higher values.
         */
        vector_mult_scalar(axis, 1.0f / sqrtf(1.0f - diff_q[0] * diff_q[0])); /* Divide by abs(sin(angle / 2)) to normalize */

        /*
         * If we already have some samples, align the sign of the axis with previous samples so we
         * can accumulate them, and change angle sign accordingly.  We could align the signs of the
         * angles and accumulate the raw axes per encoder with the longest accumulated axis wins but
         * that complicates some things.
         */
        if (nsamples) {
            float axis_d_pos[] = { axis_accum[0] - axis[0], axis_accum[1] - axis[1], axis_accum[2] - axis[2] };
            float axis_d_neg[] = { axis_accum[0] + axis[0], axis_accum[1] + axis[1], axis_accum[2] + axis[2] };

            if (vector_normsq(axis_d_pos) > vector_normsq(axis_d_neg)) {
                vector_mult_scalar(axis, -1.0f);
                angle = -angle;
            }
        }

        vector_add(axis_accum, axis);
        nsamples++;

        for (i = 0; i < 3; i++) {
            float enc_diff = enc[i] - prev_enc[i];

            if (!data->encoders[i])
                continue;

            /*
             * If the encoder readings wrapped around we have a problem because we try to not
             * assume a specific encoder resolution in this code and try to autocalibrate it
             * from the readings.  That would mean we also don't have the exact min/max values
             * or the period.  But assume the driver has at least a rough approximation of the
             * scale factor and that a delta of > M_PI means we wrapped around.  Skip this
             * sample.  TODO: if we have some samples already, use current enc_scale_accum to
             * decide if this new delta has the wrong sign and is roughly off by 2 * M_PI
             * scaled by enc_scale_accum.
             */
            if (fabsf(enc_diff) > M_PI)
                continue;

            /*
             * The idea here is that enc_corr_max[i] is the upper bound for
             * enc_corr_accum[i] * enc_scale_accum[i].  If the correlation is perfect they're
             * equal and lower values mean worse correlation, so we can get a percentage value.
             */
            enc_corr_accum[i] += angle * enc_diff;
            if (fabsf(enc_diff) > 0.01f)
                enc_scale_accum[i] += angle / enc_diff;
            enc_corr_max[i] += angle * angle;
            enc_nsamples[i]++;
        }

        memcpy(conj_prev_q, q, sizeof(q));
        conj_prev_q[0] = -q[0];
        memcpy(prev_enc, enc, sizeof(enc));

        sprintf(msg, "   %i / 64 samples so far\r", nsamples);
        data->print(msg);

        /* Wait for more samples? TODO: check enc_nsamples too */
        if (nsamples < 64)
            continue;

        corr_axis = vector_norm(axis_accum) / nsamples;

        for (i = 0; i < 3; i++) {
            int j;

            if (!data->encoders[i])
                continue;

            /*
             * No fabsf here to find best absolute correlation because we multiply enc_corr_accum by
             * enc_scale_accum which *should* have the same sign.  They can perhaps have different
             * signs but that just means the correaltion is really bad?  So positive values still win.
             */
            enc_scale_accum[i] *= 1.0f / enc_nsamples[i];
            corr_enc[i] = enc_corr_accum[i] * enc_scale_accum[i] / enc_corr_max[i];

            for (j = 0; j < naxis; j++)
                if (data->out->axis_to_encoder[j] == i)
                    break; /* This encoder is already used */

            /*
             * Until we figure out a better way, use the absolute correlation values for the
             * comparison, unscaled.  enc_scale_accum[i] could end up being really big for a
             * completely unrelated axis and boost corr_enc[i] for that axis and make it win.
             * For now assume the three encoders have a similar scale and use enc_corr_accum[i]
             * directly.  We have no max value for enc_corr_accum[i] so we cannot display it
             * as a percentage but we can use it for the comparisons.
             */
            if ((best < 0 || fabsf(enc_corr_accum[i]) > fabsf(enc_corr_accum[best])) && j == naxis)
                best = i;
        }

        sprintf(msg, "   Done. Axis correlation %.1f%%, encoder correlations (%.1f%%, %.1f%%, %.1f%%) * "
                "scale (%.2f, %.2f, %.2f)\r\n", corr_axis * 100.0f,
                corr_enc[0] * 100.0f, corr_enc[1] * 100.0f, corr_enc[2] * 100.0f,
                enc_scale_accum[0], enc_scale_accum[1], enc_scale_accum[2]);
        data->print(msg);

        accepted = corr_axis >= 0.9f && (best < 0 || corr_enc[best] >= 0.8f);
        if (!accepted)
            data->print("   Correlation is too low, let's redo this axis.\r\n");

        for (i = 0; i < 3; i++)
            if (accepted && best >= 0 && i != best && fabsf(enc_corr_accum[i]) >= fabsf(enc_corr_accum[best]) * 0.2f) {
                accepted = false;
                data->print("   Correlation too high with other axes, let's redo this axis.\r\n");
            }

        if (accepted) {
            vector_normalize(axis_accum);
            memcpy(data->out->axes[naxis], axis_accum, sizeof(axis_accum));

            if (best >= 0) {
                data->out->axis_to_encoder[naxis] = best;
                data->out->encoder_scale[best] = enc_scale_accum[best];
            } else
                data->out->axis_to_encoder[naxis] = -1;

            naxis++;

            if (naxis == 1) {
                data->print("2. Rotate the middle joint while keeping the inner joint angle fixed.\r\n");
            } else if (naxis == 2) {
                data->print("3. Finally rotate the inner joint over full range of motion.\r\n");
            }

            if (data->out->axis_to_encoder[0] == -1 || (naxis == 2 && data->out->axis_to_encoder[1] == -1))
                data->print("   Also keep the previously calibrated axes as stable as possible.\r\n");
        }

        reinit = true;
    }

    /* With all 3 encoders, this will give us exactly what we need */
    /* TODO: but without encoders, I guess we'll be using the orientation of 3-rd joint axis wrt. the IMU,
     * not sure how much we can really do without encoders here. */
    get_q(data, data->out->main_imu_mount_q, 3);
    return 0;
}

/* Get the relative rotation between IMUs from encoders and axis data.  Fill in frame_ahrs->q if no frame IMU */
void axes_precalc_rel_q(struct axes_data_s *data, struct obgc_encoder_s **encoders,
        const float *main_q, float *out_rel_q, float *out_frame_q) {
    float q_tmp[4], q0[4], q1[4], q01[4], q2[4], angles[3];
    float damp_factor = 1e-4f;

    /* Get encoder angles */
    for (int i = 0; i < 3; i++) {
        int num = data->axis_to_encoder[i];

        if (encoders[num])
            angles[i] = encoders[num]->reading_rad * data->encoder_scale[num];
        else
            angles[i] = 0.0f;
    }

    quaternion_from_axis_angle(q0, data->axes[0], angles[0]);
    quaternion_from_axis_angle(q1, data->axes[1], angles[1]);
    quaternion_mult_to(q0, q1, q01);
    quaternion_from_axis_angle(q2, data->axes[2], angles[2]);
    quaternion_mult_to(q01, q2, q_tmp);
    quaternion_mult_to(q_tmp, data->main_imu_mount_q, out_rel_q);
    quaternion_normalize(out_rel_q);

    memcpy(q_tmp, out_rel_q, 4 * sizeof(float));
    q_tmp[0] = -q_tmp[0];
    quaternion_mult_to(main_q, q_tmp, out_frame_q);

    memcpy(data->jacobian_t, data->axes, 9 * sizeof(float));
    vector_rotate_by_quaternion(data->jacobian_t[1], q01); /* q0 and q01 should either work */
    vector_rotate_by_quaternion(data->jacobian_t[2], q01); /* and maybe q01 helps the compiler */

    /* Since we'll be using it more than once, pre-calculate the pseudo-inverse
     * rather than solve the equation once in axes_q_to_step_proj().
     *
     * The pseudo-inverse with regularization (non-zero damp factor) is supposed to better
     * handle gimbal lock situations than matrix_inverse() would.  But near gimbal lock each
     * user needs to have a different fallback (speed vs. angle vs. torque), so we may be able
     * to switch to the cheaper matrix_inverse().
     */
    if (!matrix_t_pseudo_invert(data->jacobian_t, damp_factor, data->jacobian_pinv))
        memcpy(data->jacobian_pinv, data->jacobian_t, 9 * sizeof(float));
}

/* Simpler than angle_normalize_pi in moremath.h but may affect resolution more */
static float angle_normalize_0_2pi(float angle) {
    return fmodf(angle + 4 * M_PI, 2 * M_PI);
}

static bool angle_greater(float a, float b) {
    return angle_normalize_0_2pi(a - b) < M_PI;
}

/* Assuming angles normalized to the -720-+720deg range or smaller.
 * We will add 720deg in some places to ensure angle is positive.  This reduces FP resolution
 * but assuming the margin is wide enough that it doesn't matter.
 * 2x limit_margin cannot be wider than the range left after removing limit_min to limit_max.
 */
void axes_apply_limits_simple(const struct axes_data_s *data, float limit_margin,
        const float *angles_current, float *angles_delta) {
    int i;

    for (i = 0; i < 3; i++) {
        float dist_from_min, dist_from_max, zone_width;

        if (!data->has_limits[i])
            continue;

        dist_from_min = angle_normalize_0_2pi(data->limit_min[i] - limit_margin - angles_current[i]);
        dist_from_max = angle_normalize_0_2pi(angles_current[i] - limit_margin - data->limit_max[i]);
        // assert(angle_normalize_0_2pi(data->limit_max[i] - data->limit_min[i]) + 2 * limit_margin < 2 * M_PI);
        zone_width = angle_normalize_0_2pi(data->limit_max[i] - data->limit_min[i] + 2 * limit_margin);

        if (dist_from_min > 2 * M_PI - zone_width) { /* Current angle already within the avoid zone */
            /* Perhaps control has been disabled and just got re-enabled.  Override angles_delta
             * to move the angle over whichever limit we're closer to.
             */
            if (dist_from_min < dist_from_max)
                angles_delta[i] = 2 * M_PI - dist_from_max;
            else /* TODO: respect acceleration/speed limits though */
                angles_delta[i] = dist_from_min - 2 * M_PI;
        } else if (angles_delta[i] > dist_from_min) /* We're about to cross upwards over limit_min */
            angles_delta[i] = dist_from_min;        /* Stop at the limit (minus margin) */
        else if (-angles_delta[i] > dist_from_max)  /* We're about to cross downwards over limit_max */
            angles_delta[i] = -dist_from_max;       /* Stop at the limit (plus margin) */
    }
}

/*
 * Given current orientation and the orientation we want to achieve and current angles at each
 * joint, calculate the best change in the angles to move towards the target orientation.  This
 * makes no assumptions about the axes.
 *
 * It is however based on the local Jacobian and it doesn't generally take the shortest path
 * towards the target orientation.  If called iteratively it'll create a trajectory that marks
 * some sort of arc path from original to target orientations, with one or two bends, not a
 * "straight" line (great circle) such as with SLERP.
 *
 * The damping factor can be used to straighten the trajectory to some degree but it is
 * costly computationally.
 *
 * This returns the estimated 100% angle difference for each joint.  But the assumption is that
 * this is going to be called at small intervals through some time at the end of which we want
 * to be in the target orientation.  The caller is expected to scale the outputs to control the
 * step size, angular speed, acceleration/deceleration, smoothness of the movement.
 *
 * The formula is delta theta = (J^T * J + lambda * I)^-1 * J^T * omega
 * With lambda == 0 it becomes delta theta = J^T * omega
 */
void axes_q_to_step_proj(const struct axes_data_s *data, const float *from_q, const float *to_q,
        float damp_factor, float *out_steps) {
    float omega[3];

    if (from_q) {
        float from_q_inv[4] = INIT_CONJ_Q(from_q);
        float q_rel[4];

        quaternion_mult_to(to_q, from_q_inv, q_rel);     /* 16 multiplications */
        quaternion_to_rotvec(q_rel, omega);              /* ~5 multiplications */
    } else
        quaternion_to_rotvec(to_q, omega);               /* ~5 multiplications */

    axes_rotvec_to_step_proj(data, omega, damp_factor, out_steps);
}

void axes_rotvec_to_step_proj(const struct axes_data_s *data, float *new_omega_vec, /* Note: modifies new_omega_vec */
        float damp_factor, float *out_steps) {
    float jtj[3][3];

    /* Make sure axis[0] and axis[2] are not parallel */
    /* TODO: come up with a good enough fallback if they are, somehow force out_steps[2] to 0 */

    /* Calc J^T * omega */
    vector_mult_matrix(new_omega_vec, data->jacobian_t); /* 9 multiplications */

    /* If no damping, we're done */
    if (damp_factor != 0.0f) {
        memcpy(out_steps, new_omega_vec, 3 * sizeof(float));
        return;
    }

    matrix_jt_mult_j(data->jacobian_t, jtj);             /* 18 multiplications */
    jtj[0][0] += damp_factor;
    jtj[1][1] += damp_factor;
    jtj[2][2] += damp_factor;

    /*
     * Solve JTJ * delta theta = J^T omega instead of inverting JTJ.
     * Inverting could also be an option, it would take 36 multiples + 9 for J^T * omega.
     *
     * TODO: check if jtj fits the conditions for Cholesky Decomposition (or just try it, see if it works),
     * then see if the code ends up being faster
     */
    vector_solve(jtj, new_omega_vec, out_steps);         /* ~51 multiplications */
}

/*
 * Calculate joint angles to achieve a specific main IMU orientation.  This implementation assumes
 * orthogonal axes (at least from outer axis to middle axis and middle to inner, not outer to inner
 * which depends on angle at the middle joint) so this is only here for testing.
 *
 * This basically decomposes to_q into three individual rotations, one around axis[0], one around
 * axis[1] rotated by the previous rotation and one around axis[2] rotated by the two
 * earlier rotations.
 */
void axes_q_to_angles_orthogonal(const struct axes_data_s *data, const float *to_q,
        float *out_angles) {
    float r[3][3], q_axes[4], q_tmp1[4], q_tmp2[4], q_target[4], a2[3];
    float q_conj_mount[4] = INIT_CONJ_Q(data->main_imu_mount_q);
    bool invert;

    /*
     * Note: if this ever gets used in realtime, the following initial steps can be precaculated,
     * saved in axes_data_s.
     */

    /*
     * 1. R = rotation that maps axis[0] to (0, 0, 1) and axis[1] to (0, 1, 0) -- note the
     * orthogonality assumption.
     */
    vector_cross(data->axes[1], data->axes[0], r[0]);
    memcpy(r[1], data->axes[1], 3 * sizeof(float));
    memcpy(r[2], data->axes[0], 3 * sizeof(float));
    quaternion_from_matrix(r, q_axes);

    /* Check if inner axis and r[0] point in opposite directions (>90deg apart), if so invert inner angle */
    /* TODO: do we need to rotate data->axes[2] by outer and middle angles for this check? */
    memcpy(a2, data->axes[2], sizeof(a2));
    vector_mult_matrix(a2, r);
    invert = vector_dot(a2, r[0]) < 0;

    /* End of precalculated values */

    /* 2. Rotate to_q by R and by data->main_imu_mount_q (R @ (to_q @ data->main_imu_mount_q) @ R^T) */
    /* TODO: we have both R and data->main_imu_mount_q at calibration-time, should merge to avoid two multiplications here */
    quaternion_mult_to(to_q, q_conj_mount, q_tmp1);
    quaternion_mult_to(q_axes, q_tmp1, q_tmp2);
    q_axes[0] = -q_axes[0];
    quaternion_mult_to(q_tmp2, q_axes, q_target);

    /* 3. Use standard formula for quaternion to Tait-Bryan angles in ENU */
    quaternion_to_euler(q_target, out_angles);

    /* Double-cover, select the orientation with inner angle in -90 - 90deg range */
    if (fabsf(out_angles[2]) > M_PI * 0.5) {
        out_angles[0] = out_angles[0] - M_PI;
        out_angles[1] = -out_angles[1] - M_PI;
        out_angles[2] = out_angles[2] - M_PI;
    }

    if (invert)
        out_angles[2] = -out_angles[2];

    for (int i = 0; i < 3; i++)
        while (out_angles[i] < 0)
            out_angles[i] += 2 * M_PI;
}

void axes_q_to_angles_universal(const struct axes_data_s *data, const float *to_q, float *out_angles) {
    /* Closed-form solution for 3 arbitrary axes.  Solve for joint angles given target orientation
     * using geometric constraints.
     *
     * We solve the equation: R_target = R(a0, θ0) × R(a1, θ1) × R(a2, θ2)
     * The key insight: The inner joint axis a2 in world coordinates must satisfy:
     *   R(a0, θ0) × R(a1, θ1) × a2 = R_target × a2
     *
     * This is based on the Pieper's theorem about spherical wrists and the solutions given in the
     * original document:
     *   Pieper, D. L. The kinematics of manipulation under computer control. Ph.D. thesis,
     *   Stanford Artificial Intelligence Laboratory - Stanford University, 1968.
     *
     * Note this is currently coded so as to select a solution with inner angle in the -90-90deg
     * range, with outer and middle allowing full range.  This can be changed if needed.
     */
    const float (*axes)[3] = data->axes;
    float v[3] = INIT_VEC(axes[2]);
    float cross[3];
    float a, b, c, e, k, denominator, dot_v_a0, phase, angle;
    float θ[3], θ1_candidates[2]; /* UTF-8 identifiers seem to work Ok */
    float q[4], q_conj_mount[4] = INIT_CONJ_Q(data->main_imu_mount_q);
    int i;

    /* TODO: check data->orthogonal */

    quaternion_mult_to(to_q, q_conj_mount, q);
    vector_rotate_by_quaternion(v, q); /* a₂ in frame coordinates after full rotation */

    /* Step 1: Solve constraint equation: R(a0, θ0) × R(a1, θ1) × a2 = v
     *
     * Let w = R(a1, θ1) × a2 (a2 after middle joint rotation)
     * Then: R(a0, θ0) × w = v
     * For this rotation to exist, the projections of w and v onto the plane perpendicular to a0
     * must have equal lengths:
     *   ||w - (w·a0)a0|| = ||v - (v·a0)a0||
     *
     * Since ||w|| = ||a2|| = 1 and ||v|| = 1, this simplifies to:
     *   (w·a0)² = (v·a0)²   ⇒   w·a0 = ± (v·a0)
     *
     * Now expand w·a0:
     *   w·a₀ = [R(a₁, θ₁) × a₂] · a₀
     *     = [cos(θ₁)a₂ + sin(θ₁)(a₁ × a₂) + (1-cos(θ₁))(a₁·a₂)a₁] · a₀
     *     = cos(θ₁)(a₂·a₀) + sin(θ₁)[(a₁ × a₂)·a₀] + (1-cos(θ₁))(a₁·a₂)(a₁·a₀)
     *     = (a₁·a₂)(a₁·a₀) + cos(θ₁)[(a₂·a₀) - (a₁·a₂)(a₁·a₀)] + sin(θ₁)[(a₁ × a₂)·a₀]
     *
     * Name some coefficients to get: w·a₀ = E + A cos(θ₁) + B sin(θ₁) = ± (v·a₀)
     */
    /* TODO: precalculate these coefficients and denominator */
    c = vector_dot(axes[2], axes[0]);
    e = vector_dot(axes[1], axes[2]) * vector_dot(axes[1], axes[0]);
    a = c - e;
    vector_cross(axes[1], axes[2], cross);
    b = vector_dot(cross, axes[0]); /* Scalar triple product */

    dot_v_a0 = vector_dot(v, axes[0]);

    /* Rearranging: A cos(θ₁) + B sin(θ₁) = ± (v·a₀) - E
     *
     * Solve that for θ₁:
     *
     * This form: A cos(θ) + B sin(θ) = K
     * has solution: θ = atan2(B, A) ± acos(K / √(A² + B²))
     */
    k = dot_v_a0 - e;
    denominator = sqrtf(a * a + b * b);

    /* TODO: check at calibration */
    if (fabsf(denominator) < 0.00001f) {
        /* Degenerate case - axes are aligned */
        memset(out_angles, 0, 3 * sizeof(float));
        return;
    }

    if (fabsf(k / denominator) > 1.0f) {
        /* Target orientation unreachable for these axes */
        memset(out_angles, 0, 3 * sizeof(float));
        return;
    }

    phase = atan2f(b, a);
    angle = acosf(k / denominator);
    θ1_candidates[0] = phase - angle;
    θ1_candidates[1] = phase + angle;

    for (i = 0; i < 2; i++) {
        float a2_after_θ1[3] = INIT_VEC(axes[2]);
        float v_proj[3], a2_proj[3];
        float q0[4], q1[4], q2[4], q01[4];

        θ[1] = θ1_candidates[i];

        /* Step 2: With θ1 known, solve for θ0 using the a₂ direction constraint.
         *
         * We have: R(a₀, θ₀) × w = v, where w = R(a₁, θ₁) × a₂
         * This is a rotation around a₀ that takes w to v.
         *
         * We find θ₀ by projecting w and v onto the plane perpendicular to a₀ and computing
         * the angle between their projections.
         */
        vector_rotate_around_axis(a2_after_θ1, axes[1], θ[1]); /* Not much faster than producing the quaternion */

        vector_weighted_sum(v, 1, axes[0], -vector_dot(v, axes[0]), v_proj);
        vector_weighted_sum(a2_after_θ1, 1, axes[0], -vector_dot(a2_after_θ1, axes[0]), a2_proj);

        if (vector_normsq(v_proj) < 0.000001f || vector_normsq(a2_proj) < 0.000001f)
            θ[0] = 0; /* Arbitrary choice when projections vanish, TODO: replace with current angle */
        else {
            /* No normalization needed for v_proj and a2_proj, the dot and cross product scale together */
            vector_cross(a2_proj, v_proj, cross);
            θ[0] = atan2f(vector_dot(cross, axes[0]), vector_dot(a2_proj, v_proj));
        }

        /* Step 3: With θ0 and θ1 known, solve for θ2 using the full orientation */
        quaternion_from_axis_angle(q0, axes[0], θ[0]);
        quaternion_from_axis_angle(q1, axes[1], θ[1]);
        quaternion_mult_to(q0, q1, q01);
        q01[0] = -q01[0];
        quaternion_mult_to(q01, q, q2);

        /* Extract θ2 from q2.
         * We can get the angle from just q2[0] but due to the double-cover it might be inverted,
         * so we need the full quaternion or at least one other non-zero component to compare
         * signs.
         *
         * Could also project the result of quaternion_to_rotvec() onto axes[2] (we know they're
         * parallel).  The below is a reordered one-liner version that skips the sqrtf but seems,
         * just as good numerically.  copysignf(acosf(q2[0]), vector_dot()) is noticeably worse.
         */
        θ[2] = atan2f(vector_dot(axes[2], q2 + 1), q2[0]) * 2;

        if (fabsf(θ[2]) > M_PI_2)
            θ[2] -= θ[2] > 0 ? M_PI * 2 : -M_PI * 2;

        /* Select one of the two solutions based on theta2 (TODO: accept range center param?) */
        if (fabsf(θ[2]) < M_PI_2 / 2)
            break;
    }

    memcpy(out_angles, θ, 3 * sizeof(float));
}
