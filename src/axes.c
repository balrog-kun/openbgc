/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h> /* For millis() */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "ahrs.h"
#include "moremath.h"
#include "encoder.h"

#include "axes.h"

static void get_enc(struct calibrate_data_s *data, float *enc) {
    int i;

    for (i = 0; i < 3; i++)
        if (data->encoders[i])
            enc[i] = (float) data->encoders[i]->cls->read(data->encoders[i]) /
                data->encoders[i]->cls->scale * D2R;
}

static void get_q(struct calibrate_data_s *data, float *q, int naxis) {
    int i;

    /*
     * Get the main IMU orientation wrt. to the frame IMU, i.e. the quaternion describing a rotation from the frame IMU to the main IMU.
     * (if no frame IMU then fall back to world frame)
     */
    if (data->frame_ahrs) {
        float conj_frame_q[4] = INIT_CONJ_Q(data->frame_ahrs->q);

        /* Compose the main IMU rotiation with the reverse of the frame IMU rotation */
        quaternion_mult_to(data->main_ahrs->q, conj_frame_q, q);
    } else {
        /* Nothing to rotate */
        memcpy(q, data->main_ahrs->q, 4 * sizeof(float));
    }

    /* Take into account the angles from the encoders that we've already calibrated */
    for (i = 0; i < naxis; i++) {
        float angle2, sin_angle2;
        int num = data->out->axis_to_encoder[i];
        float q_frame[4], q_joint[4];

        if (num < 0 || !data->encoders[num])
            return;

        angle2 = (float) data->encoders[num]->cls->read(data->encoders[num]) /
            data->encoders[num]->cls->scale * D2R * data->out->encoder_scale[num] * 0.5f;
        sin_angle2 = sinf(angle2);

        memcpy(q_frame, q, sizeof(q_frame));
        q_joint[0] = -cosf(angle2); /* - to directly produce the conjugate */
        memcpy(q_joint + 1, data->out->axes[i], 3 * sizeof(float));
        vector_mult_scalar(q_joint + 1, sin_angle2);
        quaternion_mult_to(q_joint, q_frame, q);
    }
}

/* In theory this can be easily changed for more than 3 joints */
int axes_calibrate(struct calibrate_data_s *data) {
    float prev_enc[3];
    float conj_prev_q[4];
    int naxis = 0, nsamples;
    float enc_corr_accum[3];
    float enc_corr_max;
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
            enc_corr_max = 0.0f;
            memset(enc_scale_accum, 0, sizeof(enc_scale_accum));
            memset(axis_accum, 0, sizeof(axis_accum));
            nsamples = 0;
        }

        main_loop_sleep();
        ahrs_update(data->main_ahrs);

        if (data->frame_ahrs)
            ahrs_update(data->frame_ahrs);

        /* Get new orientation, calc difference from prev as conj(prev_q) x q */
        get_q(data, q, naxis);
        quaternion_mult_to(conj_prev_q, q, diff_q); /* We could delay the other terms calc but this is not a hot path */

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
         * can accumulate them, and change angle to accordingly.  We could align the signs of the
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

        for (i = 0; i < 3; i++) {
            float enc_diff = enc[i] - prev_enc[i];

            if (!data->encoders[i])
                continue;

            enc_corr_accum[i] += angle * enc_diff;
            if (fabsf(enc_diff) > 0.01f)
                enc_scale_accum[i] += angle / enc_diff;
        }

        enc_corr_max += angle * angle;
        memcpy(conj_prev_q, q, sizeof(q));
        conj_prev_q[0] = -q[0];
        memcpy(prev_enc, enc, sizeof(enc));
        nsamples++;

        sprintf(msg, "   %i / 32 samples so far\r", nsamples);
        data->print(msg);

        /* Wait for more samples? */
        if (nsamples < 32)
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
            corr_enc[i] = enc_corr_accum[i] * (enc_scale_accum[i] / nsamples) / enc_corr_max;

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
                "(%.2f, %.2f, %.2f)\r\n", corr_axis * 100.0f,
                corr_enc[0] * 100.0f, corr_enc[1] * 100.0f, corr_enc[2] * 100.0f,
                enc_scale_accum[0] / nsamples, enc_scale_accum[1] / nsamples, enc_scale_accum[2] / nsamples);
        data->print(msg);

        accepted = corr_axis >= 0.8f && (best < 0 || corr_enc[best] >= 0.8f);
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
                data->out->encoder_scale[best] = enc_scale_accum[best] / nsamples;
            } else
                data->out->axis_to_encoder[naxis] = -1;

            naxis++;

            if (naxis == 1) {
                data->print("2. Rotate the middle joint while keeping the inner joint angle fixed.\r\n");
            } else {
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

    /* TODO: also calculate the axes in the reverse order: inner wrt. main IMU, middle wrt. inner, outer
     * wrt. middle and frame IMU wrt. outer? */
    return 0;
}

/*
 * Calculate joint angles to achieve a specific main IMU orientation.  This implementation assumes
 * orthogonal axes (at least from outer axis to middle axis and middle to inner, not outer to inner
 * which depends on angle at the middle joint) so this is only here for testing.
 *
 * A full generalised algorithm would require inverse kinematics (iterative.)
 *
 * This basically decomposes to_q into three individual rotations, one around axis[0], one around
 * axis[1] rotated by the previous rotation and one around axis[2] rotated by the two
 * earlier rotations.
 */
void axes_q_to_angles(struct axes_data_s *data, float *to_q, float *out_angles) {
    float r[3][3], q[4], to_q2[4];
    float a2[3], zero2;

    /*
     * Note: if this ever gets used in realtime, the following initial steps can be precaculated,
     * saved in axes_data_s.
     */

    /*
     * 1. R = rotation that maps axis[0] to (0, 0, 1) and axis[1] to (0, 1, 0) -- note the
     * orthogonality assumption.
     */
    r[0][0] = data->axes[0][1] * data->axes[1][2] - data->axes[0][2] * data->axes[1][1];
    r[0][1] = data->axes[0][2] * data->axes[1][0] - data->axes[0][0] * data->axes[1][2];
    r[0][2] = data->axes[0][0] * data->axes[1][1] - data->axes[0][1] * data->axes[1][0];
    r[1][0] = data->axes[1][0];
    r[1][1] = data->axes[1][1];
    r[1][2] = data->axes[1][2];
    r[2][0] = data->axes[0][0];
    r[2][1] = data->axes[0][1];
    r[2][2] = data->axes[0][2];
    quaternion_from_matrix(r, q);

    /* 2. Rotate axis[2] by R: a2 = axis[2] @ R */
    memcpy(a2, data->axes[2], sizeof(a2));
    vector_mult_matrix(a2, r);

    /* 3. Calculate the 0 or neutral angle for axis[2]: zero2 = atan2(len(cross(a2, (1, 0, 0))), dot(a2, (1, 0, 0))) */
    zero2 = atan2f(sqrtf(a2[1] * a2[1] + a2[2] * a2[2]), a2[0]);

    /* 3.1. Set sign for the angle from (1, 0, 0) to a2: zero2 *= cross(a2, (1, 0, 0))[1] < 0 ? 1 : -1 */
    if (a2[2] >= 0.0f)
        zero2 = -zero2;

    /* End of precalculated values */

    /* 4. Rotate to_q by R too */
    quaternion_mult_to(to_q, q, to_q2); /// TODO check order

    /* 5. Use standard formula for quaternion to Tait-Bryan angles in ENU */
    quaternion_to_euler(to_q2, out_angles);

    /* 6. Rebase the inner axis angle on its neutral angle */
    out_angles[2] -= zero2;
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
void axes_q_to_step(struct axes_data_s *data, const float *from_q, const float *to_q,
        float *angles, float damp_factor, float *out_steps) {
    float q_rel[4];
    float from_q_inv[4] = INIT_CONJ_Q(from_q);
    float omega[3];
    float j[3][3], jtj[3][3];

    quaternion_mult_to(to_q, from_q_inv, q_rel);         /* 16 multiplications */
    quaternion_to_rotvec(q_rel, omega);                  /* ~5 multiplications */

    /* Rotate axis[1] and axis[2] by current angles, compose the Jacobian */
    memcpy(j, data->axes, sizeof(j));
    vector_rotate_around_axis(j[1], data->axes[0], angles[0]); /* 19 multiplications, maybe should produce quaternion first */
    vector_rotate_around_axis(j[2], data->axes[1], angles[1]); /* 19 multiplications */
    vector_rotate_around_axis(j[2], data->axes[0], angles[0]); /* 19 multiplications */

    /* Make sure axis[0] and axis[2] are not parallel */
    /* TODO: come up with a good enough fallback if they are, somehow force out_steps[2] to 0 */

    /* Calc J^T * omega */
    vector_mult_matrix(omega, j);                        /* 9 multiplications */

    /* If no damping, we're done */
    if (damp_factor != 0.0f) {
        memcpy(out_steps, omega, sizeof(omega));
        return;
    }

    matrix_jt_mult_j(j, jtj);                            /* 18 multiplications */
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
    vector_solve(jtj, omega, out_steps);                 /* ~51 multiplications */
}
