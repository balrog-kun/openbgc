/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef AXES_H
#define AXES_H

/*
 * Unit vectors representing axes of each joint in the local frame of the arm it's mounted on or the base/frame/handle.
 * Index 0 is always the outer joint axis, closest to the frame IMU, in frame's reference frame.
 * Index 1 is the middle joint axis, in the frame of reference of the outer arm (frame IMU frame rotated by the outer joint angle around the outer joint axis.)
 * Index 2 is the inner joint axis, closest to the camera IMU, in the frame of reference of the middle arm (outer arm frame rotated by the middle joint angle.)
 *
 * Frame IMU mounted after the outer joint is not supported right now.
 *
 * It doesn't matter if there is *no* frame IMU -- we can still have a coordinate system based on where it would have been.
 */
/* Note: changes here may need a STORAGE_CONFIG_VERSION bump in storage.h */
struct axes_data_s {
    float axes[3][3];
    int axis_to_encoder[3]; /* -1 for no encoder */
    float encoder_scale[3];
    float main_imu_mount_q[4];
    bool has_limits[3];
    float limit_max[3];
    float limit_min[3];
};

struct axes_calibrate_data_s {
    struct obgc_ahrs_s *main_ahrs;
    struct obgc_ahrs_s *frame_ahrs;
    struct obgc_encoder_s **encoders;
    void (*print)(const char *msg);

    struct axes_data_s *out;
};

int axes_calibrate(struct axes_calibrate_data_s *data);

void axes_precalc_rel_q(const struct axes_data_s *data, struct obgc_encoder_s **encoders,
        const float *main_q, float *out_rel_q, float *out_frame_q);

/* Calculate a step value to be added to each of the current angles to move the end-effector
 * orientation closer to to_q but without a guarantee of the shortest path.
 */
void axes_q_to_step_proj(const struct axes_data_s *data, const float *from_q, const float *to_q,
        const float *angles, float damp_factor, const float *cur_omega_vec,
        float *out_steps, float *out_cur_omega);
void axes_rotvec_to_step_proj(const struct axes_data_s *data, float *new_omega_vec,
        const float *angles, float damp_factor, const float *cur_omega_vec,
        float *out_steps, float *out_cur_omega);

/* Calculate exact joint angles to achieve given orientation.  Generally the _orthogonal
 * functions here assume that the pairs of rotation axes of successive joints are orthogonal,
 * and these functions are cheap.  The _universal versions don't make any assumptions and are
 * significantly more expensive but still constant/bounded time analytical solutions.
 */
void axes_q_to_angles_orthogonal(const struct axes_data_s *data, const float *q, float *out_angles);

#endif /* AXES_H */
