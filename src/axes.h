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
struct axes_data_s {
    float axes[3][3];
    int axis_to_encoder[3]; /* -1 for no encoder */
    float encoder_scale[3];
    float main_imu_mount_q[4];
};

struct calibrate_data_s {
    struct sbgc_ahrs_s *main_ahrs;
    struct sbgc_ahrs_s *frame_ahrs;
    struct sbgc_encoder_s **encoders;
    void (*print)(const char *msg);

    struct axes_data_s *out;
};

int axes_calibrate(struct calibrate_data_s *data);
void axes_q_to_angles(struct axes_data_s *data, float *q, float *out_angles);
void axes_q_to_step(struct axes_data_s *data, const float *from_q, const float *to_q,
        float *angles, float damp_factor, float *out_steps);

/* main.cpp */
void main_loop_sleep(void);

#endif /* AXES_H */
