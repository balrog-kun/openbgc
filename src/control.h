/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef CONTROL_H
#define CONTROL_H

/* Note: changes here may need a STORAGE_CONFIG_VERSION bump in storage.h */
struct control_settings_s {
    /* Geometry config */
    float home_q[4];
    float home_frame_q[4];
    bool have_home;
    float forward_vec[3];
    bool have_forward;

    /* Control config */
    bool keep_yaw;
    bool follow[3]; /* Per euler angle, not gimbal axis */
    float max_accel;
    float max_vel;
    float ahrs_velocity_kp;

    bool rc_mode_angle;
    float rc_gain;       /* deg/s or deg/event at full deflection */
    uint8_t rc_deadband; /* % of full range */
    /* TODO: RC in trims, etc. Probably will end up having to make all the above per angle */
};

struct control_data_s {
    /* Control inputs and outputs */
    struct obgc_ahrs_s *main_ahrs;
    struct obgc_ahrs_s *frame_ahrs;
    struct obgc_encoder_s **encoders;
    struct obgc_motor_s **motors;

    int8_t rc_ypr_readings[3]; /* Not saved over reboot */
    enum {
        SBGC_API_OVERRIDE_NONE,
        SBGC_API_OVERRIDE_SPEED,
        SBGC_API_OVERRIDE_ANGLE,
        SBGC_API_OVERRIDE_RC,
    } sbgc_api_override_mode[3];
    unsigned long sbgc_api_override_ts;
    float sbgc_api_ypr_offsets[3];
    float sbgc_api_ypr_speeds[3];
    bool sbgc_api_follow_override[3];

    /* Aux precalculated inputs */
    const float *rel_q, *frame_q;

    /* Geometry calibration + precalculated values */
    const struct axes_data_s *axes;
    float aligned_home_q[4];
    float conj_aligned_home_frame_q[4];
    float forward_az, forward_sincos2[2];

    /* User settings */
    struct control_settings_s *settings;

    /* TODO: may need to do the same thing in motor-bldc.c where the velocity is calculated
     * based on the length of the previous period (from micros()) but how much force we want
     * to apply may need to be based on how long we'll be applying that force and thus needs
     * to be given from main.cpp who knows the expected period.
     */
    float dt;

    /* State */
    float velocity_vec[3];
    float target_ypr_offsets[3];
};

void control_step(struct control_data_s *control);

#endif /* CONTROL_H */
