/* Rules for adding or changing parameter definitions:
 *
 * 1. Do not reorder the definitions, add or remove them from anywhere other
 *    than at the end.  That would break the parameter IDs which are part of
 *    the client API and the storage format, and are based strictly on the
 *    order here.
 *
 *    The IDs are not supposed to be user-visible though so logical grouping
 *    is not a priority -- always add new parameters at the end.
 *
 * 2. If a parameter needs to be removed or deprecated, replace the definition
 *    with a PARAM_UNUSED() to keep its ID reserved from reuse and preserve
 *    other IDs.  It uses no flash space.
 *
 * 3. Lines other than PARAM_DEF_*() can be added anywhere.
 *
 * 4. Name changes are Ok as long as the usage and format of a parameter
 *    isn't changing.
 *
 * 5. Size changes would be bad but not as bad as format changes without a
 *    size change.  Both the firmware and the client can detect size
 *    mismatch and fall back to defaults, report error, etc.  Without a size
 *    difference a critical parameter value could be misinterpreted and
 *    cause physical damage.  Think motor overheating, excess vibration on
 *    fragile camera optics, hitting a hard stop too hard, battery
 *    overdischarge or twisted cables.
 */
PARAM_DEF_RO(commit_str)
PARAM_DEF_RO(build_str)
PARAM_DEF_RO(board_str)
PARAM_DEF_RO(mcu_id)
PARAM_DEF_RO(mcu_rev_id)
PARAM_DEF_RO(mcu_unique_id)
PARAM_DEF_CONST(GCC_VERSION)
PARAM_DEF_CONST(STM32_CORE_VERSION)
PARAM_DEF_CONST(__STM32F3_CMSIS_VERSION)
PARAM_DEF_CONST(ARDUINO)
PARAM_DEF_RO(now)
PARAM_DEF_CONST(TARGET_LOOP_RATE)

PARAM_DEF_SAVE(config.motor_calib[0].bldc_with_encoder.pole_pairs)
PARAM_DEF_SAVE(config.motor_calib[0].bldc_with_encoder.zero_electric_offset)
PARAM_DEF_SAVE(config.motor_calib[0].bldc_with_encoder.sensor_direction)
PARAM_DEF_SAVE(config.motor_calib[1].bldc_with_encoder.pole_pairs)
PARAM_DEF_SAVE(config.motor_calib[1].bldc_with_encoder.zero_electric_offset)
PARAM_DEF_SAVE(config.motor_calib[1].bldc_with_encoder.sensor_direction)
PARAM_DEF_SAVE(config.motor_calib[2].bldc_with_encoder.pole_pairs)
PARAM_DEF_SAVE(config.motor_calib[2].bldc_with_encoder.zero_electric_offset)
PARAM_DEF_SAVE(config.motor_calib[2].bldc_with_encoder.sensor_direction)
PARAM_DEF_SAVE(config.motor_pid[0].kp)
PARAM_DEF_SAVE(config.motor_pid[0].ki)
PARAM_DEF_SAVE(config.motor_pid[0].kd)
PARAM_DEF_SAVE(config.motor_pid[0].kp_trust)
PARAM_DEF_SAVE(config.motor_pid[0].ki_falloff)
PARAM_DEF_SAVE(config.motor_pid[0].v_max)
PARAM_DEF_SAVE(config.motor_pid[0].kdrag)
PARAM_DEF_SAVE(config.motor_pid[1].kp)
PARAM_DEF_SAVE(config.motor_pid[1].ki)
PARAM_DEF_SAVE(config.motor_pid[1].kd)
PARAM_DEF_SAVE(config.motor_pid[1].kp_trust)
PARAM_DEF_SAVE(config.motor_pid[1].ki_falloff)
PARAM_DEF_SAVE(config.motor_pid[1].v_max)
PARAM_DEF_SAVE(config.motor_pid[1].kdrag)
PARAM_DEF_SAVE(config.motor_pid[2].kp)
PARAM_DEF_SAVE(config.motor_pid[2].ki)
PARAM_DEF_SAVE(config.motor_pid[2].kd)
PARAM_DEF_SAVE(config.motor_pid[2].kp_trust)
PARAM_DEF_SAVE(config.motor_pid[2].ki_falloff)
PARAM_DEF_SAVE(config.motor_pid[2].v_max)
PARAM_DEF_SAVE(config.motor_pid[2].kdrag)

PARAM_DEF_SAVE(config.have_axes)
PARAM_DEF_SAVE(config.axes.axes[0])
PARAM_DEF_SAVE(config.axes.axes[1])
PARAM_DEF_SAVE(config.axes.axes[2])
PARAM_DEF_SAVE(config.axes.orthogonal)
PARAM_DEF_SAVE(config.axes.axis_to_encoder[0])
PARAM_DEF_SAVE(config.axes.axis_to_encoder[1])
PARAM_DEF_SAVE(config.axes.axis_to_encoder[2])
PARAM_DEF_SAVE(config.axes.encoder_scale[0])
PARAM_DEF_SAVE(config.axes.encoder_scale[1])
PARAM_DEF_SAVE(config.axes.encoder_scale[2])
PARAM_DEF_SAVE(config.axes.main_imu_mount_q)
PARAM_DEF_SAVE(config.axes.has_limits[0])
PARAM_DEF_SAVE(config.axes.limit_min[0])
PARAM_DEF_SAVE(config.axes.limit_max[0])
PARAM_DEF_SAVE(config.axes.has_limits[1])
PARAM_DEF_SAVE(config.axes.limit_min[1])
PARAM_DEF_SAVE(config.axes.limit_max[1])
PARAM_DEF_SAVE(config.axes.has_limits[2])
PARAM_DEF_SAVE(config.axes.limit_min[2])
PARAM_DEF_SAVE(config.axes.limit_max[2])

PARAM_DEF_SAVE(config.control.home_q)
PARAM_DEF_SAVE(config.control.home_frame_q)
PARAM_DEF_SAVE(config.control.home_angles)
PARAM_DEF_SAVE(config.control.have_home)
PARAM_DEF_SAVE(config.control.forward_vec)
PARAM_DEF_SAVE(config.control.have_forward)
PARAM_DEF_SAVE(config.control.park_angles)
PARAM_DEF_SAVE(config.control.have_parking)
PARAM_DEF_SAVE(config.control.keep_yaw)
PARAM_DEF_SAVE(config.control.follow[0])
PARAM_DEF_SAVE(config.control.follow[1])
PARAM_DEF_SAVE(config.control.follow[2])
PARAM_DEF_SAVE(config.control.max_accel)
PARAM_DEF_SAVE(config.control.max_vel)
PARAM_DEF_SAVE(config.control.prio_max_accel_over_target)
PARAM_DEF_SAVE(config.control.prio_target_max_overshoot)
PARAM_DEF_SAVE(config.control.prio_max_accel_over_max_vel)
PARAM_DEF_SAVE(config.control.prio_target_dir_over_max_accel)
PARAM_DEF_SAVE(config.control.ahrs_velocity_kp)
PARAM_DEF_SAVE(config.control.limit_margin)
PARAM_DEF_SAVE(config.control.limit_search_v)
PARAM_DEF_SAVE(config.control.default_path)
PARAM_DEF_SAVE(config.control.tripod_mode)
PARAM_DEF_SAVE(config.control.rc_mode_angle)
PARAM_DEF_SAVE(config.control.rc_gain)
PARAM_DEF_SAVE(config.control.rc_deadband)
PARAM_DEF(control_enable)
PARAM_DEF(control.path_type)
PARAM_DEF_RO(control.velocity_vec)
PARAM_DEF(control.target_ypr_offsets)
PARAM_DEF_RO(control.delta_angle)
PARAM_DEF_RO(control.limit_search.axis)

PARAM_DEF_SAVE(config.main_ahrs.acc_bias)
PARAM_DEF_SAVE(config.main_ahrs.acc_sensitivity)
PARAM_DEF_SAVE(config.main_ahrs.gyro_bias)
PARAM_DEF_SAVE(config.main_ahrs.gyro_sensitivity)
PARAM_DEF_SAVE(config.main_ahrs.acc_kp)
PARAM_DEF_SAVE(config.main_ahrs.enc_kp)
PARAM_DEF_SAVE(config.main_ahrs.ki)
PARAM_DEF_SAVE(config.main_ahrs.calibrate_on_start)
PARAM_DEF_PTR_RO(main_ahrs, q)
PARAM_DEF_PTR_RO(main_ahrs, velocity_vec)
PARAM_DEF_PTR_RO(main_ahrs, acc_reading)
PARAM_DEF_PTR_RO(main_ahrs, gyro_reading)

PARAM_DEF_SAVE(config.frame_ahrs.acc_bias)
PARAM_DEF_SAVE(config.frame_ahrs.acc_sensitivity)
PARAM_DEF_SAVE(config.frame_ahrs.gyro_bias)
PARAM_DEF_SAVE(config.frame_ahrs.gyro_sensitivity)
PARAM_DEF_SAVE(config.frame_ahrs.acc_kp)
PARAM_DEF_SAVE(config.frame_ahrs.enc_kp)
PARAM_DEF_SAVE(config.frame_ahrs.ki)
PARAM_DEF_SAVE(config.frame_ahrs.calibrate_on_start)
PARAM_DEF_PTR_RO(frame_ahrs, q)
PARAM_DEF_PTR_RO(frame_ahrs, velocity_vec)
PARAM_DEF_PTR_RO(frame_ahrs, acc_reading)
PARAM_DEF_PTR_RO(frame_ahrs, gyro_reading)

PARAM_DEF_PTR_RO(encoders[0], reading)
PARAM_DEF_PTR_RO(encoders[1], reading)
PARAM_DEF_PTR_RO(encoders[2], reading)
PARAM_DEF_PTR_RO(i2c_main, error_cnt)
PARAM_DEF_PTR_RO(i2c_int, error_cnt)

PARAM_DEF_RO(sbgc_api.last_version)
PARAM_DEF_RO(sbgc_api.rx_error_cnt)

PARAM_DEF(use_motor[0])
PARAM_DEF(use_motor[1])
PARAM_DEF(use_motor[2])
PARAM_DEF_PTR_RO(motors[0], ready)
PARAM_DEF_PTR_RO(motors[1], ready)
PARAM_DEF_PTR_RO(motors[2], ready)
PARAM_DEF_PTR_RO(motors[0], pid_stats.tracking)
PARAM_DEF_PTR_RO(motors[1], pid_stats.tracking)
PARAM_DEF_PTR_RO(motors[2], pid_stats.tracking)
PARAM_DEF_PTR_RO(motors[0], pid_stats.tracking_dev)
PARAM_DEF_PTR_RO(motors[1], pid_stats.tracking_dev)
PARAM_DEF_PTR_RO(motors[2], pid_stats.tracking_dev)
PARAM_DEF_PTR_RO(motors[0], pid_stats.i)
PARAM_DEF_PTR_RO(motors[1], pid_stats.i)
PARAM_DEF_PTR_RO(motors[2], pid_stats.i)
PARAM_DEF_SETTER(motors_on, NULL) /* TODO */

PARAM_DEF_RO(frame_q)
PARAM_DEF_RO(rel_q)

PARAM_DEF_RO(vbat)
PARAM_DEF_RO(vbat_ok)
