# vim: set ts=4 sw=4 sts=4 et :
from construct import *

#
# Command request and response message payload type definitions following
# SimpleBGC_2_6_Serial_Protocol_Specification.pdf 2025-10-28 version.  Command
# order same as in the PDF, going by sections, but with request and responses
# grouped together.
#

PerAxis = lambda subcon: Array(3, subcon)
# TODO: most uses of Default() in this file are probably wrong, we need to find a way
# to add optional fields with a default value

ErrorCode = Enum(Int8ul,
    CMD_SIZE = 1,
    WRONG_PARAMS = 2,
    CRYPTO = 4,
    UNKNOWN_COMMAND = 6,
    WRONG_STATE = 8,
    NOT_SUPPORTED = 9,
    OPERATION_FAILED = 10,
    TEMPORARY = 11,
    # File operation errors TODO: split these into a separate type, use in ErrorResponse if cmd_id matches an FS operation
    NO_ERROR = 0,
    EEPROM_FAULT = 1,
    FILE_NOT_FOUND = 2,
    FAT = 3,
    NO_FREE_SPACE = 4,
    FAT_IS_FULL = 5,
    FILE_SIZE = 6,
    CRC = 7,
    LIMIT_REACHED = 8,
    FILE_CORRUPTED = 9,
    FILE_WRONG_PARAMS = 10,
)

CmdId = Int8ul # TODO: how do we replace this with the CmdId declaration from the end of the file?
EmergencyStopCode = Int8ul # TODO: Appendix E

MenuCommand = Enum(Int8ul,
    NO = 0,
    PROFILE1 = 1,
    PROFILE2 = 2,
    PROFILE3 = 3,
    SWAP_PITCH_ROLL = 4,
    SWAP_YAW_ROLL = 5,
    CALIB_ACC = 6,
    RESET = 7,
    SET_ANGLE = 8,
    CALIB_GYRO = 9,
    MOTOR_TOGGLE = 10,
    MOTOR_ON = 11,
    MOTOR_OFF = 12,
    FRAME_UPSIDE_DOWN = 13,
    PROFILE4 = 14,
    PROFILE5 = 15,
    AUTO_PID = 16,
    LOOK_DOWN = 17,
    HOME_POSITION = 18,
    RC_BIND = 19,
    CALIB_GYRO_TEMP = 20,
    CALIB_ACC_TEMP = 21,
    BUTTON_PRESS = 22,
    RUN_SCRIPT1 = 23,
    RUN_SCRIPT2 = 24,
    RUN_SCRIPT3 = 25,
    RUN_SCRIPT4 = 26,
    RUN_SCRIPT5 = 27,
    CALIB_MAG = 33,
    LEVEL_ROLL_PITCH = 34,
    CENTER_YAW = 35,
    UNTWIST_CABLES = 36,
    SET_ANGLE_NO_SAVE = 37,
    HOME_POSITION_SHORTEST = 38,
    CENTER_YAW_SHORTEST = 39,
    ROTATE_YAW_180 = 40,
    ROTATE_YAW_180_FRAME_REL = 41,
    SWITCH_YAW_180_FRAME_REL = 42,
    SWITCH_POS_ROLL_90 = 43,
    START_TIMELAPSE = 44,
    CALIB_MOMENTUM = 45,
    LEVEL_ROLL = 46,
    REPEAT_TIMELAPSE = 47,
    LOAD_PROFILE_SET1 = 48,
    LOAD_PROFILE_SET2 = 49,
    LOAD_PROFILE_SET3 = 50,
    LOAD_PROFILE_SET4 = 51,
    LOAD_PROFILE_SET5 = 52,
    LOAD_PROFILE_SET_BACKUP = 53,
    INVERT_RC_ROLL = 54,
    INVERT_RC_PITCH = 55,
    INVERT_RC_YAW = 56,
    SNAP_TO_FIXED_POSITION = 57,
    CAMERA_REC_PHOTO_EVENT = 58,
    CAMERA_PHOTO_EVENT = 59,
    MOTORS_SAFE_STOP = 60,
    CALIB_ACC_AUTO = 61,
    RESET_IMU = 62,
    FORCED_FOLLOW_TOGGLE = 63,
    AUTO_PID_GAIN_ONLY = 64,
    LEVEL_PITCH = 65,
    MOTORS_SAFE_TOGGLE = 66,
    TIMELAPSE_STEP1 = 67,
    EXT_GYRO_ONLINE_CALIB = 68,
    DISABLE_FOLLOW_TOGGLE = 69,
    SET_CUR_POS_AS_HOME = 70,
    STOP_SCRIPT = 71,
    TRIPOD_MODE_OFF = 72,
    TRIPOD_MODE_ON = 73,
    SET_RC_TRIM = 74,
    HOME_POSITION_MOTORS = 75,
    RETRACTED_POSITION = 76,
    SHAKE_GENERATOR_OFF = 77,
    SHAKE_GENERATOR_ON = 78,
    SERVO_MODE_ON = 79,
    SERVO_MODE_OFF = 80,
    SERVO_MODE_TOGGLE = 81,
)

#
# Section: Device Information
#

# CMD_BOARD_INFO (#86) Request - Extended format
BoardInfoRequest = Struct(
    "cfg" / Bytes(2),
    "reserved" / GreedyBytes,
)

# CMD_BOARD_INFO (#86) Response
BoardFeatures = FlagsEnum(Int16ul,
    AXIS_3               = 1 << 0,
    BAT_MONITORING       = 1 << 1,
    ENCODERS             = 1 << 2,
    BODE_TEST            = 1 << 3,
    SCRIPTING            = 1 << 4,
    CURRENT_SENSOR       = 1 << 5,
    MAG_SENSOR           = 1 << 6,
    ORDER_OF_AXES_LETUS  = 1 << 7,
    IMU_EEPROM           = 1 << 8,
    FRAME_IMU_EEPROM     = 1 << 9,
    CAN_PORT             = 1 << 10,
    MOMENTUM             = 1 << 11,
    COGGING_CORRECTION   = 1 << 12,
    MOTOR4_CONTROL       = 1 << 13,
    ACC_AUTO_CALIB       = 1 << 14,
    BIG_FLASH            = 1 << 15,
)

BoardFeaturesExt = FlagsEnum(Int16ul,
    EXT_IMU              = 1 << 0,  # bit 16 in full flags
    STATE_VARS           = 1 << 2,  # bit 18
    POWER_MANAGEMENT     = 1 << 3,  # bit 19
    GYRO_ADVANCED_CALIB  = 1 << 4,  # bit 20
    LIMITED_VERSION      = 1 << 5,  # bit 21
    REACTION             = 1 << 6,  # bit 22
    ENCODER_LUT          = 1 << 7,  # bit 23
)

ConnectionFlag = FlagsEnum(Int8ul,
    CONNECTION_USB = 1,
)

# Low 8 bits of SystemStateFlags
SystemStateFlagsTruncated = FlagsEnum(Int8ul,
    DEBUG_MODE = 1 << 0,
    IS_FRAME_INVERTED = 1 << 1,
    INIT_STEP1_DONE = 1 << 2,
    INIT_STEP2_DONE = 1 << 3,
    STARTUP_AUTO_ROUTINE_DONE = 1 << 4,
    POWER_MANAGER_ENABLED = 1 << 5,
)

BoardInfoResponse = Struct(
    "board_ver" / Int8ul,
    "firmware_ver" / Int16ul,
    "state_flags" / SystemStateFlagsTruncated,
    "board_features" / BoardFeatures,
    "connection_flag" / ConnectionFlag,
    "frw_extra_id" / Int32ul,
    "board_features_ext" / BoardFeaturesExt,
    "main_imu_sens_model" / Int8ul,
    "frame_imu_sens_model" / Int8ul,
    "build_number" / Int8ul,
    "base_frw_ver" / Int16ul,
)

# CMD_BOARD_INFO_3 (#20) Request
BoardInfo3Request = Pass # No parameters

# CMD_BOARD_INFO_3 (#20) Response
BoardFeaturesExt2 = FlagsEnum(Int32ul,
    PLUS_VER         = 1 << 0,
    SHAKE_GENERATOR  = 1 << 1,
    EXT_MOTORS       = 1 << 2,
    QUAT_CONTROL     = 1 << 3,
    ADC4             = 1 << 4,
)

HardwareFlags = FlagsEnum(Int16ul,
    ONE_WIRE_CRYPTO_IC = 1 << 0,
)

BoardInfo3Response = Struct(
    "device_id" / Bytes(9),
    "mcu_id" / Bytes(12),
    "eeprom_size" / Int32ul,
    "script_slot1_size" / Int16ul,
    "script_slot2_size" / Int16ul,
    "script_slot3_size" / Int16ul,
    "script_slot4_size" / Int16ul,
    "script_slot5_size" / Int16ul,
    "profile_set_slots" / Int8ul,
    "profile_set_cur" / Int8ul,
    "flash_size" / Int8ul,
    "imu_calib_info" / Bytes(2),
    "script_slot6_size" / Int16ul,
    "script_slot7_size" / Int16ul,
    "script_slot8_size" / Int16ul,
    "script_slot9_size" / Int16ul,
    "script_slot10_size" / Int16ul,
    "hw_flags" / HardwareFlags,
    "board_features_ext2" / BoardFeaturesExt2,
    "can_drv_main_limit" / Int8ul,
    "can_drv_aux_limit" / Int8ul,
    "adj_vars_total_num" / Int8ul,
    "reserved" / Bytes(10),
)

#
# Section: Configuring Gimbal
#

# CMD_READ_PARAMS (#82), CMD_READ_PARAMS_3 (#21)
# CMD_READ_PARAMS_EXT (#33), CMD_READ_PARAMS_EXT2 (#62), CMD_READ_PARAMS_EXT3 (#104)
ReadParamsRequest = Struct(
    "profile_id" / Int8ul,  # 0-4, or >4 for current profile
)

# CMD_READ_PARAMS_3 (#21) Response
RcMode = FlagsEnum(Int8ul,
    MODE_ANGLE = 0,
    MODE_SPEED = 1,
    INVERTED = 1 << 3,
)

PwmFreq = Enum(Int8ul,
    LOW = 0,
    HIGH = 1,
    ULTRA_HIGH = 2,
)

SerialSpeed = Enum(Int8ul,
    BAUD_115200 = 0,
    BAUD_57600 = 1,
    BAUD_38400 = 2,
    BAUD_19200 = 3,
    BAUD_9600 = 4,
    BAUD_256000 = 5,
)

RcVirtMode = Enum(Int8ul,
    NORMAL = 0,
    CPPM = 1,
    SBUS = 2,
    SPEKTRUM = 3,
    API = 10,
)

SourcePinId = FlagsEnum(Int8ul,
    # PWM
    PWM_INPUT_BASE = 0,

    RC_INPUT_ROLL = 1,
    RC_INPUT_PITCH = 2,
    EXT_FC_INPUT_ROLL = 3,
    EXT_FC_INPUT_PITCH = 4,
    RC_INPUT_YAW = 5,

    # Analog (channels 1..4)
    ANALOG_INPUT_BASE = 32,

    ADC1 = 33,
    ADC2 = 34,
    ADC3 = 35,
    ADC4 = 36,

    # RC Serial (CPPM/SBUS/SPEKTRUM) (virtual channels 1..31)
    RC_SERIAL_BASE = 64,

    # API Virtual control (virtual channels 1..31)
    API_VCH_BASE = 128,

    # Step signal (channels 1..6)
    STEP_SIGNAL_BASE = 160,
)

FollowMode = Enum(Int8ul,
    DISABLED = 0,
    FC = 1,
    PITCH = 2,
)

FrameImuPos = Enum(Int8ul,
    DISABLED = 0,
    BELOW_YAW = 1,
    ABOVE_YAW = 2,
    BELOW_YAW_PID_SOURCE = 3,
)

BeeperMode = FlagsEnum(Int8ul,
    CALIBRATE = 1 << 0,
    CONFIRM = 1 << 1,
    ERROR = 1 << 2,
    ALARM = 1 << 3,
    BEEP_BY_MOTORS = 1 << 7,
)

SkipGyroCalib = Enum(Int8ul,
    DO_NOT_SKIP = 0,
    SKIP_ALWAYS = 1,
    SKIP_IF_MOTION = 2,
)

ImuOrientation = Enum(Int8sl, X=1, Y=2, Z=3, NEG_X=-1, NEG_Y=-2, NEG_Z=-3)

MotorOutput = Enum(Int8ul,
    DISABLED = 0,
    ROLL = 1,
    PITCH = 2,
    YAW = 3,
    I2C_DRV1 = 4,
    I2C_DRV2 = 5,
    I2C_DRV3 = 6,
    I2C_DRV4 = 7,
)

GeneralFlags1 = FlagsEnum(Int16ul,
    REMEMBER_LAST_USED_PROFILE = 1 << 0,
    UPSIDE_DOWN_AUTO = 1 << 1,
    SWAP_FRAME_MAIN_IMU = 1 << 2,
    BLINK_PROFILE = 1 << 3,
    EMERGENCY_STOP = 1 << 4,
    MAGNETOMETER_POS_FRAME = 1 << 5,
    FRAME_IMU_FF = 1 << 6,
    OVERHEAT_STOP_MOTORS = 1 << 7,
    CENTER_YAW_AT_STARTUP = 1 << 8,
    SWAP_RC_SERIAL_UART_B = 1 << 9,
    UART_B_SERIAL_API = 1 << 10,
    BLINK_BAT_LEVEL = 1 << 11,
    ADAPTIVE_GYRO_TRUST = 1 << 12,
    IS_UPSIDE_DOWN = 1 << 13,
    UART_3_SERIAL_API = 1 << 14,
    FRAME_INV_RC_INV = 1 << 15,
)

ProfileFlags1 = FlagsEnum(Int16ul,
    ADC1_AUTO_DETECTION = 1 << 0,
    ADC2_AUTO_DETECTION = 1 << 1,
    ADC3_AUTO_DETECTION = 1 << 2,
    FOLLOW_USE_FRAME_IMU = 1 << 4,
    BRIEFCASE_AUTO_DETECTION = 1 << 5,
    UPSIDE_DOWN_AUTO_ROTATE = 1 << 6,
    FOLLOW_LOCK_OFFSET_CORRECTION = 1 << 7,
    START_NEUTRAL_POSITION = 1 << 8,
    MENU_BUTTON_DISABLE_FOLLOW = 1 << 9,
    TIMELAPSE_FRAME_FIXED = 1 << 10,
    RC_KEEP_MIX_RATE = 1 << 11,
    RC_KEEP_CUR_POS_ON_INIT = 1 << 12,
    OUTER_MOTOR_LIMIT_FREE_ROTATION = 1 << 13,
    GIMBAL_LOCK_SMOOTH_TRANSITION = 1 << 14,
    CAM_UPSIDE_DOWN_WORKING = 1 << 15,
)

SpektrumMode = Enum(Int8ul,
    AUTO = 0,
    DSM2_11MS_10BIT = 1,
    DSM2_11MS_11BIT = 2,
    DSM2_22MS_10BIT = 3,
    DSM2_22MS_11BIT = 4,
    DSMX_11MS_10BIT = 5,
    DSMX_11MS_11BIT = 6,
    DSMX_22MS_10BIT = 7,
    DSMX_22MS_11BIT = 8,
)

OrderOfAxes = Enum(Int8ul,
    PITCH_ROLL_YAW = 0,
    YAW_ROLL_PITCH = 1,
    ROLL_YAW_PITCH = 2,
    ROLL_PITCH_YAW = 3,
)

EulerOrder = FlagsEnum(Int8ul,
    PITCH_ROLL_YAW = 0,
    ROLL_PITCH_YAW = 1,
    PITCH_M_ROLL_YAW_M = 2,
    ROLL_PITCH_M_YAW_M = 3,
    YAW_ROLL_PITCH = 4,
    CONTROL_LOCAL_XYZ = 1 << 4,
    STAB_CONTROL_LOCAL_XYZ = 1 << 5,
)

ImuType = Enum(Int8ul,
    MAIN = 1,
    FRAME = 2,
)

ReadParams3Response = Struct(
    "profile_id" / Int8ul,

    "pid" / PerAxis(Struct(
        "p" / Int8ul,
        "i" / Int8ul,
        "d" / Int8ul,
        "power" / Int8ul,
        "invert" / Int8ul,
        "poles" / Int8ul,
    )),

    "acc_limiter_all" / Int8ul,  # Deprecated

    "ext_fc_gain" / Array(3, Array(2, Int8sl)),

    # RC configuration per axis
    "rc_axes" / PerAxis(Struct(
        "min_angle" / Int16sl,
        "max_angle" / Int16sl,
        "mode" / RcMode,
        "lpf" / Int8ul,
        "speed" / Int8ul,
        "follow" / Int8sl,
    )),

    "gyro_trust" / Int8ul,
    "use_model" / Int8ul,
    "pwm_freq" / PwmFreq,
    "serial_speed" / SerialSpeed,

    "rc_trim" / PerAxis(Int8sl),
    "rc_deadband" / Int8ul,
    "rc_expo_rate" / Int8ul,
    "rc_virt_mode" / RcVirtMode,

    # RC mapping
    "rc_map" / Struct(
        "roll" / SourcePinId,
        "pitch" / SourcePinId,
        "yaw" / SourcePinId,
        "cmd" / SourcePinId,
        "fc_roll" / SourcePinId,
        "fc_pitch" / SourcePinId,
    ),

    "rc_mix_fc_roll" / Int8ul,  # Mix rate in bits 0..5, target channel in bits 6, 7
    "rc_mix_fc_pitch" / Int8ul,

    "follow_mode" / FollowMode,
    "follow_deadband" / Int8ul,
    "follow_expo_rate" / Int8ul,
    "follow_offset" / PerAxis(Int8sl),  # Deprecated in 2.70+ in favour of follow_offset_ext

    "imu_orientation" / Struct(
        "axis_top" / ImuOrientation,
        "axis_right" / ImuOrientation,
        "frame_axis_top" / ImuOrientation,
        "frame_axis_right" / ImuOrientation,
    ),
    "frame_imu_pos" / FrameImuPos,

    "gyro_deadband" / Int8ul,
    "gyro_sens" / Int8ul,  # Deprecated
    "i2c_speed_fast" / Int8ul,
    "skip_gyro_calib" / SkipGyroCalib,

    # Menu commands
    "rc_cmd" / Struct(
        "low" / MenuCommand,
        "mid" / MenuCommand,
        "high" / MenuCommand,
    ),
    "menu_btn" / Struct(
        "cmd_1" / MenuCommand,
        "cmd_2" / MenuCommand,
        "cmd_3" / MenuCommand,
        "cmd_4" / MenuCommand,
        "cmd_5" / MenuCommand,
        "cmd_long" / MenuCommand,
    ),

    "motor_output" / PerAxis(MotorOutput),

    "bat_threshold_alarm" / Int16sl,
    "bat_threshold_motors" / Int16sl,
    "bat_comp_ref" / Int16sl,
    "beeper_modes" / BeeperMode,

    "follow_roll_mix_start" / Int8ul,
    "follow_roll_mix_range" / Int8ul,

    "booster_power" / PerAxis(Int8ul),
    "follow_speed" / PerAxis(Int8ul),

    "frame_angle_from_motors" / Int8ul,
    "rc_memory" / Array(3, Int16sl),

    "servo1_out" / Int8ul,
    "servo2_out" / Int8ul,
    "servo3_out" / Int8ul,
    "servo4_out" / Int8ul,
    "servo_rate" / Int8ul,

    "adaptive_pid" / Struct(
        "enabled" / Int8ul,
        "threshold" / Int8ul,
        "rate" / Int8ul,
        "recovery_factor" / Int8ul,
    ),

    "follow_lpf" / PerAxis(Int8ul),

    "general_flags1" / GeneralFlags1,
    "profile_flags1" / ProfileFlags1,

    "spektrum_mode" / SpektrumMode,
    "order_of_axes" / OrderOfAxes,
    "euler_order" / EulerOrder,

    "cur_imu" / ImuType,
    "cur_profile_id" / Int8ul,
)

# CMD_READ_PARAMS_EXT (#33) Response
FiltersEnable = FlagsEnum(Int8ul,
    NOTCH1 = 1 << 0,
    NOTCH2 = 1 << 1,
    NOTCH3 = 1 << 2,
    LPF = 1 << 3,
)

EncoderType = FlagsEnum(Int8ul,
    AS5048A = 1,
    AS5048B = 2,
    AS5048_PWM = 3,
    AMT203 = 4,
    MA3_10BIT = 5,
    MA3_12BIT = 6,
    ANALOG = 7,
    I2C_DRV1 = 8,
    I2C_DRV2 = 9,
    I2C_DRV3 = 10,
    I2C_DRV4 = 11,
    AS5600_PWM = 12,
    AS5600_I2C = 13,
    RLS_ORBIS = 14,
    RLS_ORBIS_PWM = 15,
    SKIP_DETECTION = 1 << 4,
    IS_GEARED = 1 << 7,
)

EncoderConfig = Enum(Int8ul,
    SPI_500KHZ = 3,
    SPI_1MHZ = 0,
    SPI_2MHZ = 1,
    SPI_4MHZ = 2,
)

ReadParamsExtResponse = Struct(
    "profile_id" / Int8ul,

    "notch_filter" / PerAxis(Struct(
        "notch_freq" / Array(3, Int8ul),
        "notch_width" / Array(3, Int8ul),
    )),
    "lpf_freq" / PerAxis(Int16ul),
    "filters_en" / PerAxis(FiltersEnable),

    # Encoder configuration per axis
    "encoder_offset" / PerAxis(Int16sl),
    "encoder_fld_offset" / PerAxis(Int16sl),
    "encoder_manual_set_time" / PerAxis(Int8ul),

    "motor_heating_factor" / PerAxis(Int8ul),
    "motor_cooling_factor" / PerAxis(Int8ul),

    "reserved" / Bytes(2),

    "follow_inside_deadband" / Int8ul,

    "motor_mag_link" / PerAxis(Int8ul),  # Deprecated
    "motor_gearing" / PerAxis(Int16ul),

    "encoder_limit_min" / PerAxis(Int8sl),  # Deprecated
    "encoder_limit_max" / PerAxis(Int8sl),

    "notch1_gain" / PerAxis(Int8sl),
    "notch2_gain" / PerAxis(Int8sl),
    "notch3_gain" / PerAxis(Int8sl),

    "beeper_volume" / Int8ul,

    "encoder_gear_ratio" / PerAxis(Int16ul),
    "encoder_type" / PerAxis(EncoderType),
    "encoder_cfg" / PerAxis(EncoderConfig),

    "outer_p" / PerAxis(Int8ul),

    "reserved2" / Bytes(3),

    "mag" / Struct(
        "axis_top" / Int8sl,
        "axis_right" / Int8sl,
        "trust" / Int8ul,
        "declination" / Int8sl,
    ),

    "acc_lpf_freq" / Int16ul,

    "d_term_lpf_freq" / PerAxis(Int8ul),
)

# CMD_READ_PARAMS_EXT2 (#62) Response
MavSrc = Enum(Int8ul,
    DISABLED = 0,
    UART1 = 1,
    RC_SERIAL = 2,
    UART2 = 3,
    USB_VCP = 4,
)

MavCfgFlags = FlagsEnum(Int8ul,
    BAUD_MASK = 0x07,  # bits 0-2
    PARITY_EVEN = 1 << 3,
    HEARTBEAT = 1 << 4,
    DEBUG = 1 << 5,
    RC = 1 << 6,
)

AutoPidCfgFlags = FlagsEnum(Int8ul,
    ROLL = 1 << 0,
    PITCH = 1 << 1,
    YAW = 1 << 2,
    SEND_GUI = 1 << 3,
    KEEP_CURRENT = 1 << 4,
    TUNE_LPF_FREQ = 1 << 5,
    ALL_PROFILES = 1 << 6,
)

GeneralFlags2 = FlagsEnum(Int16ul,
    SEARCH_LIMIT_ROLL = 1 << 0,
    SEARCH_LIMIT_PITCH = 1 << 1,
    SEARCH_LIMIT_YAW = 1 << 2,
    AUTO_CALIBRATE_MOMENTUM = 1 << 3,
    USE_MOMENTUM_FEED_FORWARD = 1 << 4,
    MOTORS_OFF_AT_STARTUP = 1 << 5,
    FC_BELOW_OUTER = 1 << 6,
    DO_NOT_CHECK_ENCODER_LIMITS = 1 << 7,
    AUTO_SAVE_BACKUP_SLOT = 1 << 8,
    FC_BELOW_MIDDLE = 1 << 9,
    ENVIRONMENT_TEMP_UNKNOWN = 1 << 10,
    LPF_EXTENDED_RANGE = 1 << 11,
    SAVE_SYSTEM_STAT = 1 << 12,
    FLAG2_DISABLE_ACC = 1 << 13,
    FLAG2_DISABLE_POWER_MANAGER = 1 << 14,
    ALLOW_ONBOARD_IMU_AS_MAIN = 1 << 15,
)

MavCtrlMode = Enum(Int8ul,
    DISABLED = 0,
    ROLL_PITCH = 1,
    ALL_AXES = 2,
)

ParkingPosConfig = FlagsEnum(Int8ul,
    ROLL_NEG = 1 << 0,
    ROLL_POS = 1 << 1,
    PITCH_NEG = 1 << 2,
    PITCH_POS = 1 << 3,
    YAW_NEG = 1 << 4,
    YAW_POS = 1 << 5,
)

# Note similarity with SourcePinId but not exact match
TriggerPinId = Enum(Int8ul,
    RC_INPUT_ROLL = 1,
    RC_INPUT_PITCH = 2,
    EXT_FC_INPUT_ROLL = 3,
    EXT_FC_INPUT_PITCH = 4,
    RC_INPUT_YAW = 5,
    PIN_AUX1 = 16,
    PIN_AUX2 = 17,
    PIN_AUX3 = 18,
    PIN_BUZZER = 32,
    PIN_SSAT_POWER = 33,
)

ReadParamsExt2Response = Struct(
    "profile_id" / Int8ul,

    # MAVLink channels
    "mav_channels" / Array(2, Struct(
        "src" / MavSrc,
        "sys_id" / Int8ul,
        "comp_id" / Int8ul,
        "cfg_flags" / MavCfgFlags,
        "reserved" / Bytes(4),
    )),

    "motor_mag_link_fine" / PerAxis(Int16ul),
    "acc_limiter" / PerAxis(Int8ul),
    "pid_gain" / PerAxis(Int8ul),

    "frame_imu_lpf_freq" / Int8ul,
    "auto_pid_cfg" / AutoPidCfgFlags,
    "auto_pid_gain" / Int8ul, # See AutoPidRequest.gain_vs_stability

    "frame_cam_angle_min" / PerAxis(Int16sl),
    "frame_cam_angle_max" / PerAxis(Int16sl),

    "general_flags2" / GeneralFlags2,

    "auto_speed" / Int8ul,
    "auto_acc_limiter" / Int8ul,

    "imu_orientation_corr" / PerAxis(Int16sl),

    "timelapse_time" / Int16ul,
    "emergency_stop_restart_delay" / Int16ul,
    "timelapse_acc_part" / Int8ul,

    "momentum" / PerAxis(Int16ul),
    "momentum_calib_stimulus" / PerAxis(Int8ul),
    "momentum_ellipticity" / PerAxis(Int8ul),

    "follow_range" / PerAxis(Int8ul),
    "stab_axis" / PerAxis(Int8ul),

    "outer_mot_tilt_angle" / Int8sl,

    # Startup actions
    "startup_action" / Array(4, MenuCommand), # TODO: document the bit 7 flag
    "startup_action_src" / Array(8, SourcePinId),  # [2][4] flattened
    "startup_action_threshold" / Array(8, Int8sl),  # [2][4] flattened

    "force_position_cfg" / PerAxis(Int8ul), # TODO: enum

    # Step signals (1-6)
    "step_signal" / Array(6, Struct(
        "src" / Int8ul,
        "cfg" / Int8ul,
    )),

    # RC calibration (1-5)
    "rc_calib" / Array(5, Struct(
        "src" / Int8ul,
        "offset" / Int8sl,
        "neg_scale" / Int8ul,
        "pos_scale" / Int8ul,
    )),

    "parking_pos_cfg" / ParkingPosConfig,
    "ext_led_pin_id" / TriggerPinId,
    "interrupt_cfg" / Int16ul, # Unclear.  Based on TriggerPinId but 16-bit or are those 2 8-bit instances?
                               # And then the bit 5 usage conflicts with some TriggerPinId values.
    "overload_time" / Int8ul,
    "auto_pid_momentum" / Int8ul,

    "jerk_slope" / PerAxis(Int8ul),

    "mav_ctrl_mode" / MavCtrlMode,

    "rc_serial_speed" / SerialSpeed,
    "uart2_speed" / SerialSpeed,

    "motor_res" / PerAxis(Int8ul),

    "current_limit" / Int16ul,

    "middle_mot_tilt_angle" / Int8sl,
)

# CMD_READ_PARAMS_EXT3 (#104) Response
ExtImuType = Enum(Int8ul,
    MAVLINK1 = 1,
    MAVLINK2 = 2,
    VECTORNAV_VN200 = 3,
    INERTIALSENSE_UAHRS = 4,
)

ExtImuPort = Enum(Int8ul,
    DISABLED = 0,
    UART1 = 1,
    RC_SERIAL = 2,
    UART2 = 3,
    USB_VCP = 4,
)

ExtImuPosition = Enum(Int8ul,
    BELOW_OUTER = 1,
    ABOVE_OUTER = 2,
    BELOW_MIDDLE = 8,
    MAIN_IMU = 9,
)

ExtImuFlags = FlagsEnum(Int16ul,
    ACC_COMP_ONLY = 1 << 1,
    REPLACE = 1 << 2,
    Z = 1 << 3,
    H = 1 << 4,
    FRAME_UPSIDE_DOWN_UPDATE = 1 << 5,
    AS_FRAME_IMU = 1 << 6,
    GYRO_CORR = 1 << 7,
)

CanImuExtSensType = Enum(Int8ul,
    DISABLED = 0,
    KVH_1725 = 1,
    KVH_1750_ACC_2G = 2,
    KVH_1750_ACC_10G = 3,
    KVH_1750_ACC_30G = 4,
    KVH_1775_ACC_10G = 5,
    KVH_1775_ACC_25G = 6,
    KVH_1760 = 7,
    ADXRS453 = 8,
    ADIS16460 = 9,
    STIM210 = 10,
    STIM300 = 11,
    SCHA63X = 12,
    VECTORNAV_VN100_200_UART = 64,
    VECTORNAV_VN100_200_SPI = 65,
)

ProfileFlags2 = FlagsEnum(Int16ul,
    FOLLOW_PITCH_DISABLED = 1 << 0,
    LOW_ANGLE_PRIOR_ROLL = 1 << 1,
    LOW_ANGLE_PRIOR_PITCH = 1 << 2,
    LOW_ANGLE_PRIOR_YAW = 1 << 3,
    HEADING_TRIPOD_MODE = 1 << 4,
    MOT_ROLL_DISABLED = 1 << 5,
    MOT_PITCH_DISABLED = 1 << 6,
    MOT_YAW_DISABLED = 1 << 7,
    RC_OUT_DUTY1 = 1 << 8,
    RC_OUT_DUTY2 = 1 << 9,
    RC_OUT_DUTY3 = 1 << 10,
    RC_OUT_DUTY4 = 1 << 11,
    DISABLE_FOLLOW_STARTUP = 1 << 12,
    SERVO_MODE = 1 << 13,
    RC_EXPO_DEADBAND_SPLIT = 1 << 14,
)

GeneralFlags3 = FlagsEnum(Int32ul,
    ENC_LUT_EN_ROLL = 1 << 0,
    ENC_LUT_EN_PITCH = 1 << 1,
    ENC_LUT_EN_YAW = 1 << 2,
    MAVLINK_YAW_ABSOLUTE = 1 << 3,
    EXT_SENS_AS_REFERENCE = 1 << 4,
    DISABLE_BT = 1 << 5,
    ALLOW_SENSOR_RECOVERY = 1 << 6,
    UART2_ALT = 1 << 7,
    PASS_LIMIT_ROLL = 1 << 8,
    PASS_LIMIT_PITCH = 1 << 9,
    PASS_LIMIT_YAW = 1 << 10,
    DONT_PUSH_SOFT_LIMITS_ROLL = 1 << 11,
    DONT_PUSH_SOFT_LIMITS_PITCH = 1 << 12,
    DONT_PUSH_SOFT_LIMITS_YAW = 1 << 13,
    TRIPOD_MODE_AUTO = 1 << 14,
    TRIPOD_MODE_GYRO_ONLINE_CALIB = 1 << 15,
    RETRACTED_POSITION_MOTORS_OFF = 1 << 16,
    CAN_IMU_THERMOSTAT_ENABLE = 1 << 17,
    RETRACTED_POSITION_SHORTEST = 1 << 18,
    RETRACTED_POSITION_RC_CONTROL = 1 << 19,
    ACC_LIMIT_EXT_RANGE = 1 << 20,
    MOVE_SHORT_PATH_ROLL = 1 << 21,
    MOVE_SHORT_PATH_PITCH = 1 << 22,
    MOVE_SHORT_PATH_YAW = 1 << 23,
    FRAME_INV_SOFT_LIMIT_MIDDLE = 1 << 24,
    FRAME_INV_SOFT_LIMIT_OUTER = 1 << 25,
    CAN_IMU_SERVO1_DUTY_MODE = 1 << 26,
    CAN_IMU_SERVO2_DUTY_MODE = 1 << 27,
)

ProfileFlags3 = FlagsEnum(Int32ul,
    ACC_LIMIT_AUTO_R = 1 << 0,
    ACC_LIMIT_AUTO_P = 1 << 1,
    ACC_LIMIT_AUTO_Y = 1 << 2,
    SHAKE_GENERATOR_ENABLED = 1 << 3,
    ERR_CORR_LONG_R = 1 << 4,
    ERR_CORR_LONG_P = 1 << 5,
    ERR_CORR_LONG_Y = 1 << 6,
)

GeneralFlags4 = FlagsEnum(Int32ul,
    ACC_LIMIT_BY_MOMENTUM = 1 << 0,
    CALIB_MOMENTUM_IN_NORMAL_POS = 1 << 1,
    EULER_INVERSE_INIT_MASK = 0x0c,  # bits 2-3 TODO
)

ReadParamsExt3Response = Struct(
    "profile_id" / Int8ul,
    "reserved" / Bytes(21),

    "ext_imu" / Struct(
        "type" / ExtImuType,
        "port" / ExtImuPort,
        "position" / ExtImuPosition,
        "orientation" / Int8ul,
        "flags" / ExtImuFlags,
        "align_correction" / PerAxis(Int16sl),
        "startup_delay" / Int8ul,
        "gyro_corr_rate" / Int8ul,
        "reserved" / Bytes(4),
    ),

    "soft_limit_width" / PerAxis(Int8ul),
    "adc_replace_src" / PerAxis(SourcePinId),

    "glock_mid_mot_pos_corr_rate" / Int8ul,

    # Extra buttons (5)
    "extra_btn_cfg" / Array(5, FlagsEnum(TriggerPinId, LATCH_MODE = 1 << 6, INVERT = 1 << 7)),

    "power_ctrl_cfg" / Struct(
        "overcurrent_protection" / Int8ul,  # units: 0.5A
        "power_on_delay" / Int8ul,  # units: 100ms
        "power_off_delay" / Int8ul,  # units: 100ms
        "power_on_limiter" / Int8ul,  # 0-255
        "reserved" / Bytes(4),
    ),

    "lpf_q_inv" / PerAxis(Int8ul),

    "can_imu_ext_sens_type" / CanImuExtSensType,
    "profile_flags2" / ProfileFlags2,
    "reserved2" / Bytes(3),
    "general_flags3" / GeneralFlags3,

    "follow_offset_ext" / PerAxis(Int16sl),

    "motor_startup_delay" / Int16ul,

    "imu_model_main" / Int8ul,
    "imu_model_frame" / Int8ul,

    "stab_threshold_angle" / PerAxis(Int8ul),

    "ext_buzzer_pin" / Int8ul,

    "enc_limit_return_speed" / PerAxis(Int8ul),

    "tripod_mode_auto_threshold" / Int8ul,

    "rc_deadband_pitch" / Int8ul,
    "rc_deadband_yaw" / Int8ul,
    "rc_expo_rate_pitch" / Int8ul,
    "rc_expo_rate_yaw" / Int8ul,

    "profile_flags3" / ProfileFlags3,
    "default_profile" / Int8ul,

    "retracted_angle" / PerAxis(Int16sl),

    "shake_generator_cfg" / Struct(
        "amplitude" / PerAxis(Int8ul),  # ROLL, TILT, PAN
        "base_freq" / Int8ul,
        "freq_range" / Int8ul,
        "pause_period" / Int8ul,
        "pause_balance" / Int8ul,
        "pause_attenuation" / Int8ul,
        "pause_randomness" / Int8ul,
        "pause_phase_var" / Int8ul,
        "resonance_gain" / Array(3, Int8ul),
        "resonance_freq" / Int8ul,
        "freq_shift" / Array(3, Int8sl),
        "reserved" / Bytes(5),
    ),

    "ext_motor_drv_ids" / Array(7, Int8ul),

    "encoder_cfg_ext" / PerAxis(Int16ul),

    "ext_sens_params" / Bytes(16),

    "imu_angle_corr_r" / Int16sl,
    "imu_angle_corr_p" / Int16sl,

    "can_imu_servo_out" / Array(2, Int8ul),
    "can_imu_servo_rate" / Int8ul,

    "servo_outer_p_mult" / Int8ul,
    "servo_outer_i" / Int8ul,

    "ext_pwr_sw_shunt_r" / Int16ul,
    "general_flags4" / GeneralFlags4,

    "auto_motion_timeout" / Int8ul,

    "reserved3" / Bytes(48),
)

# CMD_WRITE_PARAMS_SET (#119)
WriteParamsSetRequest = Struct(
    "action" / Enum(Int8ul,
        START = 1,
        FINISH = 0,
    ),
)

# CMD_USE_DEFAULTS (#70)
UseDefaultsRequest = Struct(
    "profile_id" / Int8ul,  # 0-4, 253=erase EEPROM, 254=reset current
)

# CMD_CALIB_OFFSET (#79)
CalibOffsetRequest = Pass # No parameters

# CMD_READ_PROFILE_NAMES (#28)
ReadProfileNamesRequest = Pass # No parameters

# CMD_READ_PROFILE_NAMES (#28) Response
ReadProfileNamesResponse = Struct(
    "profile_names" / Array(5, PaddedString(48, "utf8")),
)
# CMD_WRITE_PROFILE_NAMES (#29)
WriteProfileNamesRequest = Struct(
    "profile_names" / Array(5, PaddedString(48, "utf8")),
)

# CMD_PROFILE_SET (#95)
ProfileSetAction = Enum(Int8ul,
    SAVE = 1,
    CLEAR = 2,
    LOAD = 3,
)

ProfileSetRequest = Struct(
    "slot" / Int8ul,  # 1-5 regular, 6=backup
    "action" / ProfileSetAction,
    "reserved" / Bytes(8),
)

#
# Section: Calibrating
#

# CMD_CALIB_ACC (#65), CMD_CALIB_GYRO (#103), CMD_CALIB_MAG (#59)
# Note similarity with ImuType
ImuIdx = Enum(Int8ul,
    CURRENT = 0,
    MAIN = 1,
    FRAME = 2,
)

CalibAction = Enum(Int8ul,
    REGULAR = 1,
    RESET_ALL = 2,
    TEMP_CALIB = 3,
    ENABLE_TEMP_CALIB = 4,
    DISABLE_TEMP_CALIB = 5,
    RESTORE_FACTORY = 6,
    SAVE_TO_SENSOR = 7,
)

# TODO: Change the Optional()s to something that accepts a default value for parsing
# Default() seems to provide deafults for building only...?

CalibSensorExtRequest = Optional(Struct(
    "imu_idx" / ImuIdx,
    "action" / CalibAction,
    "time_ms" / Int16ul,  # 0 = default time (~4 seconds)
    "reserved" / Bytes(8),
))

# CMD_CALIB_EXT_GAIN (#71)
CalibExtGainRequest = Pass # No parameters

# CMD_CALIB_POLES (#80)
CalibPolesRequest = Pass # No parameters

# CMD_CALIB_BAT (#66)
CalibBatRequest = Struct(
    "actual_voltage" / Int16ul,  # Units: 0.01V
)

# CMD_ENCODERS_CALIB_OFFSET_4 (#26)
EncodersCalibOffset4Request = Optional(Struct(
    # Extended format (frw. ver. 2.68b7+), all motors if not given
    "for_motor" / Int8ul,  # 0-2 for specific motor, 255 for all
))

# CMD_ENCODERS_CALIB_FLD_OFFSET_4 (#27)
EncoderCalibFlags = FlagsEnum(Int16ul,
    IGNORE_IMU_CHECK = 1 << 0,
    IGNORE_ENCODER_CHECK = 1 << 1,
)

EncodersCalibFldOffset4Request = Optional(Struct(
    "calib_angle" / PerAxis(Int16sl),  # Optional (frw. ver. 2.62b6+), Units: 0.02197265625 deg
    "extended" / Optional(Struct(
        "calib_speed" / PerAxis(Int16sl), # Optional (frw. ver. 2.71b1+), Units: 0.06103701895 deg/sec
        "calib_flags" / Optional(EncoderCalibFlags), # Optional (frw. ver. 2.70b8+)
    )),
))

# CMD_CALIB_ORIENT_CORR (#91)
CalibOrientCorrRequest = Struct(
    "reserved" / Bytes(16),
)

# CMD_CALIB_ACC_EXT_REF (#94)
CalibAccExtRefRequest = Struct(
    "acc_ref" / PerAxis(Int16sl),  # [X,Y,Z] in frame coordinates, Units: 1g/512
    "reserved" / Bytes(14),
)

# CMD_CALIB_COGGING (#93)
CoggingAction = Enum(Int8ul,
    CALIBRATE = 1,
    DELETE = 2,
)

CalibCoggingRequest = Struct(
    "action" / CoggingAction,
    "axis_to_calibrate" / FlagsEnum(Int8ul,
        ROLL = 1 << 0,
        PITCH = 1 << 1,
        YAW = 1 << 2,
    ),
    "axes" / PerAxis(Struct(
        "angle" / Int16ul,  # degrees, 20-360
        "smooth" / Int8ul,  # 0-100, smoothing percentage
        "speed" / Int8ul,   # relative units
        "period" / Int16ul, # expected period in degrees, 0 for auto-detect
        "reserved" / Bytes(9),
    )),
    "iterations_num" / Int8ul,  # 2+
    "reserved2" / Bytes(9),
)

# CMD_SYNC_MOTORS (#123)
SyncMotorsRequest = Struct(
    "axis" / Enum(Int8ul,
        ROLL = 0,
        PITCH = 1,
        YAW = 2,
    ),
    "power" / Int8ul,  # 0-255
    "time_ms" / Int16ul,  # 0-65535
    "angle" / Int16ul,  # 0 to hold current position
)

#
# Section: Real-time state monitoring and diagnostics
#

# CMD_REALTIME_DATA_CUSTOM (#88)
RealtimeDataCustomFlags = FlagsEnum(Int32ul,
    IMU_ANGLES = 1 << 0,
    TARGET_ANGLES = 1 << 1,
    TARGET_SPEED = 1 << 2,
    FRAME_CAM_ANGLE = 1 << 3,
    GYRO_DATA = 1 << 4,
    RC_DATA = 1 << 5,
    Z_VECTOR_H_VECTOR = 1 << 6,
    RC_CHANNELS = 1 << 7,
    ACC_DATA = 1 << 8,
    MOTOR4_CONTROL = 1 << 9,
    AHRS_DEBUG_INFO = 1 << 10,
    ENCODER_RAW24 = 1 << 11,
    IMU_ANGLES_RAD = 1 << 12,
    SCRIPT_VARS_FLOAT = 1 << 13,
    SCRIPT_VARS_INT16 = 1 << 14,
    SYSTEM_POWER_STATE = 1 << 15,
    FRAME_CAM_RATE = 1 << 16,
    IMU_ANGLES_20 = 1 << 17,
    TARGET_ANGLES_20 = 1 << 18,
    COMM_ERRORS = 1 << 19,
    SYSTEM_STATE = 1 << 20,
    IMU_QUAT = 1 << 21,
    TARGET_QUAT = 1 << 22,
    IMU_TO_FRAME_QUAT = 1 << 23,
    ADC_CH_RAW = 1 << 24,
    SW_LIMITS_DIST = 1 << 25,
    FOLLOW_DIST = 1 << 26,
    EXT_TARGET_LIMIT = 1 << 27,
)

RealtimeDataCustomRequest = Struct(
    "flags" / RealtimeDataCustomFlags,
    "reserved" / Bytes(6),
)

# CMD_REALTIME_DATA_CUSTOM (#88) Response
CalibMode = Enum(Int8ul,
    NONE = 0,
    EXT_GAIN = 1,
    SET_ANGLE_AND_SAVE = 2,
    POLES = 3,
    ACC = 4,
    GYRO = 5,
    ENCODER_OFFSET = 6,
    ENCODER_FLD_OFFSET = 7,
    AUTO_PID = 8,
    BODE_TEST = 9,
    GYRO_TEMP = 10,
    ACC_TEMP = 11,
    MAG = 12,
    SET_ANGLE = 13,
    SYSTEM_IDENT = 14,
    MOTOR_MAG_LINK = 15,
    SEARCH_LIMITS = 16,
    SET_OPERATION_POS = 17,
    IMU_ORIENTATION_CORR = 18,
    TIMELAPSE = 19,
    MOMENTUM = 20,
    MOMENTUM_AUTO = 21,
    COGGING = 22,
    ACC_EXT_REF = 23,
    SAFE_STOP = 24,
    ACC_SPHERE = 25,
    GYRO_AXES_ALIGNMENT = 26,
    EXT_IMU_GYRO = 27,
    EXT_IMU_ALIGN = 28,
    ACC_GYRO_MULTIPOINT = 34,
    ENCODER_LUT = 35,
    WRITE_PARAMS = 36,
    CUR_SENS = 37,
    MOVE_HOME_MOTORS = 38,
    ENCODER_INT = 39,
    VIBRATION_TEST = 40,
    RETRACTED_POSITION = 41,
    RETRACTED_POSITION_RC_CONTROL = 42,
)

PowerState = Enum(Int8ul,
    ON_FROM_BACKUP = -2,
    STARTUP = -1,
    OFF = 0,
    ON = 1,
    OFF_TEMPORARY = 2,
    OFF_PARKING = 3,
    ON_SAFE_STOP = 4,
)

SystemStateFlags = FlagsEnum(Int32ul,
    DEBUG_MODE = 1 << 0,
    IS_FRAME_INVERTED = 1 << 1,
    INIT_STEP1_DONE = 1 << 2,
    INIT_STEP2_DONE = 1 << 3,
    STARTUP_AUTO_ROUTINE_DONE = 1 << 4,
    POWER_MANAGER_ENABLED = 1 << 5,
    SHAKE_GEN_ENABLED = 1 << 8,
    SERVO_MODE_ENABLED = 1 << 9,
    FOLLOW_MODE_ROLL = 1 << 10,
    FOLLOW_MODE_PITCH = 1 << 11,
    FOLLOW_MODE_YAW = 1 << 12,
    TRIPOD_MODE_ENABLED = 1 << 13,
    RETRACTED_POSITION = 1 << 14,
)

SystemPowerState = Struct(
    "motors" / PerAxis(Struct(
        "mot_power" / Int16sl,  # -10000 to 10000, 10000 = 100%
        "mot_current" / Int16ul,  # mA
        "mot_temp" / Int8sl,  # °C
        "mot_flags" / FlagsEnum(Int16ul,
            SW_LIMIT_VIOLATED = 1 << 0,
            CURRENT_EXCEEDS_LIMIT = 1 << 1,
            DRIVER_ENABLED = 1 << 2,
        ),
        "mot_reserved" / Bytes(6),
    )),
    "system_power_state" / PowerState,
    "battery_voltage" / Int16ul,  # Units: 0.01V
    "total_current" / Int16ul,  # mA
    "system_flags" / FlagsEnum(Int16ul,
        SW_LIMIT_VIOLATED = 1 << 0,
        OVERHEAT_WARNING = 1 << 1,
        DRIVER_OTW = 1 << 2,
        DRIVER_FAULT = 1 << 3,
    ),
)

CommErrors = Struct(
    "i2c_errors" / Int16ul,
    "serial_errors" / Int16ul,
    "can_errors" / Int16ul,
    "can_err_flags" / FlagsEnum(Int8ul,
        ERR_WARN_IRQ = 1 << 0,
        ERR_PASSIVE_IRQ = 1 << 1,
        BUS_OFF_IRQ = 1 << 2,
    ),
)

SystemState = Struct(
    "flags" / SystemStateFlags,
    "calib_mode" / CalibMode,
    "reserved" / Bytes(8),
)

AhrsRefSource = Enum(Int8ul,
    REF_NO = 0,
    REF_INTERNAL = 1,
    REF_EXTERNAL = 2,
    REF_TRANSLATE = 3,
)

ExtImuStatus = FlagsEnum(Int8ul,
    STATUS_MASK = 0x07,  # bits 0-2
    STATUS_DISABLED = 0,
    STATUS_NOT_CONNECTED = 1,
    STATUS_UNKNOWN = 2,
    STATUS_ERROR = 3,
    STATUS_BAD = 4,
    STATUS_COARSE = 5,
    STATUS_GOOD = 6,
    STATUS_FINE = 7,
    BAD_MAG = 1 << 6,
    NO_GPS_SIGNAL = 1 << 7,
)

# AHRS debug info structure (26 bytes)
AhrsDebugInfo = Struct(
    "main_imu_ref_src" / Int8ul,  # bits 0-2: attitude, 3-5: heading, 6: sensor connected, 7: processing enabled
    "frame_imu_ref_src" / Int8ul,
    "main_imu_z_ref_err" / Int8ul,  # Units: 0.1°
    "main_imu_h_ref_err" / Int8ul,  # Units: 0.1°
    "frame_imu_z_ref_err" / Int8ul,  # Units: 0.1°
    "frame_imu_h_ref_err" / Int8ul,  # Units: 0.1°
    "ext_imu_status" / ExtImuStatus,
    "ext_imu_packets_received_cnt" / Int16ul,
    "ext_imu_parse_err_cnt" / Int16ul,
    "ext_corr_h_err" / Int8ul,  # Units: 0.1°
    "ext_corr_z_err" / Int8ul,  # Units: 0.1°
    "reserved" / Bytes(13),
)

# MOTOR4_CONTROL structure (8 bytes)
Motor4Control = Struct(
    "ff_speed" / Int16sl,    # Units: 0.06103701895 deg/sec
    "angle_error" / Int16sl, # Units: 0.02197265625 deg
    "pid_out" / Float32l,
)

RealtimeDataCustomResponse = Struct(
    "timestamp_ms" / Int16ul,

    # Conditional fields based on request flags
    "imu_angles" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.IMU_ANGLES,
        PerAxis(Int16sl)),
    "target_angles" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.TARGET_ANGLES,
        PerAxis(Int16sl)),
    "target_speed" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.TARGET_SPEED,
        PerAxis(Int16sl)),
    "frame_cam_angle" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.FRAME_CAM_ANGLE,
        PerAxis(Int16sl)),
    "gyro_data" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.GYRO_DATA,
        PerAxis(Int16sl)),
    "rc_data" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.RC_DATA,
        Array(6, Int16sl)),
    "z_vector" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.Z_VECTOR_H_VECTOR,
        PerAxis(Float32l)),
    "h_vector" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.Z_VECTOR_H_VECTOR,
        PerAxis(Float32l)),
    "rc_channels" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.RC_CHANNELS,
        Array(18, Int16sl)),
    "acc_data" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.ACC_DATA,
        PerAxis(Int16sl)),
    "motor4_control" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.MOTOR4_CONTROL,
        Motor4Control),
    "ahrs_debug_info" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.AHRS_DEBUG_INFO,
        AhrsDebugInfo),
    "encoder_raw24" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.ENCODER_RAW24,
        Bytes(9)),  # 3x3 bytes
    "imu_angles_rad" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.IMU_ANGLES_RAD,
        PerAxis(Float32l)),
    "script_vars_float" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.SCRIPT_VARS_FLOAT,
        Array(10, Float32l)),
    "script_vars_int16" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.SCRIPT_VARS_INT16,
        Array(10, Int16sl)),
    "system_power_state" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.SYSTEM_POWER_STATE,
        SystemPowerState),
    "frame_cam_rate" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.FRAME_CAM_RATE,
        PerAxis(Int16sl)),
    "imu_angles_20" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.IMU_ANGLES_20,
        PerAxis(Int32sl)),
    "target_angles_20" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.TARGET_ANGLES_20,
        PerAxis(Int32sl)),
    "comm_errors" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.COMM_ERRORS,
        CommErrors),
    "system_state" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.SYSTEM_STATE,
        SystemState),
    "imu_quat" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.IMU_QUAT,
        Bytes(8)),  # Compressed quat
    "target_quat" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.TARGET_QUAT,
        Bytes(8)),
    "imu_to_frame_quat" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.IMU_TO_FRAME_QUAT,
        Bytes(8)),
    "adc_ch_raw" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.ADC_CH_RAW,
        Array(4, Int16ul)),
    "sw_limits_dist" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.SW_LIMITS_DIST,
        PerAxis(Array(2, Int16sl))),
    "follow_dist" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.FOLLOW_DIST,
        PerAxis(Int16sl)),
    "ext_target_limit_min" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.EXT_TARGET_LIMIT,
        PerAxis(Int32sl)),
    "ext_target_limit_max" / If(lambda ctx: ctx._.request.flags & RealtimeDataCustomFlags.EXT_TARGET_LIMIT,
        PerAxis(Int32sl)),
)

# CMD_REALTIME_DATA (#68), CMD_REALTIME_DATA_3 (#23)
RealtimeData3Request = Pass # No parameters

# CMD_REALTIME_DATA_3 (#23) Response
SystemError = FlagsEnum(Int16ul,
    NO_SENSOR = 1 << 0,
    CALIB_ACC = 1 << 1,
    SET_POWER = 1 << 2,
    CALIB_POLES = 1 << 3,
    PROTECTION = 1 << 4,
    SERIAL = 1 << 5,
    LOW_BAT1 = 1 << 6,
    LOW_BAT2 = 1 << 7,
    GUI_VERSION = 1 << 8,
    MISS_STEPS = 1 << 9,
    SYSTEM = 1 << 10,
    EMERGENCY_STOP = 1 << 11,
)

RtDataFlags = FlagsEnum(Int8ul,
    MOTORS_ON = 1 << 0,
)

RealtimeData3Response = Struct(
    "imu_data" / PerAxis(Struct(
        "acc" / Int16sl,  # Units: 1/512 G
        "gyro" / Int16sl, # Units: 0.06103701895 deg/sec
    )),

    "serial_err_cnt" / Int16ul,
    "system_error" / SystemError,
    "system_sub_error" / EmergencyStopCode,
    "reserved" / Bytes(3),

    "rc_roll" / Int16sl,
    "rc_pitch" / Int16sl,
    "rc_yaw" / Int16sl,
    "rc_cmd" / Int16sl,
    "ext_fc_roll" / Int16sl,
    "ext_fc_pitch" / Int16sl,

    "imu_angle" / PerAxis(Int16sl),  # Units: 0.02197265625 deg
    "frame_imu_angle" / PerAxis(Int16sl),
    "target_angle" / PerAxis(Int16sl),

    "cycle_time" / Int16ul,  # microseconds
    "i2c_error_count" / Int16ul,
    "error_code" / Int8ul,  # Deprecated
    "bat_level" / Int16ul,  # Units: 0.01V
    "rt_data_flags" / RtDataFlags,
    "cur_imu" / ImuType,
    "cur_profile" / Int8ul,
    "motor_power" / PerAxis(Int8ul),
)

# CMD_REALTIME_DATA_4 (#25)
RealtimeData4Request = Pass # No parameters

# CMD_REALTIME_DATA_4 (#25) Response
RealtimeData4Response = Struct(
    # Includes all fields from CMD_REALTIME_DATA_3
    "realtime_data_3" / RealtimeData3Response,

    # Extra fields
    "frame_cam_angle" / PerAxis(Int16sl),  # Units: 0.02197265625 deg
    "reserved2" / Bytes(1),
    "balance_error" / PerAxis(Int16sl),
    "current" / Int16ul,  # mA
    "mag_data" / PerAxis(Int16sl),
    "imu_temperature" / Int8sl,  # °C
    "frame_imu_temperature" / Int8sl,  # °C
    "imu_g_err" / Int8ul,  # Units: 0.1 deg
    "imu_h_err" / Int8ul,  # Units: 0.1 deg
    "motor_out" / PerAxis(Int16sl),  # -10000 to 10000
    "calib_mode" / CalibMode,
    "can_imu_ext_sens_err" / Int8ul,
    "actual_angle" / PerAxis(Int16sl),  # frw.ver. 2.72b0+
    "system_state_flags" / SystemStateFlags,  # frw.ver. 2.73.0
    "reserved3" / Bytes(18),
)

# Data stream sync mode
DataStreamSync = Enum(Int8ul,
    NO_SYNC = 0,
    IMU_ATTITUDE = 1,
)

# CMD_DATA_STREAM_INTERVAL (#85)
DataStreamIntervalRequest = Struct(
    "cmd_id" / Int8ul,
    "interval_ms" / Int16ul,  # 0 to unregister, 1=each cycle (0.8ms)
    "config" / Bytes(8),  # Command-specific configuration
    "sync_to_data" / DataStreamSync,  # frw.ver 2.70b1+
    "reserved" / Bytes(9),
)

# CMD_READ_RC_INPUTS (#100)
ReadRcInputsRequest = Struct(
    "cfg_flags" / FlagsEnum(Int16ul,
        INIT_IF_NEEDED = 1 << 0,
    ),
    "rc_src" / GreedyRange(SourcePinId),
)

# CMD_READ_RC_INPUTS (#100) Response
ReadRcInputsResponse = Struct(
    "rc_val" / GreedyRange(Int16sl),  # -16384 to 16384, -32768 for undefined
)

# CMD_GET_ANGLES (#73), CMD_GET_ANGLES_EXT (#61)
GetAnglesRequest = Pass    # No parameters
GetAnglesExtRequest = Pass # No parameters

# CMD_GET_ANGLES (#73) Response
GetAnglesResponse = PerAxis(Struct(
    "imu_angle" / Int16sl,    # Units: 0.02197265625 deg
    "target_angle" / Int16sl,
    "target_speed" / Int16sl, # Units: 0.1220740379 deg/sec
))

# CMD_GET_ANGLES_EXT (#61) Response
GetAnglesExtResponse = PerAxis(Struct(
    "imu_angle" / Int16sl,       # Units: 0.02197265625 deg
    "target_angle" / Int16sl,
    "frame_cam_angle" / Int32sl, # Units: 0.02197265625 deg
    "reserved" / Bytes(10),
))

# CMD_SELECT_IMU_3 (#24)
SelectImu3Request = Struct(
    "imu_type" / ImuType,
)

# CMD_DEBUG_VARS_INFO_3 (#253)
DebugVarsInfo3Request = Struct(
    "start_idx" / Default(Int8ul, 0),  # frw.ver. 2.72b0+, optional
)

# CMD_DEBUG_VARS_INFO_3 (#253) Response
DebugVarFlags = FlagsEnum(Int8ul,
    UINT8 = 1,
    INT8 = 2,
    UINT16 = 3,
    INT16 = 4,
    UINT32 = 5,
    INT32 = 6,
    FLOAT = 7,

    TYPE_MASK = 0x0f,  # bits 0-3
    ROLL = 0x10,
    PITCH = 0x20,
    YAW = 0x30,  # bits 4-5
    ANGLE14 = 0x40,
)

DebugVarsInfo3Response = Struct(
    "debug_vars_num" / Int8ul,
    "vars" / Array(this.debug_vars_num, Struct( # TODO: min(debug_vars_num, limit)
        "name_length" / Int8ul,
        "name" / PaddedString(this.name_length, "ascii"),
        "var_type" / DebugVarFlags,
        "reserved" / Bytes(2),
    )),
)

# CMD_DEBUG_VARS_3 (#254)
DebugVars3Request = Struct(
    "selected_mask" / GreedyRange(Int32ul),  # Optional (frw.ver. 2.72b0+)
)

# CMD_DEBUG_VARS_3 (#254) Response
# Variable structure - depends on variable types from INFO command
DebugVars3Response = Struct(
    "selected_mask" / Optional(GreedyRange(Int32ul)),  # Optional, frw.ver. 2.72b0+
    "var_values" / GreedyBytes,  # Variable size/type data
)

# CMD_CALIB_INFO (#49)
CalibInfoRequest = Struct(
    "imu_type" / ImuType,
    "reserved" / Bytes(11),
)

# CMD_CALIB_INFO (#49) Response
CalibInfoResponse = Struct(
    "progress" / Int8ul,  # 0-100%
    "imu_type" / ImuType,
    "acc_data" / PerAxis(Int16sl),  # Units: 1/512 G
    "gyro_abs_val" / Int16ul,
    "acc_cur_axis" / Int8ul,  # 0-2
    "acc_limits_info" / Int8ul,  # Bit set for calibrated limits
    "imu_temp_cels" / Int8sl,  # °C
    "temp_calib_gyro_enabled" / Int8ul,
    "temp_calib_gyro_t_min_cels" / Int8sl,
    "temp_calib_gyro_t_max_cels" / Int8sl,
    "temp_calib_acc_enabled" / Int8ul,
    "temp_calib_acc_slot_num" / Array(6, Int8ul),
    "temp_calib_acc_t_min_cels" / Int8sl,
    "temp_calib_acc_t_max_cels" / Int8sl,
    "h_err_length" / Int8ul,
    "reserved" / Bytes(7),
)

# CMD_SCRIPT_DEBUG (#58) Notification
ScriptDebugNotification = Struct(
    "cmd_count" / Int16ul,
    "err_code" / ErrorCode,
)

# CMD_READ_STATE_VARS (#111)
ReadStateVarsRequest = Pass # No parameters

# CMD_READ_STATE_VARS (#111) Response
ReadStateVarsResponse = Struct(
    "step_signal_val" / Array(6, Int8ul),
    "sub_error" / EmergencyStopCode,
    "max_acc" / Int8ul,  # Units: 1/16G
    "work_time" / Int32ul,  # seconds
    "startup_cnt" / Int16ul,
    "max_current" / Int16ul,  # mA
    "imu_temp_min" / Int8ul,  # °C
    "imu_temp_max" / Int8ul,
    "mcu_temp_min" / Int8ul,
    "mcu_temp_max" / Int8ul,
    "shock_cnt" / Array(4, Int8ul),
    "energy_time" / Int32ul,  # seconds
    "energy" / Float32l,  # Watt*hour
    "avg_current_time" / Int32ul,  # seconds
    "avg_current" / Float32l,  # A
    "reserved" / Bytes(152),
)

# CMD_WRITE_STATE_VARS (#112)
ReadStateVarsRequest = LazyBound(lambda: ReadStateVarsResponse)

# CMD_SET_DEBUG_PORT (#249)
SetDebugPortRequest = Struct(
    "action" / Enum(Int8ul,
        STOP = 0,
        START = 1,
    ),
    "cmd_filter" / FlagsEnum(Int32ul,  # "plus" version only
        CMD_REALTIME_DATA_3 = 1 << 0,
        CMD_REALTIME_DATA_4 = 1 << 1,
        CMD_REALTIME_DATA_CUSTOM = 1 << 2,
        CMD_DEBUG_VARS_3 = 1 << 3,
        CMD_MAVLINK_DEBUG = 1 << 4,
        CMD_GET_ANGLES = 1 << 5,
        CMD_GET_ANGLES_EXT = 1 << 6,
        CMD_BODE_TEST_DATA = 1 << 7,
        CMD_HELPER_DATA = 1 << 8,
        CMD_AHRS_HELPER = 1 << 9,
        CMD_GYRO_CORRECTION = 1 << 10,
        CMD_CONTROL = 1 << 11,
        CMD_SET_ADJ_VARS_VAL = 1 << 12,
        CMD_API_VIRT_CH_CONTROL = 1 << 13,
        CMD_API_VIRT_CH_HIGH_RES = 1 << 14,
    ),
    "reserved" / Bytes(11),
)

# CMD_SET_DEBUG_PORT (#249) Response (more like notification)
SetDebugPortResponse = Struct(
    "time_ms" / Int16ul,
    "port_and_dir" / Int8ul,  # bits 0-6: port, bit 7: direction
    "cmd_id" / CmdId,
    "payload" / GreedyBytes, # TODO: use a Switch()
)

# CMD_CAN_DRV_TELEMETRY (#127) Notification
CanDrvPinState = FlagsEnum(Int8ul,
    LIMIT = 1 << 0,
    INDEX = 1 << 1,
    EMERGENCY = 1 << 2,
    AUX1 = 1 << 3,
    AUX2 = 1 << 4,
    AUX3 = 1 << 5,
    HALF_BRIDGE_ENABLE = 1 << 6,
    ENCODER_SPI_CS = 1 << 7,
)

CanDrvTelemetryFlags = FlagsEnum(Int32ul,
    CAN_BUS_FLAGS = 1 << 0,
    CAN_BUS_ERR_CNT = 1 << 1,
    TEMP_BOARD = 1 << 2,
    TEMP_MOTOR = 1 << 3,
    AVG_CURRENT = 1 << 4,
    BUS_VOLTAGE = 1 << 5,
    PIN_STATE = 1 << 6,
    PHASE_CURRENT = 1 << 7,
    ENCODER_ERROR_CNT = 1 << 8,
    IQ_ID_CURRENT = 1 << 9,
    MOTOR_SPEED = 1 << 10,
    PID_POS_ERR = 1 << 11,
    PID_SPEED_ERR = 1 << 12,
    IQ_REF = 1 << 13,
    PID_CURRENT_ERR = 1 << 14,
)

CanDrvTelemetryNotification = Struct(
    "drv_id" / Int8ul,  # 0-6
    "timestamp_ms" / Int16ul,
    # DATA field depends on flags in request
    "can_bus_flags" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.CAN_BUS_FLAGS,
        Int8ul),
    "can_bus_err_cnt" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.CAN_BUS_ERR_CNT,
        Int16ul),
    "temp_board" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.TEMP_BOARD,
        Int8sl),  # °C
    "temp_motor" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.TEMP_MOTOR,
        Int8sl),  # °C
    "avg_current" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.AVG_CURRENT,
        Int16ul),  # mA
    "bus_voltage" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.BUS_VOLTAGE,
        Int16ul),  # mV
    "pin_state" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.PIN_STATE,
        CanDrvPinState),
    "phase_current" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.PHASE_CURRENT,
        PerAxis(Int16sl)),  # mA
    "encoder_error_cnt" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.ENCODER_ERROR_CNT,
        Int16ul),
    "iq_current" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.IQ_ID_CURRENT,
        Int16sl),  # mA
    "id_current" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.IQ_ID_CURRENT,
        Int16sl),  # mA
    "motor_speed" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.MOTOR_SPEED,
        Int16sl),  # RPM
    "pid_pos_err" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.PID_POS_ERR,
        Int16sl),
    "pid_speed_err" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.PID_SPEED_ERR,
        Int16sl),
    "iq_ref" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.IQ_REF,
        Int16sl),  # ±32768 = ±100%
    "pid_current_err" / If(lambda ctx: ctx._.request.config[:4] and
        Int32ul.parse(ctx._.request.config[:4]) & CanDrvTelemetryFlags.PID_CURRENT_ERR,
        Int16sl),
)

# CMD_EXT_MOTORS_STATE (#131)
ExtMotorStateDataSet = FlagsEnum(Int32ul,
    CONTROL_MODE = 1 << 0,
    TORQUE = 1 << 1,
    TORQUE_SETPOINT = 1 << 2,
    SPEED16 = 1 << 3,
    SPEED16_SETPOINT = 1 << 4,
    SPEED32 = 1 << 5,
    SPEED32_SETPOINT = 1 << 6,
    ANGLE16 = 1 << 7,
    ANGLE16_SETPOINT = 1 << 8,
    ANGLE32 = 1 << 9,
    ANGLE32_SETPOINT = 1 << 10,
    STATE_FLAGS = 1 << 11,
    MAX_SPEED = 1 << 12,
    MAX_ACCELERATION = 1 << 13,
    JERK_SLOPE = 1 << 14,
    MAX_TORQUE = 1 << 15,
    CURRENT = 1 << 16,
    BATTERY_VOLTAGE = 1 << 17,
    MOTOR_TEMPERATURE = 1 << 18,
    DRIVER_TEMPERATURE = 1 << 19,
)

ExtMotorsStateRequest = Struct(
    "for_motors" / FlagsEnum(Int8ul,
        MOTOR1 = 1 << 0,
        MOTOR2 = 1 << 1,
        MOTOR3 = 1 << 2,
        MOTOR4 = 1 << 3,
        MOTOR5 = 1 << 4,
        MOTOR6 = 1 << 5,
        MOTOR7 = 1 << 6,
    ),
    "data_set" / ExtMotorStateDataSet,
)

# CMD_EXT_MOTORS_STATE (#131) Response
ExtMotorControlMode = Enum(Int8ul,
    POSITION = 0,
    SPEED = 1,
    TORQUE = 2,
)

ExtMotorStateFlags = FlagsEnum(Int16ul,
    MOTOR_STATE_MASK = 0x03,  # bits 0-1
    OFF_BRAKE = 0,
    ON = 1,
    OFF_FLOATING = 2,
    OFF_SAFE_STOP = 3,
    INDEX_PIN_STATE = 1 << 2,
)

ExtMotorsStateResponse = Struct(
    "motor_id" / Int8ul,  # 0-6
    "data_set" / ExtMotorStateDataSet,
    # DATA field depends on data_set flags in this response
    "control_mode" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.CONTROL_MODE,
        ExtMotorControlMode),
    "torque" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.TORQUE,
        Int16sl),
    "torque_setpoint" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.TORQUE_SETPOINT,
        Int16sl),
    "speed16" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.SPEED16,
        Int16sl),  # Units: 0.1220740379 deg/sec
    "speed16_setpoint" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.SPEED16_SETPOINT,
        Int16sl),
    "speed32" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.SPEED32,
        Int32sl),  # Units: micro-radians/sec
    "speed32_setpoint" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.SPEED32_SETPOINT,
        Int32sl),
    "angle16" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.ANGLE16,
        Int16sl),  # Units: 0.02197265625 deg
    "angle16_setpoint" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.ANGLE16_SETPOINT,
        Int16sl),
    "angle32" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.ANGLE32,
        Int32sl),  # Units: 0.00034332275390625 deg
    "angle32_setpoint" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.ANGLE32_SETPOINT,
        Int32sl),
    "state_flags" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.STATE_FLAGS,
        ExtMotorStateFlags),
    "max_speed" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.MAX_SPEED,
        Int16ul),
    "max_acceleration" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.MAX_ACCELERATION,
        Int16ul),
    "jerk_slope" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.JERK_SLOPE,
        Int16ul),
    "max_torque" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.MAX_TORQUE,
        Int16ul),
    "current" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.CURRENT,
        Int16ul),  # mA
    "battery_voltage" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.BATTERY_VOLTAGE,
        Int16ul),  # mV
    "motor_temperature" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.MOTOR_TEMPERATURE,
        Int8sl),  # °C
    "driver_temperature" / If(lambda ctx: ctx.data_set & ExtMotorStateDataSet.DRIVER_TEMPERATURE,
        Int8sl),  # °C
)

# CMD_CONTROL_QUAT_STATUS (#141)
ControlQuatStatusDataSet = FlagsEnum(Int32ul,
    MODE = 1 << 0,
    TARGET_ATTITUDE = 1 << 1,
    SETPOINT_ATTITUDE = 1 << 2,
    ACTUAL_ATTITUDE = 1 << 3,
    TARGET_SPEED = 1 << 4,
    SETPOINT_SPEED = 1 << 5,
    ACTUAL_SPEED = 1 << 6,
    TARGET_ATTITUDE_PACKED = 1 << 7,
    SETPOINT_ATTITUDE_PACKED = 1 << 8,
    ACTUAL_ATTITUDE_PACKED = 1 << 9,
)

ControlQuatStatusRequest = Struct(
    "data_set" / ControlQuatStatusDataSet,
)

# CMD_CONTROL_QUAT_STATUS (#141) Response
ControlQuatStatusResponse = Struct(
    "data_set" / ControlQuatStatusDataSet,
    # DATA field depends on data_set flags in this response
    "mode" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.MODE,
        Int8ul),
    "flags" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.MODE,
        Int16ul),
    "target_attitude" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.TARGET_ATTITUDE,
        Array(4, Float32l)),  # [w,x,y,z]
    "setpoint_attitude" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.SETPOINT_ATTITUDE,
        Array(4, Float32l)),
    "actual_attitude" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.ACTUAL_ATTITUDE,
        Array(4, Float32l)),
    "target_speed" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.TARGET_SPEED,
        PerAxis(Int16sl)),  # Units: 0.06103701895 deg/sec
    "setpoint_speed" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.SETPOINT_SPEED,
        PerAxis(Int16sl)),
    "actual_speed" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.ACTUAL_SPEED,
        PerAxis(Int16sl)),
    "target_attitude_packed" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.TARGET_ATTITUDE_PACKED,
        Bytes(8)),  # Compressed quaternion
    "setpoint_attitude_packed" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.SETPOINT_ATTITUDE_PACKED,
        Bytes(8)),
    "actual_attitude_packed" / If(lambda ctx: ctx.data_set & ControlQuatStatusDataSet.ACTUAL_ATTITUDE_PACKED,
        Bytes(8)),
)

#
# Section: Run-time gimbal parameters
#

# Adjustable variable IDs enum (from Appendix B)
AdjustableVarId = Enum(Int8ul,
    # PID parameters (ROLL=0, PITCH=1, YAW=2)
    P_ROLL = 0,
    P_PITCH = 1,
    P_YAW = 2,
    I_ROLL = 3,
    I_PITCH = 4,
    I_YAW = 5,
    D_ROLL = 6,
    D_PITCH = 7,
    D_YAW = 8,
    POWER_ROLL = 9,
    POWER_PITCH = 10,
    POWER_YAW = 11,

    # Control parameters
    ACC_LIMITER = 12,

    # Follow speed per axis
    FOLLOW_SPEED_ROLL = 13,
    FOLLOW_SPEED_PITCH = 14,
    FOLLOW_SPEED_YAW = 15,

    # Follow LPF per axis
    FOLLOW_LPF_ROLL = 16,
    FOLLOW_LPF_PITCH = 17,
    FOLLOW_LPF_YAW = 18,

    # RC speed per axis
    RC_SPEED_ROLL = 19,
    RC_SPEED_PITCH = 20,
    RC_SPEED_YAW = 21,

    # RC LPF per axis
    RC_LPF_ROLL = 22,
    RC_LPF_PITCH = 23,
    RC_LPF_YAW = 24,

    # RC trim per axis
    RC_TRIM_ROLL = 25,
    RC_TRIM_PITCH = 26,
    RC_TRIM_YAW = 27,

    # RC configuration
    RC_DEADBAND = 28,
    RC_EXPO_RATE = 29,

    # Follow mode
    FOLLOW_PITCH = 30,
    FOLLOW_YAW_PITCH = 31,
    FOLLOW_DEADBAND = 32,
    FOLLOW_EXPO_RATE = 33,
    FOLLOW_ROLL_MIX_START = 34,
    FOLLOW_ROLL_MIX_RANGE = 35,

    # Gyro parameters
    GYRO_TRUST = 36,
    FRAME_HEADING_ANGLE = 37,
    GYRO_HEADING_CORRECTION = 38,

    # Acceleration limiter per axis
    ACC_LIMITER_ROLL = 39,
    ACC_LIMITER_PITCH = 40,
    ACC_LIMITER_YAW = 41,

    # PID gain per axis
    PID_GAIN_ROLL = 42,
    PID_GAIN_PITCH = 43,
    PID_GAIN_YAW = 44,

    # LPF frequency per axis
    LPF_FREQ_ROLL = 45,
    LPF_FREQ_PITCH = 46,
    LPF_FREQ_YAW = 47,

    # Timelapse
    TIMELAPSE_TIME = 48,

    # MAVLink
    MAV_CTRL_MODE = 49,

    # Heading correction
    H_CORR_FACTOR = 50,

    # Software limits per axis (encoder firmware)
    SW_LIM_MIN_ROLL = 51,
    SW_LIM_MAX_ROLL = 52,
    SW_LIM_MIN_PITCH = 53,
    SW_LIM_MAX_PITCH = 54,
    SW_LIM_MIN_YAW = 55,
    SW_LIM_MAX_YAW = 56,

    # Follow range per axis
    FOLLOW_RANGE_ROLL = 57,
    FOLLOW_RANGE_PITCH = 58,
    FOLLOW_RANGE_YAW = 59,

    # Auto PID
    AUTO_PID_TARGET = 60,

    # RC mode per axis
    RC_MODE_ROLL = 61,
    RC_MODE_PITCH = 62,
    RC_MODE_YAW = 63,

    # Euler order
    EULER_ORDER = 64,

    # Follow inside deadband
    FOLLOW_IN_DBAND = 65,

    # RC limits per axis
    RC_LIMIT_MIN_ROLL = 66,
    RC_LIMIT_MAX_ROLL = 67,
    RC_LIMIT_MIN_PITCH = 68,
    RC_LIMIT_MAX_PITCH = 69,
    RC_LIMIT_MIN_YAW = 70,
    RC_LIMIT_MAX_YAW = 71,

    # RC deadband and expo for PITCH/YAW
    RC_DEADBAND_PITCH = 72,
    RC_DEADBAND_YAW = 73,
    RC_EXPO_RATE_PITCH = 74,
    RC_EXPO_RATE_YAW = 75,

    # Shake generator parameters (frw. ver. 2.73.0)
    SHAKE_AMP_ROLL = 76,
    SHAKE_AMP_TILT = 77,
    SHAKE_AMP_PAN = 78,
    SHAKE_FREQ = 79,
    SHAKE_FREQ_RANGE = 80,
    SHAKE_PAUSE_PERIOD = 81,
    SHAKE_PAUSE_BALANCE = 82,
    SHAKE_PAUSE_RANDOMNESS = 83,
    SHAKE_RESONANCE_GAIN_ROLL = 84,
    SHAKE_RESONANCE_GAIN_TILT = 85,
    SHAKE_RESONANCE_GAIN_PAN = 86,
    SHAKE_FREQ_SHIFT_ROLL = 87,
    SHAKE_FREQ_SHIFT_TILT = 88,
    SHAKE_FREQ_SHIFT_PAN = 89,
    SHAKE_MASTER_GAIN = 90,

    # Outer P per axis (frw. ver. 2.73.7)
    OUTER_P_ROLL = 91,
    OUTER_P_PITCH = 92,
    OUTER_P_YAW = 93,

    # D-term LPF per axis (frw. ver. 2.73.7)
    D_LPF_FREQ_ROLL = 94,
    D_LPF_FREQ_PITCH = 95,
    D_LPF_FREQ_YAW = 96,

    # IMU angle correction (frw. ver. 2.73.8)
    IMU_ANGLE_CORR_ROLL = 97,
    IMU_ANGLE_CORR_PITCH = 98,

    # Software limit width per axis (frw. ver. 2.73.8)
    SW_LIM_WIDTH_ROLL = 99,
    SW_LIM_WIDTH_PITCH = 100,
    SW_LIM_WIDTH_YAW = 101,
)

# CMD_ADJ_VARS_INFO (#132)
AdjVarsInfoRequest = Struct(
    "start_id" / Default(AdjustableVarId, 0),  # Optional, default 0
)

# CMD_ADJ_VARS_INFO (#132) Response
AdjVarInfo = Struct(
    "var_id" / AdjustableVarId,
    "name_length" / Int8ul,  # 0-16
    "name" / PaddedString(this.name_length, "ascii"),
    "range_min" / Int16sl,
    "range_max" / Int16sl,
    "value" / Int32sl,
)

AdjVarsInfoResponse = Struct(
    "total_number" / Int8ul,
    "vars" / GreedyRange(AdjVarInfo), # TODO: total_number as length?
)

# CMD_SET_ADJ_VARS_VAL (#31) - Set values as 32-bit integers
SetAdjVarsValRequest = Struct(
    "num_params" / Int8ul,  # 1-40
    "params" / Array(this.num_params, Struct(
        "id" / AdjustableVarId,
        "value" / Int32sl,
    )),
)

# CMD_SET_ADJ_VARS_VAL (#31) Response - Trick: sent in response to GET request
SetAdjVarsValResponse = LazyBound(lambda: SetAdjVarsValRequest)

# CMD_SET_ADJ_VARS_F (#134) - Set values as floating points (frw. ver. 2.73.4)
SetAdjVarsValFRequest = Struct(
    "num_params" / Int8ul,  # 1-40
    "params" / Array(this.num_params, Struct(
        "id" / AdjustableVarId,
        "value" / Float32l,
    )),
)

# CMD_SET_ADJ_VARS_F (#134) Response - Trick: sent in response to GET request
SetAdjVarsValFResponse = LazyBound(lambda: SetAdjVarsValFRequest)

# CMD_GET_ADJ_VARS_VAL (#64) - Query values as integers
GetAdjVarsValRequest = Struct(
    "num_params" / Int8ul,  # 1-40
    "param_ids" / Array(this.num_params, AdjustableVarId),
)

# CMD_GET_ADJ_VARS_VAL_F (#135) - Query values as floating points (frw. ver. 2.73.4)
GetAdjVarsValFRequest = Struct(
    "num_params" / Int8ul,  # 1-40
    "param_ids" / Array(this.num_params, AdjustableVarId),
)

# CMD_READ_ADJ_VARS_CFG (#43)
ReadAdjVarsCfgRequest = Pass # No parameters

# CMD_READ_ADJ_VARS_CFG (#43) Response
ReadAdjVarsCfgResponse = Struct(
    # 10 trigger slots
    "triggers" / Array(10, Struct(
        "src_ch" / SourcePinId,
        "actions" / Array(5, MenuCommand),  # 5 action IDs for different RC levels
    )),

    # 15 analog slots
    "analogs" / Array(15, Struct(
        "src_ch" / SourcePinId,
        "var_id" / FlagsEnum(AdjustableVarId,
            VAR_ID_MASK = 0x7F,  # bits 0-6
            MULTIPLIER_MODE = 0x80,  # bit 7: if set, value is a multiplier
        ),
        "min_val" / Int8ul,
        "max_val" / Int8ul,
    )),

    "reserved" / Bytes(8),
)

# CMD_WRITE_ADJ_VARS_CFG (#44)
WriteAdjVarsCfgRequest = LazyBound(lambda: ReadAdjVarsCfgResponse)

# CMD_SAVE_PARAMS_3 (#32)
SaveParams3Request = Struct(
    "adj_var_ids" / GreedyRange(AdjustableVarId),  # Optional (frw.ver. 2.68b9+), if empty save all
)

# CMD_ADJ_VARS_STATE (#46) - prior to frw. ver. 2.62b5
AdjVarsStateRequestOld = Struct(
    "trigger_slot" / Int8ul,  # 0-9
    "analog_slot" / Int8ul,  # 0-14
)

# CMD_ADJ_VARS_STATE (#46) - frw. ver. 2.62b5+
AdjVarsStateRequest = Struct(
    "trigger_slot" / Int8ul,  # 0-9
    "analog_src_id" / Int16ul,
    "analog_var_id" / AdjustableVarId,
    "lut_src_id" / Int16ul,
    "lut_var_id" / AdjustableVarId,
)

# CMD_ADJ_VARS_STATE (#46) Response (frw. ver. 2.62b5+)
AdjVarsStateResponse = Struct(
    "trigger_rc_data" / Int16sl,  # -16384 to 16384
    "trigger_action" / MenuCommand,
    "analog_src_value" / Int16sl,  # -16384 to 16384
    "analog_var_value" / Float32l,
    "lut_src_value" / Int16sl,  # -16384 to 16384
    "lut_var_value" / Float32l,
)

#
# Section: IMU correction and diagnostic
#

# CMD_HELPER_DATA (#72) request
HelperDataFlags = FlagsEnum(Int8ul,
    GROUND_YAW_ROTATED = 1,
    GROUND_END = 2,
    FRAME = 3,

    COORD_SYS_MASK = 7, # bits 0-2
    EULER_ORDER_ROLL_PITCH_YAW = 1 << 6,
    USE_FRAME_HEADING = 1 << 7,
)

HelperDataRequest = Struct(
    "frame_acc" / PerAxis(Int16sl),  # [X,Y,Z], Units: 1g/512
    "frame_angle_roll" / Int16sl,  # Units: 0.02197265625 deg
    "frame_angle_pitch" / Int16sl,
    "extended" / Optional(Struct( # Extended format (frw. ver. 2.60+)
        "flags" / HelperDataFlags,
        "frame_speed" / PerAxis(Int16sl),  # [X,Y,Z], Units: 0.06103701895 deg/sec
        "frame_heading" / Int16sl,  # Units: 0.02197265625 deg, 32767=stop correction
        "reserved" / Bytes(1),
    )),
)

# CMD_AHRS_HELPER (#56) - 2-byte MODE (frw.ver. 2.69b5+)
AhrsHelperMode = FlagsEnum(Int16ul,
    GET = 0,
    SET = 1,
    LOCATION_CAMERA = 0 << 1,
    LOCATION_FRAME = 1 << 1,
    USE_AS_REFERENCE = 1 << 2,
    TRANSLATE_FROM_TO = 1 << 3,
    Z_ONLY = 1 << 4,
    H_ONLY = 1 << 5,
    TRANSLATE_Z_ONLY = 1 << 6,  # frw.ver. 2.69b5+
    TRANSLATE_H_ONLY = 1 << 7,  # frw.ver. 2.69b5+
    FRAME_POS_SAME_AS_FRAME_IMU = 0 << 8,  # bits 8-9 (frw.ver. 2.69b5+)
    FRAME_POS_ON_FRAME = 1 << 8,
    FRAME_POS_BELOW_OUTER = 2 << 8,
    DISABLE_CORRECTION = 1 << 10,  # frw.ver. 2.70b1+
)

AhrsHelperRequest = Struct(
    "mode" / Switch(lambda ctx: len(ctx._io.getvalue()), { 26: AhrsHelperMode, 27: Int8ul }, default=AhrsHelperMode), # post vs pre frw.ver. 2.69b5
    "z_vect" / PerAxis(Float32l),  # Unit vector pointing down (END coords)
    "h_vect" / PerAxis(Float32l),  # Unit vector pointing North (END coords)
)

# CMD_AHRS_HELPER (#56) Response
AhrsHelperResponse = Struct(
    "z_vect" / PerAxis(Float32l),  # Unit vector pointing down (END coords)
    "h_vect" / PerAxis(Float32l),  # Unit vector pointing North (END coords)
)

# CMD_GYRO_CORRECTION (#75)
GyroCorrectionRequest = Struct(
    "imu_type" / ImuType,
    "gyro_zero_corr" / PerAxis(Int16sl),  # [X,Y,Z], Units: 0.001 gyro sensor unit
    "gyro_zero_heading_corr" / Int16sl,  # Global Z axis, Units: 0.001 gyro sensor unit
)

# CMD_EXT_IMU_DEBUG_INFO (#106) Response
ExtImuDebugInfoResponse = Struct(
    "ahrs_debug_info" / AhrsDebugInfo,
    "dcm" / Array(9, Float32l),  # 3x3 rotation matrix (flattened row-major)
    "acc_body" / PerAxis(Float32l),  # Linear acceleration in sensor's local coords
)

# Helper functions for parsing reference source encoding
def parse_ref_src_attitude(byte_val):
    """Extract attitude reference source from bits 0-2"""
    return byte_val & 0x07

def parse_ref_src_heading(byte_val):
    """Extract heading reference source from bits 3-5"""
    return (byte_val >> 3) & 0x07

def parse_ref_src_sensor_connected(byte_val):
    """Extract sensor connected flag from bit 6"""
    return bool(byte_val & 0x40)

def parse_ref_src_processing_enabled(byte_val):
    """Extract processing enabled flag from bit 7"""
    return bool(byte_val & 0x80)

#
# Section: Controlling gimbal movements
#

# CMD_CONTROL (#67)
ControlMode = FlagsEnum(Int8ul,
    # Mode (bits 0-3)
    MODE_NO_CONTROL = 0,
    MODE_SPEED = 1,
    MODE_ANGLE = 2,
    MODE_SPEED_ANGLE = 3,
    MODE_RC = 4,
    MODE_ANGLE_REL_FRAME = 5,
    MODE_RC_HIGH_RES = 6,
    MODE_IGNORE = 7,
    MODE_ANGLE_SHORTEST = 8,

    # Flags (bits 4-7)
    CONTROL_FLAG_MIX_FOLLOW = 1 << 4,
    CONTROL_FLAG_TARGET_PRECISE = 1 << 5,
    CONTROL_FLAG_AUTO_TASK = 1 << 6,
    CONTROL_FLAG_FORCE_RC_SPEED = 1 << 6,  # Same bit, different mode
    CONTROL_FLAG_HIGH_RES_SPEED = 1 << 7,
)

# Legacy format (could use Switch() but meh)
ControlRequestOld = Struct(
    "control_mode" / ControlMode, # Common for all axes
    "target" / PerAxis(Struct(
        "speed" / Int16sl,       # Units: 0.1220740379 deg/sec or 0.001 if HIGH_RES flag
        "angle" / Int16sl,       # Units: 0.02197265625 deg
    )),
)

# Frw. ver. 2.55b5+, mode per axis
ControlRequest = Struct(
    "control_mode" / PerAxis(ControlMode),
    "target" / PerAxis(Struct(
        "speed" / Int16sl,       # Units: 0.1220740379 deg/sec or 0.001 if HIGH_RES flag
        "angle" / Int16sl,       # Units: 0.02197265625 deg
    )),
)

# CMD_CONTROL_EXT (#121) (frw. ver. 2.68+)
ControlExtDataSet = FlagsEnum(Int16ul,
    # Per-axis flags (bits 0-4 for axis 1, 5-9 for axis 2, 10-14 for axis 3)
    AXIS1_SPEED = 1 << 0,
    AXIS1_ANGLE = 1 << 1,
    AXIS1_ANGLE_32BIT = 1 << 2,
    AXIS1_SPEED_32BIT = 1 << 3,

    AXIS2_SPEED = 1 << 5,
    AXIS2_ANGLE = 1 << 6,
    AXIS2_ANGLE_32BIT = 1 << 7,
    AXIS2_SPEED_32BIT = 1 << 8,

    AXIS3_SPEED = 1 << 10,
    AXIS3_ANGLE = 1 << 11,
    AXIS3_ANGLE_32BIT = 1 << 12,
    AXIS3_SPEED_32BIT = 1 << 13,
)

ControlExtModeFlags = FlagsEnum(Int8ul,
    DISABLE_ANGLE_ERR_CORR = 1 << 0,
)

ControlExtRequest = Struct(
    "data_set" / ControlExtDataSet,
    # TODO: could use Computed(ctx._.data_set >> (ctx._index * 5)) (or Index) for the flag checks in parsing, but how will building work?
    # also check if we can use "this" instead of "ctx"
    "target" / PerAxis(If(lambda ctx: ctx._.data_set & (3 << (ctx._index * 5)), Struct(
        "control_mode" / ControlMode,
        "mode_flags" / ControlExtModeFlags,
        "speed" / If(lambda ctx: ctx._._.data_set & (1 << (ctx._._index * 5 + 0)),
            IfThenElse(lambda ctx: ctx._._.data_set & (1 << (ctx._._index * 5 + 3)), Int32sl, Int16sl)),
        "angle" / If(lambda ctx: ctx._._.data_set & (1 << (ctx._._index * 5 + 1)),
            IfThenElse(lambda ctx: ctx._._.data_set & (1 << (ctx._._index * 5 + 2)), Int32sl, Int16sl)),
    ))),
)

# CMD_CONTROL_CONFIG (#90)
ControlConfigFlags = FlagsEnum(Int16ul,
    NO_CONFIRM = 1 << 0,
    SERVO_MODE_ENABLE = 1 << 1,
    SERVO_MODE_DISABLE = 1 << 2,
    LPF_EXT_RANGE = 1 << 3,
)

ControlConfigRequest = Struct(
    "timeout_ms" / Int16ul,   # 0=disable timeout
    "ch1_priority" / Int8ul,  # UART1
    "ch2_priority" / Int8ul,  # RC_SERIAL
    "ch3_priority" / Int8ul,  # UART2
    "ch4_priority" / Int8ul,  # USB_VCP/UART3
    "this_ch_priority" / Int8ul, # Current port

    "axis_config" / PerAxis(Struct(
        "angle_lpf" / Int8ul,
        "speed_lpf" / Int8ul,
        "rc_lpf" / Int8ul,
        "acc_limit" / Int16ul, # Units: deg/sec^2
        "jerk_slope" / Int8ul, # Units: 20ms

        "reserved" / Bytes(1),
    )),

    "rc_expo_rate" / Int8ul,
    "flags" / ControlConfigFlags,
    "euler_order" / Int8ul, # 0=don't change, >0: subtract 1 and apply as EulerOrder
    "reserved" / Bytes(9),
)

# CMD_API_VIRT_CH_CONTROL (#45)
ApiVirtChControlRequest = Struct(
    "channels" / GreedyRange(Int16sl),  # 1-32 channels, -500 to 500, -10000=undefined
)

# CMD_API_VIRT_CH_HIGH_RES (#116) (frw.ver. 2.68b7+)
ApiVirtChHighResRequest = Struct(
    "channels" / GreedyRange(Int16sl),  # 1-32 channels, -16384 to 16384, -32768=undefined
)

# CMD_CONTROL_QUAT (#140) (frw.ver. 2.73.0)
ControlQuatMode = Enum(Int8ul,
    DISABLED = 0,
    SPEED = 1,
    ATTITUDE = 2,
    SPEED_ATTITUDE = 5,
    SPEED_LIMITED = 9,
)

ControlQuatFlags = FlagsEnum(Int8ul,
    CONFIRM = 1 << 0,
    ATTITUDE_PACKED = 1 << 1,
    ATTITUDE_LIMITED_180 = 1 << 2,
    ATT_REMOVE_ABSENT_AXES = 1 << 3,
    AUTO_TASK = 1 << 6,
)

ControlQuatRequest = Struct(
    "mode" / ControlQuatMode,
    "flags" / ControlQuatFlags,

    # TODO: not clear if "omitted" in the spec means actually omitting the bytes that would be
    # otherwise used by that field, must test.

    # Target attitude - conditional on mode.  TODO: implement the packing/unpacking here
    "target_attitude_full" / If(
        lambda ctx: ctx.mode in [ControlQuatMode.ATTITUDE, ControlQuatMode.SPEED_ATTITUDE] and
                    not (ctx.flags & ControlQuatFlags.ATTITUDE_PACKED),
        Array(4, Float32l)  # [w, x, y, z] quaternion
    ),
    "target_attitude_packed" / If(
        lambda ctx: ctx.mode in [ControlQuatMode.ATTITUDE, ControlQuatMode.SPEED_ATTITUDE] and
                    (ctx.flags & ControlQuatFlags.ATTITUDE_PACKED),
        Bytes(8)  # Compressed quaternion
    ),

    # Target speed - conditional on mode
    "target_speed" / If(
        lambda ctx: ctx.mode in [ControlQuatMode.SPEED, ControlQuatMode.SPEED_ATTITUDE,
                                  ControlQuatMode.SPEED_LIMITED],
        PerAxis(Float32l)  # [x, y, z] rotation vector, Units: rad/sec
    ),
)

# CMD_CONTROL_QUAT_CONFIG (#142) (frw.ver. 2.73.0)
ControlQuatConfigDataSet = FlagsEnum(Int16ul,
    MAX_SPEED = 1 << 0,
    ACC_LIMIT = 1 << 1,
    JERK_SLOPE = 1 << 2,
    FLAGS = 1 << 3,
    ATTITUDE_LPF_FREQ = 1 << 4,
    SPEED_LPF_FREQ = 1 << 5,
)

ControlQuatConfigFlags = FlagsEnum(Int16ul,
    MOTION_PROFILE_SPLIT_XYZ = 1 << 0,
)

ControlQuatConfigRequest = Struct(
    "data_set" / ControlQuatConfigDataSet,

    # Conditional fields based on data_set
    "max_speed" / If(lambda ctx: ctx.data_set & ControlQuatConfigDataSet.MAX_SPEED,
        PerAxis(Int16ul)),  # Units: 0.1220740379 deg/sec
    "acc_limit" / If(lambda ctx: ctx.data_set & ControlQuatConfigDataSet.ACC_LIMIT,
        PerAxis(Int16ul)),  # Units: deg/sec^2, 0=disable
    "jerk_slope" / If(lambda ctx: ctx.data_set & ControlQuatConfigDataSet.JERK_SLOPE,
        PerAxis(Int16ul)),  # Units: ms, 0=disable
    "flags" / If(lambda ctx: ctx.data_set & ControlQuatConfigDataSet.FLAGS,
        ControlQuatConfigFlags),
    "attitude_lpf_freq" / If(lambda ctx: ctx.data_set & ControlQuatConfigDataSet.ATTITUDE_LPF_FREQ,
        Int8ul),  # Units: Hz, 0=disable
    "speed_lpf_freq" / If(lambda ctx: ctx.data_set & ControlQuatConfigDataSet.SPEED_LPF_FREQ,
        Int8ul),  # Units: Hz, 0=disable
)

#
# Section: Controlling extra auxiliary motors
#

# CMD_EXT_MOTORS_ACTION (#128) (frw. ver. 2.73.0)
ExtMotorAction = Enum(Int8ul,
    MOTOR_OFF_FLOATING = 1,
    MOTOR_OFF_BRAKE = 2,
    MOTOR_OFF_SAFE = 3,
    MOTOR_ON = 4,
    HOME_POSITION = 5,
    SEARCH_HOME = 6,
)

ExtMotorsActionRequest = Struct(
    "for_motors" / FlagsEnum(Int8ul,
        MOTOR1 = 1 << 0,
        MOTOR2 = 1 << 1,
        MOTOR3 = 1 << 2,
        MOTOR4 = 1 << 3,
        MOTOR5 = 1 << 4,
        MOTOR6 = 1 << 5,
        MOTOR7 = 1 << 6,
        CONFIRM = 1 << 7,
    ),
    "action" / ExtMotorAction,
    "args" / GreedyBytes,  # Reserved, optional arguments depending on action
)

# CMD_EXT_MOTORS_CONTROL (#129) (frw. ver. 2.73.0)
ExtMotorControlDataSet = FlagsEnum(Int8ul,
    SETPOINT_32BIT = 1 << 0,
    PARAM1_16BIT = 1 << 1,
    PARAM1_32BIT = 1 << 2,
)

ExtMotorsControlRequest = Struct(
    "for_motors" / FlagsEnum(Int8ul,
        MOTOR1 = 1 << 0,
        MOTOR2 = 1 << 1,
        MOTOR3 = 1 << 2,
        MOTOR4 = 1 << 3,
        MOTOR5 = 1 << 4,
        MOTOR6 = 1 << 5,
        MOTOR7 = 1 << 6,
        CONFIRM = 1 << 7,
    ),
    "data_set" / ExtMotorControlDataSet,
    # Per-motor setpoints (number depends on bits set in for_motors) TODO: count bits
    "motor_data" / GreedyRange(Struct(
        # Setpoint - 16bit or 32bit based on data_set bit 0 # TODO: use Switch() for setpoint and param?
        "setpoint16" / If(lambda ctx: not (ctx._.data_set & ExtMotorControlDataSet.SETPOINT_32BIT),
        Int16sl),
        "setpoint32" / If(lambda ctx: ctx._.data_set & ExtMotorControlDataSet.SETPOINT_32BIT,
        Int32sl),
        # Optional PARAM1 - 16bit or 32bit based on data_set bits 1-2
        "param1_16" / If(lambda ctx: ctx._.data_set & ExtMotorControlDataSet.PARAM1_16BIT,
        Int16sl),
        "param1_32" / If(lambda ctx: ctx._.data_set & ExtMotorControlDataSet.PARAM1_32BIT,
        Int32sl),
    )),
)

# CMD_EXT_MOTORS_CONTROL_CONFIG (#130) (frw. ver. 2.73.0)
ExtMotorControlMode = Enum(Int8ul,
    POSITION = 0,
    SPEED = 1,
    TORQUE = 2,
)

ExtMotorControlConfigDataSet = FlagsEnum(Int16ul,
    MODE = 1 << 0,
    MAX_SPEED = 1 << 1,
    MAX_ACCELERATION = 1 << 2,
    JERK_SLOPE = 1 << 3,
    MAX_TORQUE = 1 << 4,
)

ExtMotorsControlConfigRequest = Struct(
    "for_motors" / FlagsEnum(Int8ul,
        MOTOR1 = 1 << 0,
        MOTOR2 = 1 << 1,
        MOTOR3 = 1 << 2,
        MOTOR4 = 1 << 3,
        MOTOR5 = 1 << 4,
        MOTOR6 = 1 << 5,
        MOTOR7 = 1 << 6,
        CONFIRM = 1 << 7,
    ),
    "data_set" / ExtMotorControlConfigDataSet,

    # Conditional fields based on data_set
    "mode" / If(lambda ctx: ctx.data_set & ExtMotorControlConfigDataSet.MODE,
        ExtMotorControlMode),
    "max_speed" / If(lambda ctx: ctx.data_set & ExtMotorControlConfigDataSet.MAX_SPEED,
        Int16ul),  # Units: 2 deg/sec
    "max_acceleration" / If(lambda ctx: ctx.data_set & ExtMotorControlConfigDataSet.MAX_ACCELERATION,
        Int16ul),  # Units: 2 deg/sec^2
    "jerk_slope" / If(lambda ctx: ctx.data_set & ExtMotorControlConfigDataSet.JERK_SLOPE,
        Int16ul),  # Units: ms
    "max_torque" / If(lambda ctx: ctx.data_set & ExtMotorControlConfigDataSet.MAX_TORQUE,
        Int16ul),  # Relative to max available torque
)

#
# Section: Miscellaneous commands
#

# CMD_CONFIRM (#67)
ConfirmData = Enum(Int8ul,
    SUCCESS = 0,
    TASK_COMPLETE = 1,
    # Other values depend on the command being confirmed
)

ConfirmResponse = Struct(
    "cmd_id" / CmdId,     # Command ID being confirmed
    "data" / GreedyBytes, # Command-specific data (often 0 for success) TODO: Int16ul or Int8ul
)

# CMD_ERROR (#255)
ErrorResponse = Struct(
    "cmd_id" / CmdId,
    "error_code" / ErrorCode,
    "error_data" / Bytes(4),
)

# CMD_RESET (#114)
ResetFlags = FlagsEnum(Int8ul,
    SEND_NOTIFICATION = 1 << 0,
    BACKUP_STATE = 1 << 1,
)

ResetRequest = Optional(Struct(
    "flags" / ResetFlags,
    "delay_ms" / Int16ul,
))

# CMD_RESET (#114) Response - notification
ResetResponse = Pass # Sent before reset, no data

# CMD_BOOT_MODE_3 (#51)
BootMode3Request = Optional(Struct(
    "confirm" / Enum(Int8ul, NO_CONFIRM=0, SEND_CMD_RESET=1),
    "delay_ms" / Int16ul,
))

# CMD_TRIGGER_PIN (#84)
PinState = Enum(Int8ul,
    LOW = 0,  # GND, can sink up to 40mA
    HIGH = 1,  # +3.3V, can source up to 40mA
    FLOATING = 2,  # frw. ver. 2.66+
)

TriggerPinRequest = Struct(
    "pin_id" / TriggerPinId,
    "state" / PinState,
)

# CMD_MOTORS_ON (#77)
MotorsOnRequest = Pass # No parameters

# CMD_MOTORS_OFF (#109)
MotorsOffMode = Enum(Int8ul,
    NORMAL = 0,    # High impedance
    BRAKE = 1,     # Low impedance
    SAFE_STOP = 2, # Reduce power, wait for stop, then OFF
)

MotorsOffRequest = Struct(
    "mode" / Default(MotorsOffMode, MotorsOffMode.NORMAL),  # frw.ver. 2.68b7+
)

# CMD_EXECUTE_MENU (#69)
# Execute menu flags (frw.ver. 2.73.8)
ExecuteMenuFlags = FlagsEnum(Int8ul,
    CONFIRM = 1 << 0,
    CONFIRM_ON_FINISH = 1 << 1,
)

ExecuteMenuRequest = Struct(
    "cmd_id" / MenuCommand,
    "flags" / Default(ExecuteMenuFlags, 0),  # Optional
)

# CMD_EVENT (#102) notification
EventId = Enum(Int8ul,
    MENU_BUTTON = 1,
    MOTOR_STATE = 2,
    EMERGENCY_STOP = 3,
    CAMERA = 4,
    SCRIPT = 5,
    RETRACTED_POSITION = 6,
)

EventType = FlagsEnum(Int8ul,
    OFF = 1 << 0,
    ON = 1 << 1,
    HOLD = 1 << 2,
    REC_PHOTO = 1 << 0,  # For CAMERA events
    PHOTO = 1 << 1,
)

EventNotification = Struct(
    "event_id" / EventId,
    "event_type" / EventType,
    "param1" / Bytes(2),  # Meaning depends on event_id and event_type
    "future_reserved" / GreedyBytes,
)

# CMD_AUTO_PID (#35) (frw. ver. prior to 2.73)
AutoPidAction = Enum(Int8ul,
    START = 0,
)

AutoPidRequest = Struct(
    "profile_id" / Int8ul,
    "cfg_flags" / AutoPidCfgFlags,
    "gain_vs_stability" / Int8ul,  # 0=stability, 255=tracking
    "momentum" / Int8ul,  # 0=auto-detect, 1=low weight, 255=big weight
    "action" / AutoPidAction,
    "reserved" / Bytes(14),
)

# CMD_AUTO_PID (#35) Response - progress updates
AutoPidResponse = Struct(
    "p" / Array(3, Int8ul),
    "i" / Array(3, Int8ul),
    "d" / Array(3, Int8ul),
    "lpf_freq" / Array(3, Int16ul),
    "iteration_cnt" / Int16ul,
    "tracking_error" / Float32l,
    "reserved" / Bytes(6),
    # Per-axis reserved fields
    "reserved_axes" / Array(3, Bytes(10)),
)

# CMD_AUTO_PID2 (#108) (frw. ver. 2.73+)
AutoPid2Action = Enum(Int8ul,
    START = 1,
    START_SAVE = 2,
    SAVE = 3,
    STOP = 5,
    READ = 6,
)

AutoPid2GeneralFlags = FlagsEnum(Int16ul,
    START_FROM_CURRENT = 1 << 0,
    UPDATE_ALL_PROFILES = 1 << 1,
    TUNE_GAIN_ONLY = 1 << 2,
    AUTO_SAVE = 1 << 4,
    RUN_AT_START_ALL = 1 << 14,
    RUN_AT_START_GAIN = 1 << 15,
)

AutoPid2AxisFlags = FlagsEnum(Int8ul,
    ENABLED = 1 << 0,
    TUNE_LPF = 1 << 1,
    NOTCH_COUNT_MASK = 0xc,  # bits 2-3: 0-3 notch filters
)

AutoPid2Request = Struct(
    "action" / AutoPid2Action,
    "cmd_flags" / FlagsEnum(Int16ul,
        SEND_CONFIG_ON_FINISH = 1 << 0,
    ),
    "reserved" / Bytes(8),

    # Following fields only for START, START_SAVE, SAVE actions
    "cfg" / If(this.action in [AutoPid2Action.START, AutoPid2Action.START_SAVE, AutoPid2Action.SAVE], Struct(
        "cfg_version" / Const(1, Int8ul),
        "axes" / PerAxis(Struct(
            "axis_flags" / AutoPid2AxisFlags,
            "gain" / Int8ul,            # 0-255, stability vs performance
            "stimulus_gain" / Int16ul,  # 0-65535
            "effective_freq" / Int8ul,  # Hz
            "problem_freq" / Int8ul,    # Hz
            "problem_margin" / Int8ul,  # dB/10
            "reserved" / Bytes(6),
        )),
        "general_flags" / AutoPid2GeneralFlags,
        "reserved2" / Bytes(1),
        "test_freq_from" / Int8ul,      # Units: 0.1Hz
        "test_freq_to" / Int8ul,        # Units: 2Hz
        "multi_pos_flag" / FlagsEnum(Int8ul,
            POS1 = 1 << 0,
            POS2 = 1 << 1,
            POS3 = 1 << 2,
            POS4 = 1 << 3,
        ),
        "multi_pos_angle" / Array(4, Int8sl),  # Units: 3 deg
        "reserved3" / Bytes(12),
        # "iterations_num" / Default(Int8ul, 2),
        # "reserved4" / Bytes(9),
    )),
)

# CMD_SERVO_OUT (#36)
ServoOutRequest = Struct(
    "servo_time" / Array(4, Int16sl),  # -1=float, 0=low, >0=PWM pulse time (μs)
    "reserved" / Bytes(8),
)

# CMD_SERVO_OUT_EXT (#133) (frw.ver. 2.73.2)
ServoOutExtRequest = Struct(
    "servo_bits" / FlagsEnum(Int32ul,
        SERVO1 = 1 << 0,
        SERVO2 = 1 << 1,
        SERVO3 = 1 << 2,
        SERVO4 = 1 << 3,
        CAN_DRV1_SERVO1 = 1 << 4,
        CAN_DRV1_SERVO2 = 1 << 5,
        CAN_DRV2_SERVO1 = 1 << 6,
        CAN_DRV2_SERVO2 = 1 << 7,
        CAN_DRV3_SERVO1 = 1 << 8,
        CAN_DRV3_SERVO2 = 1 << 9,
        CAN_DRV4_SERVO1 = 1 << 10,
        CAN_DRV4_SERVO2 = 1 << 11,
        CAN_DRV5_SERVO1 = 1 << 12,
        CAN_DRV5_SERVO2 = 1 << 13,
        CAN_DRV6_SERVO1 = 1 << 14,
        CAN_DRV6_SERVO2 = 1 << 15,
        CAN_DRV7_SERVO1 = 1 << 16,
        CAN_DRV7_SERVO2 = 1 << 17,
    ),
    "servo_time" / GreedyRange(Int16sl),  # Number of values = number of bits set # TODO: count them
)

# CMD_I2C_WRITE_REG_BUF (#39)
I2cWriteRegBufRequest = Struct(
    "device_addr" / Int8ul,  # bit0: port (0=external, 1=internal), bits1-7: I2C address
    "reg_addr" / Int8ul,
    "data" / GreedyBytes,
)

# CMD_I2C_READ_REG_BUF (#40)
I2cReadRegBufRequest = Struct(
    "device_addr" / Int8ul,  # bit0: port, bits1-7: I2C address
    "reg_addr" / Int8ul,
    "data_len" / Int8ul,
)

# CMD_I2C_READ_REG_BUF (#40) Response
I2cReadRegBufResponse = Struct(
    "data" / GreedyBytes,  # Length specified in request
)

# CMD_RUN_SCRIPT (#57)
RunScriptMode = Enum(Int8ul,
    STOP = 0,
    START = 1,
    START_DEBUG = 2,
)

RunScriptRequest = Struct(
    "mode" / RunScriptMode,
    "slot" / Int8ul,  # 0-4
    "reserved" / Bytes(32),
)

# CMD_BEEP_SOUND (#89)
BeeperMode = FlagsEnum(Int16ul,
    CALIBRATE = 1 << 0,
    CONFIRM = 1 << 1,
    ERROR = 1 << 2,
    CLICK = 1 << 4,
    COMPLETE = 1 << 5,
    INTRO = 1 << 6,
    CUSTOM_MELODY = 1 << 15,
)

BeepSoundRequest = Struct(
    "mode" / BeeperMode,
    "note_length" / Int8ul,  # Units: 8ms, for custom melody
    "decay_factor" / Int8ul, # 0-15, for custom melody
    "reserved" / Bytes(8),
    "note_freq_hz" / GreedyRange(Int16ul), # 0-50 notes, 554-21000 Hz, 21000+=restart envelope+pause
)

# CMD_SIGN_MESSAGE (#50)
SignMessageRequest = Struct(
    "sign_type" / Int8ul,
    "message" / Bytes(32),
)

# CMD_SIGN_MESSAGE (#50) Response
SignMessageResponse = Struct(
    "signature" / Bytes(32),
)

# CMD_EXT_IMU_CMD (#110)
ExtImuCmdRequest = Struct(
    "cmd_id" / Int8ul, # GPS_IMU API IDs
    "data" / GreedyBytes,
)

# CMD_EXT_IMU_CMD (#110) Response
ExtImuCmdResponse = LazyBound(lambda: ExtImuCmdRequest)

# CMD_EXT_SENS_CMD (#150) (min. frw.ver. 2.68b7)
ExtSensCmdRequest = Struct(
    "flags" / FlagsEnum(Int8ul,
        HIGH_PRIORITY = 1 << 0,
    ),
    "cmd_id" / Int8ul, # GPS_IMU API IDs
    "data" / GreedyBytes,
)

# CMD_EXT_SENS_CMD (#150) Response
ExtSensCmdResponse = LazyBound(lambda: ExtImuCmdRequest)

# CMD_CAN_DEVICE_SCAN (#96)
CanDeviceScanRequest = Pass # No parameters

# CMD_CAN_DEVICE_SCAN (#96) Response
CanDeviceScanResponse = Struct(
    "devices" / GreedyRange(Struct(
        "uid" / Bytes(12),
        "id" / Enum(Int8ul, ## TODO unify if there are other places that use this or the next field
            NOT_ASSIGNED = 0,
            CAN_IMU_MAIN = 5,
            CAN_IMU_FRAME = 6,
            GPS_IMU_MAIN = 7,
            CAN_DRV1 = 17,
            CAN_DRV2 = 18,
            CAN_DRV3 = 19,
            CAN_DRV4 = 20,
            CAN_DRV5 = 21,
            CAN_DRV6 = 22,
            CAN_DRV7 = 23,
            CAN_IMU_MAIN_OLD = 28,
            CAN_IMU_FRAME_OLD = 29,
        ),
        "type" / FlagsEnum(Int8ul,
            TYPE_MASK = 0x7f,  # bits 0-6
            MOTOR_DRIVER = 1,
            IMU = 2,
            HARD_ASSIGNED = 1 << 7,  # bit 7
        ),
    )),
)

# CMD_MODULE_LIST (#76)
ModuleListRequest = Pass # No parameters

# CMD_MODULE_LIST (#76) Response
ModuleListResponse = Struct(
    "device_num" / Int8ul,
    "modules" / Array(this.device_num, Struct(
        "id" / Enum(Int8ul,
            IMU_MAIN = 1,
            IMU_FRAME = 2,
            CAN_DRV1 = 3,
            CAN_DRV2 = 4,
            CAN_DRV3 = 5,
            CAN_DRV4 = 6,
            CAN_DRV5 = 7,
            CAN_DRV6 = 8,
            CAN_DRV7 = 9,
            GPS_IMU = 10,
            CAN_HUB_GNSS = 11,
            CAN_HUB1 = 12,
            CAN_HUB2 = 13,
        ),
        "board_ver" / Int16ul,      # Format: xx.xx
        "bootloader_ver" / Int16ul,
        "firmware_ver" / Int16ul,
        "reserved" / Bytes(6),
    )),
)

# Transparent SAPI target device
TransparentSapiDevice = Enum(Int8ul,
    SBGC32 = 1,
    GPS_IMU = 2,
    CAN_IMU_MAIN = 3,
    CAN_IMU_FRAME = 4,
    GPS_SPLIT_RCVR = 5,
    CAN_SERIAL_HUB1 = 6,
    CAN_SERIAL_HUB2 = 7,
    CAN_DRV1 = 8,
    CAN_DRV2 = 9,
    CAN_DRV3 = 10,
    CAN_DRV4 = 11,
)

# CMD_TRANSPARENT_SAPI (#151) (min. frw.ver. 2.72b0)
TransparentSapiPort = FlagsEnum(Int8ul,
    SBGC32_UART1     = (1 << 2) | 0,
    SBGC32_RC_SERIAL = (1 << 2) | 1,
    SBGC32_UART2     = (1 << 2) | 2,
    SBGC32_USB_VCP   = (1 << 2) | 3,
    GPS_IMU_UART1    = (2 << 2) | 0,
    GPS_IMU_UART2    = (2 << 2) | 1,
    IMU_MAIN_UART1   = (3 << 2) | 0,
    IMU_MAIN_UART2   = (3 << 2) | 1,
    IMU_FRAME_UART1  = (4 << 2) | 0,
    IMU_FRAME_UART2  = (4 << 2) | 1,
    GPS_SPLIT_RCVR_UART1 = (5 << 2) | 0,
    GPS_SPLIT_RCVR_UART2 = (5 << 2) | 1,
    GPS_SPLIT_RCVR_UART3 = (5 << 2) | 2,
    GPS_SPLIT_RCVR_GNSS  = (5 << 2) | 3,
    CAN_HUB1_SERIAL1 = (6 << 2) | 1,
    CAN_HUB1_SERIAL2 = (6 << 2) | 2,
    CAN_HUB1_SERIAL3 = (6 << 2) | 3,
    CAN_HUB2_SERIAL1 = (7 << 2) | 1,
    CAN_HUB2_SERIAL2 = (7 << 2) | 2,
    CAN_HUB2_SERIAL3 = (7 << 2) | 3,
    CAN_DRV1_UART2   = (8 << 2) | 0,
    CAN_DRV2_UART2   = (9 << 2) | 0,
    CAN_DRV3_UART2   = (10 << 2) | 0,
    CAN_DRV4_UART2   = (11 << 2) | 0,

    WAIT_FLAG        = 1 << 6
)

TransparentSapiRequest = Struct(
    "target" / TransparentSapiPort,  # bits0-1: port, bits2-5: device, bit6: wait flag, bit7: reserved
    "payload" / GreedyBytes,
)

# CMD_TRANSPARENT_SAPI (#151) Response
TransparentSapiResponse = Struct(
    "payload" / GreedyBytes,
)

# Helper to construct target byte
def make_transparent_sapi_target(device_id, port_id, wait_flag=False):
    """Construct TARGET byte: PortId + (DeviceId<<2) + (WaitFlag<<6)"""
    return port_id | (device_id << 2) | (int(wait_flag) << 6)

#
# Section: EEPROM and internal file system
#

# CMD_READ_FILE (#53)
FileType = Enum(Int8ul,
    SCRIPT = 1,
    IMU_CALIB = 3,
    COGGING_CORRECTION = 4,
    ADJ_VAR_LUT = 5,
    PROFILE_SET = 6,
    PARAMS = 7,
    TUNE = 8,
    CANDRV = 10,
)

FileId = Struct(
    "type" / FileType,
    "subtype" / Int8ul,  # Meaning depends on type
)

ReadFileRequest = Struct(
    "file_id" / FileId,
    "page_offset" / Int16ul, # Offset in pages (1 page = 64 bytes)
    "max_size" / Int16ul,    # Maximum bytes to read
    "reserved" / Bytes(14),
)

# CMD_READ_FILE (#53) Response - Success
ReadFileResponse = Struct(
    "file_size" / Int16ul,   # Total file size
    "page_offset" / Int16ul, # Page offset from request
    "data" / GreedyBytes,    # Data (up to max_size from request)
)

# CMD_READ_FILE (#53) Response - Error
ReadFileErrorResponse = Struct(
    "err_code" / ErrorCode,
)

# CMD_WRITE_FILE (#54)
WriteFileRequest = Struct(
    "file_id" / FileId,
    "file_size" / Int16ul,   # Total file size
    "page_offset" / Int16ul, # Offset in pages (1 page = 64 bytes)
    "data" / GreedyBytes,    # Data to write (empty to delete file)
)

# CMD_FS_CLEAR_ALL (#55)
FsClearAllRequest = Pass # No parameters

# CMD_EEPROM_WRITE (#47)
EepromWriteRequest = Struct(
   "addr" / Int32ul,  # Address (must be 64-byte aligned) # TODO: enforce
   "data" / GreedyBytes,  # Data (must be 64-byte aligned) # TODO: enforce
)

# CMD_EEPROM_READ (#48)
EepromReadRequest = Struct(
    "addr" / Int32ul,  # Address (must be 64-byte aligned) # TODO: enforce
    "size" / Int16ul,  # Size (64-192 bytes, 64-byte aligned) # TODO: enforce
)

# CMD_EEPROM_READ (#48) Response
EepromReadResponse = Struct(
    "addr" / Int32ul,  # Address that was read TODO: enforce alignment
    "data" / GreedyBytes,  # Data (size specified in request)
)

# CMD_WRITE_EXTERNAL_DATA (#41)
WriteExternalDataRequest = Struct(
    "data" / Bytes(128),
)

# CMD_READ_EXTERNAL_DATA (#42)
ReadExternalDataRequest = Pass # No parameters

# CMD_READ_EXTERNAL_DATA (#42) Response
ReadExternalDataResponse = Struct(
    "data" / Bytes(128),
)

# Format: (request_cmd_id, command_name, request_struct, response_cmd_id, response_struct)
cmd_id_list = [
    # Device Information
    (86, "CMD_BOARD_INFO", BoardInfoRequest, 86, BoardInfoResponse),
    (20, "CMD_BOARD_INFO_3", BoardInfo3Request, 20, BoardInfo3Response),

    # Configuring Gimbal - Read Parameters
    (82, "CMD_READ_PARAMS", ReadParamsRequest, 82, None),  # Deprecated, use CMD_READ_PARAMS_3
    (21, "CMD_READ_PARAMS_3", ReadParamsRequest, 21, ReadParams3Response),
    (33, "CMD_READ_PARAMS_EXT", ReadParamsRequest, 33, ReadParamsExtResponse),
    (62, "CMD_READ_PARAMS_EXT2", ReadParamsRequest, 62, ReadParamsExt2Response),
    (104, "CMD_READ_PARAMS_EXT3", ReadParamsRequest, 104, ReadParamsExt3Response),

    # Configuring Gimbal - Write Parameters
    (87, "CMD_WRITE_PARAMS", None, 67, ConfirmResponse),  # Deprecated
    (22, "CMD_WRITE_PARAMS_3", ReadParams3Response, 67, ConfirmResponse),
    (34, "CMD_WRITE_PARAMS_EXT", ReadParamsExtResponse, 67, ConfirmResponse),
    (63, "CMD_WRITE_PARAMS_EXT2", ReadParamsExt2Response, 67, ConfirmResponse),
    (105, "CMD_WRITE_PARAMS_EXT3", ReadParamsExt3Response, 67, ConfirmResponse),
    (119, "CMD_WRITE_PARAMS_SET", WriteParamsSetRequest, 67, ConfirmResponse),

    # Configuring Gimbal - Other
    (70, "CMD_USE_DEFAULTS", UseDefaultsRequest, 67, ConfirmResponse),
    (79, "CMD_CALIB_OFFSET", CalibOffsetRequest, 67, ConfirmResponse),
    (28, "CMD_READ_PROFILE_NAMES", ReadProfileNamesRequest, 28, ReadProfileNamesResponse),
    (29, "CMD_WRITE_PROFILE_NAMES", WriteProfileNamesRequest, 67, ConfirmResponse),
    (95, "CMD_PROFILE_SET", ProfileSetRequest, 67, ConfirmResponse),

    # Calibrating
    (65, "CMD_CALIB_ACC", CalibSensorExtRequest, 67, ConfirmResponse),
    (103, "CMD_CALIB_GYRO", CalibSensorExtRequest, 67, ConfirmResponse),
    (59, "CMD_CALIB_MAG", CalibSensorExtRequest, 67, ConfirmResponse),
    (71, "CMD_CALIB_EXT_GAIN", CalibExtGainRequest, 67, ConfirmResponse),
    (80, "CMD_CALIB_POLES", CalibPolesRequest, 67, ConfirmResponse),
    (66, "CMD_CALIB_BAT", CalibBatRequest, 67, ConfirmResponse),
    (26, "CMD_ENCODERS_CALIB_OFFSET_4", EncodersCalibOffset4Request, 67, ConfirmResponse),
    (27, "CMD_ENCODERS_CALIB_FLD_OFFSET_4", EncodersCalibFldOffset4Request, 67, ConfirmResponse),
    (91, "CMD_CALIB_ORIENT_CORR", CalibOrientCorrRequest, 67, ConfirmResponse), # Also ReadParamsExt2Response
    (94, "CMD_CALIB_ACC_EXT_REF", CalibAccExtRefRequest, 67, ConfirmResponse),
    (93, "CMD_CALIB_COGGING", CalibCoggingRequest, 67, ConfirmResponse),
    (123, "CMD_SYNC_MOTORS", SyncMotorsRequest, 67, ConfirmResponse),

    # Real-time State Monitoring
    (88, "CMD_REALTIME_DATA_CUSTOM", RealtimeDataCustomRequest, 88, RealtimeDataCustomResponse),
    (68, "CMD_REALTIME_DATA", RealtimeData3Request, 23, RealtimeData3Response),
    (23, "CMD_REALTIME_DATA_3", RealtimeData3Request, 23, RealtimeData3Response),
    (25, "CMD_REALTIME_DATA_4", RealtimeData4Request, 25, RealtimeData4Response),
    (85, "CMD_DATA_STREAM_INTERVAL", DataStreamIntervalRequest, 67, ConfirmResponse),
    (100, "CMD_READ_RC_INPUTS", ReadRcInputsRequest, 100, ReadRcInputsResponse),
    (73, "CMD_GET_ANGLES", GetAnglesRequest, 73, GetAnglesResponse),
    (61, "CMD_GET_ANGLES_EXT", GetAnglesExtRequest, 61, GetAnglesExtResponse),
    (24, "CMD_SELECT_IMU_3", SelectImu3Request, None, None),
    (253, "CMD_DEBUG_VARS_INFO_3", DebugVarsInfo3Request, 253, DebugVarsInfo3Response),
    (254, "CMD_DEBUG_VARS_3", DebugVars3Request, 254, DebugVars3Response),
    (49, "CMD_CALIB_INFO", CalibInfoRequest, 49, CalibInfoResponse),
    (111, "CMD_READ_STATE_VARS", ReadStateVarsRequest, 111, ReadStateVarsResponse),
    (112, "CMD_WRITE_STATE_VARS", ReadStateVarsResponse, 67, ConfirmResponse),
    (249, "CMD_SET_DEBUG_PORT", SetDebugPortRequest, 249, SetDebugPortResponse),
    (127, "CMD_CAN_DRV_TELEMETRY", None, 127, CanDrvTelemetryNotification),
    (131, "CMD_EXT_MOTORS_STATE", ExtMotorsStateRequest, 131, ExtMotorsStateResponse),
    (141, "CMD_CONTROL_QUAT_STATUS", ControlQuatStatusRequest, 141, ControlQuatStatusResponse),

    # Run-time Gimbal Parameters
    (132, "CMD_ADJ_VARS_INFO", AdjVarsInfoRequest, 132, AdjVarsInfoResponse),
    (31, "CMD_SET_ADJ_VARS_VAL", SetAdjVarsValRequest, 31, SetAdjVarsValResponse),
    (134, "CMD_SET_ADJ_VARS_F", SetAdjVarsValFRequest, 134, SetAdjVarsValFResponse),
    (64, "CMD_GET_ADJ_VARS_VAL", GetAdjVarsValRequest, 31, SetAdjVarsValResponse),
    (135, "CMD_GET_ADJ_VARS_VAL_F", GetAdjVarsValFRequest, 134, SetAdjVarsValFResponse),
    (43, "CMD_READ_ADJ_VARS_CFG", ReadAdjVarsCfgRequest, 43, ReadAdjVarsCfgResponse),
    (44, "CMD_WRITE_ADJ_VARS_CFG", ReadAdjVarsCfgResponse, 67, ConfirmResponse),
    (32, "CMD_SAVE_PARAMS_3", SaveParams3Request, 67, ConfirmResponse),
    (46, "CMD_ADJ_VARS_STATE", AdjVarsStateRequest, 46, AdjVarsStateResponse),

    # IMU Correction and Diagnostic
    (72, "CMD_HELPER_DATA", HelperDataRequest, None, None),
    (56, "CMD_AHRS_HELPER", AhrsHelperRequest, 56, AhrsHelperResponse),
    (75, "CMD_GYRO_CORRECTION", GyroCorrectionRequest, None, None),
    (106, "CMD_EXT_IMU_DEBUG_INFO", None, 106, ExtImuDebugInfoResponse),  # Notification

    # Controlling Gimbal Movements
    (67, "CMD_CONTROL", ControlRequest, 67, ConfirmResponse),
    (121, "CMD_CONTROL_EXT", ControlExtRequest, 67, ConfirmResponse),
    (90, "CMD_CONTROL_CONFIG", ControlConfigRequest, 67, ConfirmResponse),
    (45, "CMD_API_VIRT_CH_CONTROL", ApiVirtChControlRequest, None, None),
    (116, "CMD_API_VIRT_CH_HIGH_RES", ApiVirtChHighResRequest, None, None),
    (140, "CMD_CONTROL_QUAT", ControlQuatRequest, 67, ConfirmResponse),
    (142, "CMD_CONTROL_QUAT_CONFIG", ControlQuatConfigRequest, 67, ConfirmResponse),

    # Controlling Extra Auxiliary Motors
    (128, "CMD_EXT_MOTORS_ACTION", ExtMotorsActionRequest, 67, ConfirmResponse),
    (129, "CMD_EXT_MOTORS_CONTROL", ExtMotorsControlRequest, 67, ConfirmResponse),
    (130, "CMD_EXT_MOTORS_CONTROL_CONFIG", ExtMotorsControlConfigRequest, 67, ConfirmResponse),

    # Miscellaneous Commands
    (114, "CMD_RESET", ResetRequest, 114, ResetResponse),
    (51, "CMD_BOOT_MODE_3", BootMode3Request, None, None),
    (84, "CMD_TRIGGER_PIN", TriggerPinRequest, 67, ConfirmResponse),
    (77, "CMD_MOTORS_ON", MotorsOnRequest, 67, ConfirmResponse),
    (109, "CMD_MOTORS_OFF", MotorsOffRequest, 67, ConfirmResponse),
    (69, "CMD_EXECUTE_MENU", ExecuteMenuRequest, 67, ConfirmResponse),
    (35, "CMD_AUTO_PID", AutoPidRequest, 35, AutoPidResponse),
    (108, "CMD_AUTO_PID2", AutoPid2Request, 67, ConfirmResponse),
    (36, "CMD_SERVO_OUT", ServoOutRequest, None, None),
    (133, "CMD_SERVO_OUT_EXT", ServoOutExtRequest, None, None),
    (39, "CMD_I2C_WRITE_REG_BUF", I2cWriteRegBufRequest, 67, ConfirmResponse),
    (40, "CMD_I2C_READ_REG_BUF", I2cReadRegBufRequest, 40, I2cReadRegBufResponse),
    (57, "CMD_RUN_SCRIPT", RunScriptRequest, 67, ConfirmResponse),
    (89, "CMD_BEEP_SOUND", BeepSoundRequest, None, None),
    (50, "CMD_SIGN_MESSAGE", SignMessageRequest, 50, SignMessageResponse),
    (110, "CMD_EXT_IMU_CMD", ExtImuCmdRequest, 110, ExtImuCmdResponse),
    (150, "CMD_EXT_SENS_CMD", ExtSensCmdRequest, 150, ExtSensCmdResponse),
    (96, "CMD_CAN_DEVICE_SCAN", CanDeviceScanRequest, 96, CanDeviceScanResponse),
    (76, "CMD_MODULE_LIST", ModuleListRequest, 76, ModuleListResponse),
    (151, "CMD_TRANSPARENT_SAPI", TransparentSapiRequest, 151, TransparentSapiResponse),

    # Response-only / Notifications
    (None, "CMD_CONFIRM", None, 67, ConfirmResponse),
    (None, "CMD_ERROR", None, 255, ErrorResponse),
    (None, "CMD_EVENT", None, 102, EventNotification),
    (None, "CMD_SCRIPT_DEBUG", None, 58, ScriptDebugNotification),

    # EEPROM and Internal File System
    (53, "CMD_READ_FILE", ReadFileRequest, 53, ReadFileResponse),
    (54, "CMD_WRITE_FILE", WriteFileRequest, 67, ConfirmResponse),
    (55, "CMD_FS_CLEAR_ALL", FsClearAllRequest, 67, ConfirmResponse),
    (47, "CMD_EEPROM_WRITE", EepromWriteRequest, 67, ConfirmResponse),
    (48, "CMD_EEPROM_READ", EepromReadRequest, 48, EepromReadResponse),
    (41, "CMD_WRITE_EXTERNAL_DATA", WriteExternalDataRequest, 67, ConfirmResponse),
    (42, "CMD_READ_EXTERNAL_DATA", ReadExternalDataRequest, 42, ReadExternalDataResponse),
]

by_name = {name: cmd_id for cmd_id, name, _, _, _ in cmd_id_list if cmd_id is not None}
by_name.update({name: resp_id for cmd_id, name, _, resp_id, _ in cmd_id_list if cmd_id is None})### maybe wanna add resp name??
CmdId = Enum(Int8ul, **by_name)

request_dict = { cmd[0]: cmd[2] for cmd in cmd_id_list if cmd[0] is not None and cmd[2] is not None }
response_dict = { cmd[3]: cmd[4] for cmd in cmd_id_list if cmd[0] is not None and cmd[2] is not None }

RequestPayload = Switch(this.cmd_id, request_dict, default=Bytes(this.payload_size))
ResponsePayload = Switch(this.cmd_id, response_dict, default=Bytes(this.payload_size))
