/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef HW_H
#define HW_H

extern "C" {
#include "main.h"
}
#include "i2c.h"
#include "nt.h"
#include "encoder.h"
#include "imu.h"
#include "imu-mpuxxxx.h"
#include "motor.h"
extern "C" {
#include "motor-bldc.h"
}
#include "sbgc32_i2c_drv.h"
#include "encoder-as5600.h"
extern "C" {
#include "util.h"
}

struct obgc_hw_i2c_addr_s {
    uint8_t bus;
    uint8_t addr;
} __attribute__((packed));

/* Note: changes here may need a STORAGE_CONFIG_VERSION bump in storage.h */
/* We have no space for a more generic Device-Tree like setup,
 * expose exactly the supported combinations and nothing more.
 */
struct obgc_hw_config_s {
    struct obgc_imu_hw_config_s {
        enum {
            OBGC_IMU_NONE,
            OBGC_IMU_I2C_MPU6050,
            OBGC_IMU_I2C_MPU9250, /* TODO: maybe MPUXXXX_AUTO */
            OBGC_IMU_NT,
        } __attribute__((packed)) type;
        union {
            struct obgc_hw_i2c_addr_s i2c;
            enum obgc_nt_imu_id_e {
                OBGC_NT_IMU_ID_IMU1,
                OBGC_NT_IMU_ID_IMU2,
                OBGC_NT_IMU_ID_IMU3,
            } __attribute__((packed)) nt_id;
        } params; /* Can't define enums inline if this is anonymous */
    } main_imu, frame_imu;
    struct obgc_motor_hw_config_s {
        enum {
            OBGC_MOTOR_NONE,
            OBGC_MOTOR_DRV_ONBOARD0,
            OBGC_MOTOR_DRV_ONBOARD1,
            OBGC_MOTOR_DRV_ONBOARD2,
            OBGC_MOTOR_DRV_3IN_1EN,
            OBGC_MOTOR_DRV_SBGC32_I2C,
            OBGC_MOTOR_DRV_NT,
        } __attribute__((packed)) type;
        union {
            struct obgc_motor_drv_hw_pins_s {
                uint8_t in[3];
                uint8_t en;
            } pins;
            enum obgc_sbgc32_i2c_drv_id_e {
                OBGC_SBGC32_I2C_DRV_ID_1,
                OBGC_SBGC32_I2C_DRV_ID_2,
                OBGC_SBGC32_I2C_DRV_ID_3,
                OBGC_SBGC32_I2C_DRV_ID_4,
            } __attribute__((packed)) sbgc32_id; /* TODO: accept bus num? */
            enum obgc_nt_motor_id_e {
                OBGC_NT_MOTOR_ID_PITCH,
                OBGC_NT_MOTOR_ID_ROLL,
                OBGC_NT_MOTOR_ID_YAW,
            } __attribute__((packed)) nt_id;
        } params;
    } motor[3];
    struct obgc_encoder_hw_config_s {
        enum {
            OBGC_ENCODER_NONE,
            OBGC_ENCODER_I2C_AS5600,     /* No address choice */
            OBGC_ENCODER_SBGC32_I2C_DRV, /* ID matches motor */
            OBGC_ENCODER_NT_MOTOR_DRV,   /* ID matches motor */
        } __attribute__((packed)) type;
        union {
            struct obgc_hw_i2c_addr_s i2c; /* .addr ignored for AS5600 */
            enum sbgc32_i2c_drv_encoder_type sbgc32_type;
        } params;
    } encoder[3];
};

struct busses_s {
    obgc_i2c *i2c_main, *i2c_int; /* External and internal in SBGC Serial API docs */
    obgc_nt_bus_t *nt;
};

/* platformio passes: -DARDUINO_SIMPLEBGC32_REGULAR -DBOARD_NAME=\"SIMPLEBGC32_REGULAR\".
 * We can't compare strings in preprocessor so define new macros based on -DARDUINO_*
 */
#if defined(ARDUINO_SIMPLEBGC32_REGULAR)
# define BOARD_SIMPLEBGC32_REGULAR 1
#elif defined(ARDUINO_STORM32_STM32F1)
# define BOARD_STORM32_STM32F1 1
#endif

#if BOARD_SIMPLEBGC32_REGULAR

/*
 * SBGC_ prefix for these because most of them seem to be part of the SBGC32 reference design
 * and not user configurable.  But these defines are all based specifically on the PilotFly H2
 * controller board "GH_ENCODER_MB2_RevB".
 */

# define SBGC_LED_GREEN     PB12
/* Red LED apparently always-on, not software controllable */

# define SBGC_SDA_MAIN      PA14  /* The sensors and extension boards are on this bus */
# define SBGC_SCL_MAIN      PA15

# define SBGC_SDA_AUX       PB7   /* The EEPROM chip is on this bus (something else too) */
# define SBGC_SCL_AUX       PB6

# define SBGC_VBAT          PA0   /* Through R17 (140k?) to BAT+ and R18 to GND (4.7k) */
# define SBGC_VBAT_R_BAT    140000
# define SBGC_VBAT_R_GND    4700

/* Onboard DRV8313 for the yaw motor */
# define SBGC_YAW_DRV8313_IN1     PA1   /* TIM2 */
# define SBGC_YAW_DRV8313_IN2     PA2   /* TIM2 */
# define SBGC_YAW_DRV8313_IN3     PA3   /* TIM2 */
# define SBGC_YAW_DRV8313_EN123   PB10

#if 0
/* Onboard DRV8313 traces seen on the SimpleBGC32 "Extended" board */
#define SBGC_PITCH_DRV8313_IN1   PE3
#define SBGC_PITCH_DRV8313_IN2   PE4
#define SBGC_PITCH_DRV8313_IN3   PE5

#define SBGC_ROLL_DRV8313_EN123  PC1
#endif

/* It does not look like there's any Rsense connected between DRV8313's PGND{1,2,3} and GND */
/* TODO: current and temperature sensing for yaw motor */

/* These 3 signals come from the 8-wire pogo-pin base/handle connector.  On the handle side, with
 * the basic one-hand handle the MODE pin connects directly to the button.  The YAW and PITCH lines
 * come from the Renesas R5F10268 SSOP-20 MCU chip (8KB code flash, 2KB data flash, 768B RAM) which
 * probably generates the PWM signals from the analog joystick signals.  20 PWM cycles per second.
 * We only receive these signals when the handle is powered on (battery or external barrel connector).
 *
 * In the two-hand base I believe the pins are left unconnected and the actual joystick and
 * MODE button inputs come through the Bluetooth remote interface.
 *
 * TODO: on PilotFly H2 ignore the RC signals until power settles as there are sometimes flukes the
 * moment the handle is powered on while we were already powered from USB.
 */
# define SBGC_IN_YAW        PB3  /* PilotFly H2 handle joystick horizontal axis PWM */
# define SBGC_IN_PITCH      PB5  /* PilotFly H2 handle joystick vertical axis PWM */
# define SBGC_IN_MODE       PC13 /* PilotFly H2 handle button, short to GND when pressed */
                                /* No discrete pull-up so needs our internal pull-up enabled */

# define SBGC_IN_RC_YAW     SBGC_IN_YAW
# define SBGC_IN_RC_ROLL    PB4
# define SBGC_IN_RC_PIT     SBGC_IN_PITCH

# define PIN_VBAT           SBGC_VBAT
# define PIN_VBAT_R_BAT     SBGC_VBAT_R_BAT
# define PIN_VBAT_R_GND     SBGC_VBAT_R_GND

# define PIN_LED0           SBGC_LED_GREEN

# define PIN_NT_RX          0
# define PIN_NT_TX          0
# define PIN_I2C_MAIN_SDA   SBGC_SDA_MAIN
# define PIN_I2C_MAIN_SCL   SBGC_SCL_MAIN
# define PIN_I2C_INT_SDA    SBGC_SDA_AUX
# define PIN_I2C_INT_SCL    SBGC_SCL_MAIN

# define PIN_RC_YAW         SBGC_IN_RC_YAW
# define PIN_RC_PITCH       SBGC_IN_RC_PIT
# define PIN_RC_ROLL        SBGC_IN_RC_ROLL
# define PIN_MODE           SBGC_IN_MODE

#elif BOARD_STORM32_STM32F1

# define PIN_VBAT           LIPO
# define PIN_VBAT_R_BAT     10000
# define PIN_VBAT_R_GND     1500

# define PIN_LED0           LED_GREEN

# define PIN_NT_RX          PB11
# define PIN_NT_TX          PB10
# define PIN_I2C_MAIN_SDA   I2C1_SDA /* PB11 */
# define PIN_I2C_MAIN_SCL   I2C1_SCL /* PB10 */
# define PIN_I2C_INT_SDA    I2C2_SDA
# define PIN_I2C_INT_SCL    I2C2_SCL

# define PIN_RC_YAW         RC2_0
# define PIN_RC_PITCH       RC2_1
# define PIN_RC_ROLL        RC2_2
# define PIN_MODE           USER_BTN

# define SBGC_YAW_DRV8313_IN1   M0_A
# define SBGC_YAW_DRV8313_IN2   M0_B
# define SBGC_YAW_DRV8313_IN3   M0_C
# define SBGC_YAW_DRV8313_EN123 0

#endif

static inline void hw_early_init() {
    pinMode(PIN_VBAT, INPUT);
    pinMode(PIN_MODE, INPUT_PULLUP);
    pinMode(PIN_RC_YAW, INPUT);
    pinMode(PIN_RC_PITCH, INPUT);
    pinMode(PIN_RC_ROLL, INPUT);
    pinMode(PIN_LED0, OUTPUT);

    /* On STorM32 initialize the port used by the crash handler separately
     * because we're not using it as the console.
     */
#if BOARD_STORM32_STM32F1
    Serial1.begin(115200);
#endif
}

#if BOARD_SIMPLEBGC32_REGULAR
# define ConsoleSerial           HardwareSerial
# define hw_get_console_serial() new HardwareSerial(USART1)
#elif BOARD_STORM32_STM32F1
# define ConsoleSerial           USBSerial
# define hw_get_console_serial() new USBSerial()
#endif

/* "External" and "internal" in SimpleBGC32 Serial API docs */
static inline void hw_early_i2c_init(struct busses_s *bus) {
#if BOARD_SIMPLEBGC32_REGULAR
    /* Initialize the onboard I2C bus early and unconditionally because
     * we need it to load our hardware config from the I2C EEPROM.  All other
     * peripherals after that can be made user-configurable, but this one
     * must be hardcoded (barring some sort of 2-level configuration with the
     * location of the true storage backend read from the on-chip flash?)
     *
     * On these boards both I2C busses use I2C1 peripheral pins so we can
     * use the fast hardware I2C1 block for only one of the busses.  We prefer
     * to use it for the "external" (main) bus because it sees much more
     * run-time traffic.  Drive the onboard, "internal", bus in software using
     * FlexWire.
     */
    bus->i2c_int = new obgc_i2c_subcls<FlexWire>(PIN_I2C_INT_SDA, PIN_I2C_INT_SCL);
    bus->i2c_int->begin();
#endif
}

void hw_storage_init(struct busses_s *bus);
void hw_setup(const struct obgc_hw_config_s *config, struct busses_s *bus,
        struct obgc_imu_s **main_imu, struct obgc_imu_s **frame_imu,
        struct obgc_foc_driver_s **motors,
        struct obgc_encoder_s **encoders);

#endif /* HW_H */
