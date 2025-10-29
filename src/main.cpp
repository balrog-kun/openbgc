/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

#include "imu-mpu6050.h"
#include "sbgc32_i2c_drv.h"
#include "encoder-as5600.h"
extern "C" {
#include "motor-pwm.h"
#include "motor-bldc.h"
#include "ahrs.h"
#include "axes.h"
#include "moremath.h"
#include "control.h"

#include "main.h"
}

/*
 * SBGC_ prefix for these because most of them seem to be part of the SBGC32 reference design
 * and not user configurable.
 */

#define SBGC_LED_GREEN     PB12
/* Red LED apparently always-on, not software controllable */

#define SBGC_SDA           PA14
#define SBGC_SCL           PA15

#define SBGC_VBAT          PA0   /* Through R17 (140k?) to BAT+ and R18 to GND (4.7k) */
#define SBGC_VBAT_R_BAT    140000
#define SBGC_VBAT_R_GND    4700
#define SBGC_VBAT_SCALE    0.9f  /* TODO: make user configurable */

#define SBGC_DRV8313_IN1   PA1   /* TIM1 */
#define SBGC_DRV8313_IN2   PA2   /* TIM2 */
#define SBGC_DRV8313_IN3   PA3   /* TIM2 */
#define SBGC_DRV8313_EN123 PB10

/* It does not look like there's any Rsense connected between DRV8313's PGND{1,2,3} and GND */
/* TODO: current and temperature sensing per motor */

/* TODO: save in flash */
/* 'm' to autocalibrate and print new values.  Zero .pole_pairs will trigger calibration on power-on */
static const struct obgc_motor_calib_data_s motor_calib[3] = {
    { .bldc_with_encoder = { .pole_pairs = 11, .zero_electric_offset = 146.7, .sensor_direction = 1 } },  /* Yaw   (axis 0) */
    { .bldc_with_encoder = { .pole_pairs = 11, .zero_electric_offset = 91.11, .sensor_direction = -1 } }, /* Pitch (axis 2) */
    { .bldc_with_encoder = { .pole_pairs = 11, .zero_electric_offset = 64.64, .sensor_direction = 1 } },  /* Roll  (axis 1) */
};

/* Keep SimpleFOC support as a backup.  SimpleFOC doesn't autodetect .pole_pairs so they come from the user */
#define SBGC_MOTOR0_PAIRS  11
static const struct obgc_motor_calib_data_s sfoc_motor0_calib = { .bldc_with_encoder = { SBGC_MOTOR0_PAIRS, 3.7, 1 } };

static obgc_imu *main_imu;
static obgc_ahrs *main_ahrs;
static obgc_imu *frame_imu;
static obgc_ahrs *frame_ahrs;
static sbgc32_i2c_drv *drv_modules[3];
static obgc_encoder *encoders[3];
static obgc_foc_driver *motor_drivers[3];
static obgc_motor *motors[3];
static TwoWire *i2c;
static HardwareSerial *serial;

static bool use_motor[3], set_use_motor;
static obgc_motor_bldc_param set_param = __BLDC_PARAM_MAX;
static int set_param_power = -1;

static struct axes_data_s axes;
static bool have_axes;

static struct control_data_s control;
static bool control_enable;
static bool have_home;
static bool have_forward;

static float rel_q[4];   /* aux: true or estimated (from encoders) expression of main_ahrs->q in frame_ahrs->q frame of ref */
static float frame_q[4]; /* aux: frame_ahrs->q or its estimation from main_ahrs->q x rel_q^-1 */

static int vbat;
static int vbat_ok;
static bool motors_on;

static struct main_loop_cb_s *cbs;

#define TARGET_LOOP_RATE 128

void print_mcu() {
    /*
     * Specific register addresses for F3 in RM0316, F4 RM0090:
     *  0x1ffff7ac or UID_BASE: Unique Device ID
     *  0x1fffd7cc or FLASHSIZE_BASE: Flash Size
     *  0xe0042000 or DBGMCU->IDCODE/DBGMCU_BASE: MCU Device ID
     * But use the HAL functions where possbile
     */
    uint32_t uid[3] = {
        HAL_GetUIDw0(),
        HAL_GetUIDw1(),
        HAL_GetUIDw2(),
    };

    serial->print("MCU ID: ");
    serial->print(DBGMCU->IDCODE & 0xfff, HEX);
    serial->print(", REV: ");
    serial->print(DBGMCU->IDCODE >> 16, HEX);
    serial->print(", Unique-dev-ID: ");
    serial->print(uid[0], HEX);
    serial->print(", Flash size: ");
    serial->print((*(uint32_t *) FLASHSIZE_BASE) & 0xffff);
    serial->print("kB, CPUID: ");
    serial->println(SCB->CPUID, HEX);
}

void scan_i2c() {
    serial->println("Scanning I2C bus...");
    uint8_t found = 0;
    for (uint8_t address = 1; address < 127; address++) {
        i2c->beginTransmission(address);
        uint8_t error = i2c->endTransmission();

        if (error == 0) {
            serial->print("Device found at 0x");
            if (address < 16)
                serial->print("0");
            serial->println(address, HEX);
            found++;
        }
    }

    if (found == 0) {
        serial->println("No I2C devices found!");
    } else {
        serial->print(found);
        serial->println(" device(s) found");
    }
}

#define PROBE_PINS_NUM 12
static const uint8_t probe_pins[PROBE_PINS_NUM] = {
    PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB10, PB11
};

static const char *probe_pin_names[PROBE_PINS_NUM] = {
    "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PB0", "PB1", "PB2", "PB10", "PB11"
};

static int current_low_pin = -1;  /* Start with no pin forced LOW */
const unsigned long high_duration = 2000;
const unsigned long low_duration = 1000;

void probe_pins_setup() {
    for (int i = 0; i < PROBE_PINS_NUM; i++) {
        pinMode(probe_pins[i], OUTPUT);
        digitalWrite(probe_pins[i], LOW);
    }
}

void probe_pins_update() {
    unsigned long now = millis();
    uint8_t level = (now % (high_duration + low_duration)) > high_duration;

#if 0
    if (serial->read() < 0)
        return;

    if (current_low_pin >= PROBE_PINS_NUM - 1)
        return;

    serial->print("Enabling ");
    serial->print(probe_pin_names[++current_low_pin]);
    serial->println(" and setting it LOW");
    delay(100);
    pinMode(probe_pins[current_low_pin], OUTPUT);
    digitalWrite(probe_pins[current_low_pin], LOW);
    return;
#endif

    if (serial->available() > 0) {
        serial->read();  /* Read and discard the character */
        current_low_pin = (current_low_pin + 1) % PROBE_PINS_NUM;
        serial->print("Now forcing ");
        serial->print(probe_pin_names[current_low_pin]);
        serial->println(" to constant LOW");
    }

    for (int i = 0; i < PROBE_PINS_NUM; i++)
        digitalWrite(probe_pins[i], current_low_pin == i ? LOW : level);
}

void imu_debug_update() {
    int32_t accel[3], gyro[3], temp;

    main_imu->cls->read_main(main_imu, accel, gyro);
    main_imu->cls->read_temp(main_imu, &temp);

    /* Convert to physical values */
    float ax = (float) accel[0] / main_imu->cls->accel_scale;
    float ay = (float) accel[1] / main_imu->cls->accel_scale;
    float az = (float) accel[2] / main_imu->cls->accel_scale;

    float gx = (float) gyro[0] / main_imu->cls->gyro_scale;
    float gy = (float) gyro[1] / main_imu->cls->gyro_scale;
    float gz = (float) gyro[2] / main_imu->cls->gyro_scale;

    serial->print("Accel: ");
    serial->print(ax); serial->print("g, ");
    serial->print(ay); serial->print("g, ");
    serial->print(az); serial->println("g");

    serial->print("Gyro: ");
    serial->print(gx); serial->print("°/s, ");
    serial->print(gy); serial->print("°/s, ");
    serial->print(gz); serial->println("°/s");

    serial->print("Temp: ");
    serial->print(temp / 65536.0f);
    serial->println("°C\n");
}

bool quiet = 0;

static void main_ahrs_debug_print(const char *str) {
    if (quiet)
        return;

    serial->print("[");
    serial->print(micros());
    serial->print("][main] ");
    serial->println(str);
}

static void main_ahrs_from_encoders_debug(void) {
    float q[4], q_gyr[4], angles[3];
    char buf[128];
    static float q_prev[4] = { 1, 0, 0, 0 };

    if (quiet || !have_axes)
        return;

    /* We want the global main_ahrs->q estimated from frame_ahrs + rel_q (encoders).
     * If no frame_ahrs assume it was stationary.
     */
    if (frame_ahrs)
        quaternion_mult_to(frame_ahrs->q, rel_q, q);
    else
        memcpy(q, rel_q, 4 * sizeof(float));

    q_prev[0] = -q_prev[0];
    quaternion_mult_to(q_prev, q, q_gyr);
    quaternion_to_rotvec(q_gyr, angles);
    memcpy(q_prev, q, 4 * sizeof(float));

    sprintf(buf, "est_gyr %.5f %.5f %.5f", angles[0] * R2D, angles[1] * R2D, angles[2] * R2D);
    main_ahrs_debug_print(buf);
}

static void calibrate_print(const char *str) {
    serial->print("[");
    serial->print(micros());
    serial->print("][calib] ");
    serial->print(str);
}

void main_loop_sleep(void) {
    static unsigned long next_update = 0;
    unsigned long now = micros();

#define TARGET_INTERVAL (1e6 / TARGET_LOOP_RATE)

    if (next_update - now > TARGET_INTERVAL)
        next_update = now;
    else
        delayMicroseconds(next_update - now);

    next_update += TARGET_INTERVAL;
}

static void setup_control(void) {
    control.main_ahrs = main_ahrs;
    control.frame_ahrs = frame_ahrs;
    control.encoders = encoders;
    control.motors = motors;
    control.axes = &axes;

    control.dt = 1.0f / TARGET_LOOP_RATE;
    control.rel_q = rel_q;
    control.frame_q = frame_q;

    /* Setting defaults */
    control.keep_yaw = true;
    control.follow[0] = true;  /* Yaw only */
    control.follow[1] = false;
    control.follow[2] = false;
    control.max_accel = 30 * D2R; /* rad/s/s */
    control.max_vel = 60 * D2R;   /* rad/s */

    /* As we apply .max_accel, i.e. the max change in velocity from current value to get to the
     * rotational speed we need to get from current camera orientation to the desired one (target),
     * we depend on the whole feedback look through the motor PID up to the IMU and AHRS to
     * propagate and reflect the speed changes we command with little delay.  If there's any delay,
     * we'll be applying the acceleration limit to a velocity we commanded some iterations ago and
     * effectively the rate of acceleration will be much lower than what is requested.  And there's
     * an inherent delay in the sensors.  And a delay in that the current omega the AHRS sees is
     * the difference between the current and the last iteration i.e. the average speed over this
     * period (if we use the IMU FIFO), and this short period is the time the motor PID has only
     * started to apply the new torque so it cannot immediately show the speed we commanded.
     *
     * So depending on how well the loop is tuned, set this higher (up to 1.0) to fully depend on
     * the current physical rotiation rate from the gyro or lower and especially low when in
     * development or tuning.  control.c will replace the remaining portion of the velocity from
     * the gyro with the velocity it itself has most recently commanded, at the cost of responding
     * more slowly to speed changes due to outside physical forces.  This may even be desired but
     * consider things like mechanical limits on some joints.
     */
    control.ahrs_velocity_kp = 0.05;
}

static void stop_control(void) {
    int i;

    if (!control_enable)
        return;

    control_enable = false;

    for (i = 0; i < 3; i++)
        if (motors[i] && use_motor[i])
            motors[i]->cls->set_velocity(motors[i], 0);

    serial->println("Control off");
}

extern HardwareSerial *error_serial;

void setup(void) {
    int i;

    error_serial = serial = new HardwareSerial(USART1);
    serial->begin(115200);
    while (!*serial); /* Wait for serial port connection */
    delay(2000);
    serial->println("Initializing");

    /* Initialize I2C */
    i2c = new TwoWire(SBGC_SDA, SBGC_SCL);
    i2c->begin();
    i2c->setClock(400000); /* 400kHz I2C */

    /* Board debug info */
    // scan_i2c();
    print_mcu();
    // probe_pins_setup();

    pinMode(SBGC_LED_GREEN, OUTPUT);
    pinMode(SBGC_VBAT, INPUT);

    /* Initialize IMUs */
    main_imu = mpu6050_new(MPU6050_DEFAULT_ADDR, i2c);
    if (!main_imu) {
        serial->println("Main MPU6050 initialization failed!");
        while (1);
    }

    // mpu6050_set_srate(main_imu, 8000 / TARGET_LOOP_RATE - 1, 0); /* No DLPF */
    /* The above minimzes number of samples to read, the below seems to reduce noise */
    mpu6050_set_srate(main_imu, 3, 0); /* 2000 Hz, no DLPF */

    serial->println("Main MPU6050 initialized!");

    main_ahrs = ahrs_new(main_imu, SBGC_IMU_X, SBGC_IMU_MINUS_Z);
    ahrs_set_weights(main_ahrs, 0.5f, 0.05f, 0.5f, M_PI / 0x800);
    ahrs_set_debug(main_ahrs, main_ahrs_debug_print);
    delay(100);
    ahrs_calibrate(main_ahrs);
    serial->println("Main AHRS initialized!");
    /* TODO: make AHRS an abstract class, then convert the current AHRS to its subclass and add another that exposes the mpu6050
     * directly as an AHRS with DMP enabled? would only need to keep an adjustment quaternion that corrects for yaw and positions
     * from encoders?? */

    /* TODO: frame_ahrs -- not crucial though, we fill in for it with encoder data and forget about yaw drift */

    drv_modules[0] = sbgc32_i2c_drv_new(SBGC32_I2C_DRV_ADDR(1), i2c, SBGC32_I2C_DRV_ENC_TYPE_AS5600);
    drv_modules[1] = sbgc32_i2c_drv_new(SBGC32_I2C_DRV_ADDR(4), i2c, SBGC32_I2C_DRV_ENC_TYPE_AS5600);
    if (drv_modules[0] && drv_modules[1])
        serial->println("SBGC32_I2C_Drv initialized");
    else
        serial->println("SBGC32_I2C_Drv init failed");

    encoders[0] = as5600_new(i2c);
    encoders[1] = sbgc32_i2c_drv_get_encoder(drv_modules[0]);
    encoders[2] = sbgc32_i2c_drv_get_encoder(drv_modules[1]);
    serial->println("Encoders initialized");

    /*
     * We stopped using the SimpleFOC high-level motor abstraction so that we can use common code for
     * both the on-board motor driver (which SimpleFOC can handle), if any, and the motors connected to
     * SBGC32_I2C_Drv extension boards (which SimpleFOC cannot handle out of the box), again, if any.
     *
     * For the on-board driver we still use SimpleFOC's low-level abstraction to give us the following:
     *   * STM32 PWM output timer setup,
     *   * the PWM output sine modulation, or one of the other 3 modes available.
     *
     * Keep the code below in case we need to cross check something against SimpleFOC.
     */
#if 0
# define MOTOR_DEBUG
# ifdef MOTOR_DEBUG
    SimpleFOCDebug::enable(serial);
# endif
    /* SimpleFOC as a full motor object */
    motors[0] = motor_3pwm_new(SBGC_DRV8313_IN1, SBGC_DRV8313_IN2, SBGC_DRV8313_IN3, SBGC_DRV8313_EN123,
            encoders[0], &sfoc_motor0_calib);
#endif

    /* SimpleFOC as a PWM output driver only */
    motor_drivers[0] = motor_drv_3pwm_new(SBGC_DRV8313_IN1, SBGC_DRV8313_IN2, SBGC_DRV8313_IN3, SBGC_DRV8313_EN123);
    if (!motor_drivers[0])
        serial->println("Motor 0 driver init failed!");

    motor_drivers[1] = sbgc32_i2c_drv_get_motor_drv(drv_modules[0]);
    motor_drivers[2] = sbgc32_i2c_drv_get_motor_drv(drv_modules[1]);

    for (i = 0; i < 3; i++) {
        motors[i] = motor_bldc_new(encoders[i], motor_drivers[i],
                motor_calib[i].bldc_with_encoder.pole_pairs ? &motor_calib[i] : NULL);

        motor_bldc_set_param(motors[i], BLDC_PARAM_KP, i ? 0.03f : 0.06f);
        motor_bldc_set_param(motors[i], BLDC_PARAM_KI, i ? 0.01f : 0.03f);
        motor_bldc_set_param(motors[i], BLDC_PARAM_KD, 0.001f); /* Look 0.001s ahead */
        motor_bldc_set_param(motors[i], BLDC_PARAM_KI_FALLOFF, 0.005f);
        motor_bldc_set_param(motors[i], BLDC_PARAM_V_MAX, i ? 0.3f : 1.0f); /* Limit to 0.3 x VBAT */
    }

    serial->println("Motors early init done");

    setup_control();
}
void setup_end(void) {}

static void motors_on_off(bool on) {
    if (motors_on == on)
        return;

    if (!on)
        stop_control();

    for (int i = 0; i < 3; i++) {
        if (!motors[i] || !use_motor[i])
            continue;

        if (on) {
            if (!motors[i]->ready || motors[i]->cls->on(motors[i]) != 0) {
                for (int j = 0; j < i; j++)
                    if (motors[j])
                        motors[j]->cls->off(motors[j]);
                serial->print("Motor ");
                serial->print(i);
                serial->println(" power on failed!");
                return;
            }
        } else
            motors[i]->cls->off(motors[i]);
    }

    motors_on = on;
    /* TODO: beep */
}

static void shutdown_to_bl(void) __attribute__((noreturn));
static void shutdown_to_bl(void) {
    int i;

    motors_on_off(false);

    /* Things we set up explicitly (TODO: steal_ptr() syntax) */
    /* TODO: refcounting would be nice too */
    ahrs_free(main_ahrs);
    main_imu->cls->free(main_imu);
    for (i = 0; i < 3; i++)
        if (motors[i])
            motors[i]->cls->free(motors[i]);
    for (i = 0; i < 3; i++)
        if (encoders[i])
            encoders[i]->cls->free(encoders[i]);
    for (i = 0; i < 3; i++)
        if (motor_drivers[i])
            motor_drivers[i]->cls->free(motor_drivers[i]);
    for (i = 0; i < 3; i++)
        if (drv_modules[i])
            sbgc32_i2c_drv_free(drv_modules[i]);
    i2c->end();
    serial->end();
    digitalWrite(SBGC_LED_GREEN, 0);
    pinMode(SBGC_LED_GREEN, INPUT);

    /* Things arduino, PlatformIO, framework, CMSIS, etc. may have set up */
    __disable_irq();
    USB->CNTR = 0x0003; /* Reset USB in case it is used (not on PilotFly H2) */
    HAL_RCC_DeInit();
    SysTick->CTRL = 0;  /* Stop clock after DeInit which may have used it */
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /* Reset interrupts */
    for (int i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
        NVIC->ICER[i] = 0xffffffff;
        NVIC->ICPR[i] = 0xffffffff;
    }

    /*
     * STM32F303xB bootloader System Memory start address according to Section 23 in ST Application Note AN2606, Table 28.
     * Same for all F3 series but not most other STM32s.  For other models, look at the same doc, all models summarized in
     * Section 92, Table 209.
     */
    __set_MSP(*(uint32_t *) 0x1fffd800);
    __set_PSP(*(uint32_t *) 0x1fffd800);
    ((void (*)(void)) *(uint32_t *) 0x1fffd804)();
    while (1); /* Silence warning */
}

static void powered_init(void) {
    for (int i = 0; i < 3; i++) {
        if (!motors[i] || !use_motor[i] || motors[i]->ready)
            continue;

        if (motors[i]->cls->powered_init(motors[i]) != 0) {
            serial->print("Motor ");
            serial->print(i);
            serial->println(" powered init failed!");
        }
    }

    serial->println("Motors powered init done");
}

static void blink(void) {
    static uint8_t led_val = 0;

    led_val ^= 1;
    digitalWrite(SBGC_LED_GREEN, led_val);
}

static void vbat_update(void) {
    uint16_t raw = analogRead(SBGC_VBAT); /* TODO: noisy, use multiple samples */
    static int vbat_prev = 0;
    static int lvco = 0;
    static unsigned long vbat_ts = 0;
    static unsigned long msg_ts = 0;
    unsigned long now = millis();

    vbat = (uint64_t) raw * 3300/*mV*/ * (SBGC_VBAT_R_BAT + SBGC_VBAT_R_GND) / (4095 * SBGC_VBAT_R_GND) * SBGC_VBAT_SCALE;
    vbat_ok = lvco > 0 && vbat > lvco;

    /* TODO: Allow user to set min/max alarm voltages, fall back to the below if unset */
    /* TODO: scale calibration? */
    if (!lvco) {
        if (vbat > 3000) {
            if (!vbat_ts)
                vbat_ts = now;
            else if (now - vbat_ts >= 10000) {
                /* This calc works roughly for 1-6S Li-Po packs if between 3.6 and 4.4V (Li-HV) per cell at startup */
                int cells = vbat / (vbat < 10000 ? 4500 : 4400) + 1;
                lvco = cells * 3300;
                vbat_ts = 0;
                serial->print("VBAT low-voltage cutoff set at ");
                serial->print((lvco / 100) * 0.1f);
                serial->println("V (guessed)");
                powered_init();
                goto print_update;
            }
        } else
            vbat_ts = 0;
    } else if (vbat <= lvco) {
        lvco = -1; /* Only do this once */
        serial->println("VBAT low, disabling motors!");
        motors_on_off(false);
        /* TODO: play a sound? is there any data to save? */
        goto print_update;
    }

    if (now - msg_ts < 2000)
        return;

    if (abs(vbat - vbat_prev) < 400)
        return;

print_update:
    vbat_prev = vbat;
    msg_ts = now;
    serial->print("VBAT now at ");
    serial->print((vbat / 100) * 0.1f);
    serial->println("V");
}

void loop(void) {
    int i;

    main_loop_sleep();

    /* Run all the update functions here in the right order.  Each one calculates something that one of the other
     * functions that follows it needs.  Eventually the final result of all the calculations is what gets output
     * to the motor drivers.  The motor-bldc.c code uses main_loop_cb_add() to schedule its update functions to
     * be called so we call them indirectly through the callback list.
     *
     * It might sound like a good idea to move all of these steps here to the callback list.  The problem with
     * this is that main_loop_cb_add() would need to either be called in a very specific order, or take an extra
     * parameter to tell it when each callback is expected to run, which would be a mess because the values would
     * need to be very specific.  So don't do that, let only the motors and any extra, non-critical tasks use the
     * callback list.
     *
     * The alternative would be to just have a (probably static) array of functions to call, which is basically
     * the same thing as calling all of them directly in a long flat function like we do here.  Nevertheless it
     * would be good to document what each function requires/depends on (this quaternion, that quaternion,
     * a sensor reading) and what it provides.
     *
     * Another idea is, instead of a simple main loop -- which forces all of the update functions to run at the
     * same frequency -- let's have a queue of callbacks that need to happen where things are added or re-added
     * every time they're needed instead of once in setup().  It would also have to allow for something to be
     * waited on, like I2C transfers.  This would allow some of the noisy sensors, like the VBAT ADC, to be
     * sampled more often for resampling, and the critical feedback loops, like the one in motor-bldc.c, to run
     * more frequently.  At the same time it would be synchronous and not have the issues of timer interrupts
     * where HW access would need locking.
     *
     * Our single I2C bus is really the single bottle-neck resource that everyone needs on the SimpleBGC
     * reference gimbal and is the factor deciding who can run and who has to wait if we add any type of
     * asynchronicity or queuing.
     */

    /* These are the only users of the IMUs so they perform the reading internally */
    ahrs_update(main_ahrs);

    if (frame_ahrs)
        ahrs_update(frame_ahrs);

    /* Read the encoders for everyone, there are multiple users */
    for (i = 0; i < 3; i++)
        encoder_update(encoders[i]);

    if (have_axes)
        axes_precalc_rel_q(&axes, encoders, main_ahrs->q, rel_q, frame_q);

    if (control_enable)
        control_step(&control);

    blink(); /* TODO: convert to cbs */
    vbat_update();

    // imu_debug_update();
    // probe_pins_update();

    if (main_ahrs->debug_print && !main_ahrs->debug_cnt)
        main_ahrs_from_encoders_debug();

    for (struct main_loop_cb_s *entry = cbs; entry; entry = entry->next)
        entry->cb(entry->data);

    if (serial->available()) {
        uint8_t cmd = serial->read();
        int i, param;
        struct axes_calibrate_data_s cs;
        static uint8_t dlpf = 0;

        switch (cmd) {
        case 'q':
            serial->println("Shutting down and jumping to bootloader");
            shutdown_to_bl();
            break;
        case 'Q':
            motors_on_off(false);
            serial->println("Crashing in loop()"); /* Crash handler test */
            delay(100);
            *(uint8_t *) -1 = 5;
            break;
        case '0' ... '5':
            if (set_use_motor) {
                set_use_motor = false;

                if (cmd >= '3')
                    break;

                use_motor[cmd - '0'] ^= 1;
                /* TODO: could ensure motors[cmd - '0'] is non-NULL so we could skip checks everywhere else */
                serial->print("use_motor = { ");
                serial->print(use_motor[0]);
                serial->print(use_motor[1]);
                serial->print(use_motor[2]);
                serial->println(" }");
                break;
            }

            if (set_param < __BLDC_PARAM_MAX) {
                float val;
handle_set_param:

                if (set_param_power == -1) {
                    set_param_power = cmd - '0';
                    break;
                }

                /*
                 * Type 'P', 'I' or 'D' followed by a power/exponent value (0 to 9) and a multiplier (0 to 9).
                 * For now the result is multiplier * (10 ^ (-power / 2)).  E.g. P05 sets the Kp to 5.0.
                 * P25 sets it to 0.5, P45 to 0.05 and so on.  There's some redundancy here but we don't care.
                 */
                val = powf(10, set_param_power * -0.5f) * (cmd - '0');

                for (i = 0; i < 3; i++)
                    if (motors[i] && use_motor[i]) {
                        motor_bldc_set_param(motors[i], set_param, val);
                        serial->print("Param set to ");
                        serial->println(val);
                    }

                set_param = __BLDC_PARAM_MAX;
                set_param_power = -1;
                break;
            }

            motors_on_off(false);
            serial->println("Setting new MPU6050 clksource");
            mpu6050_set_clksrc(main_imu, cmd - '0');
            delay(100);
            break;
        case '6' ... '9':
            if (set_param < __BLDC_PARAM_MAX)
                goto handle_set_param;

            motors_on_off(false);
            serial->println("Setting new MPU6050 sample rate");
            /* Sample rate becomes (dlpf ? 8k : 1k) / (param + 1) */
            mpu6050_set_srate(main_imu, (cmd - '6') ? 1 << 2 * (cmd - '6') : 0, dlpf);
            delay(100);
            break;
        case '*':
            dlpf ^= 2;
            serial->print("Will use MPU6050 DLPF setting ");
            serial->print(dlpf);
            serial->println(" on next sample rate change");
            break;
        case 'e':
            {
                float angles[3];

                /* Read encoder angles */
                for (i = 0; i < 3; i++) {
                    float scale;

                    if (!encoders[i])
                        continue;

                    scale = have_axes ? axes.encoder_scale[i] : 1.0f;
                    angles[i] = encoders[i]->reading * scale + (scale < 0 ? 360 : 0);

                    serial->print("Encoder ");
                    serial->print(i);
                    serial->print(" angle = ");
                    serial->println(angles[i]);
                }

                if (have_axes) {
                    float q[4], conj_rel_q[4] = INIT_CONJ_Q(rel_q);
                    float angles_est[3], mapped[3];

                    /* Only frame_ahrs->q, can't use frame_q instead because that is calculated from encoder info */
                    if (frame_ahrs) {
                        float conj_frame_q[4] = INIT_CONJ_Q(frame_ahrs->q);
                        quaternion_mult_to(conj_frame_q, main_ahrs->q, q);
                    } else
                        memcpy(q, main_ahrs->q, 4 * sizeof(float));

                    /* Try to estimate the same angles from IMU data */
                    axes_q_to_angles(&axes, q, angles_est);
                    mapped[axes.axis_to_encoder[0]] = angles_est[0];
                    mapped[axes.axis_to_encoder[1]] = angles_est[1];
                    mapped[axes.axis_to_encoder[2]] = angles_est[2];
                    serial->print("Estimated = ");
                    serial->print(mapped[0] * R2D);
                    serial->print(", ");
                    serial->print(mapped[1] * R2D);
                    serial->print(", ");
                    serial->println(mapped[2] * R2D);

                    /* Now try to estimate main_ahrs->q from encoder angles */
                    quaternion_mult_to(main_ahrs->q, conj_rel_q, q);
                    serial->print("Q x Q_est^-1 error angle = ");
                    serial->println(2 * acosf(q[0]) * R2D);
                }

                break;
            }
        case 'c':
            motors_on_off(false);
            serial->println("Recalibrating main AHRS");
            delay(100);
            ahrs_calibrate(main_ahrs);
            break;
        case 'C':
            motors_on_off(false);
            /*
             * (Re-)Detect/calibrate motor/encoder order, axes, neutral angles (offsets), directions and scales.
             * We'll ask user to move the gimbal in a specific way so we can detect the exact orientation of each joint's axis
             * with relation to the arm it's mounted on or to the base.  We'll also detect which of the three motors/encoders
             * the software sees maps to which joint.
             */
            serial->println("Recalibrating arm/joint setup: motor/encoder order, axes, neutral angles, direction and scale");
            cs.main_ahrs = main_ahrs;
            cs.frame_ahrs = frame_ahrs;
            cs.encoders = encoders;
            cs.print = calibrate_print;
            cs.out = &axes;

            /* axes_calibrate() runs its own main loop, quiet our ahrs debug info */
            quiet = 1;
            have_axes = !axes_calibrate(&cs);
            quiet = 0;

            serial->println(axes.axes[0][0]);////
            serial->println(axes.axes[0][1]);
            serial->println(axes.axes[0][2]);
            serial->println(axes.axes[1][0]);
            serial->println(axes.axes[1][1]);
            serial->println(axes.axes[1][2]);
            serial->println(axes.axes[2][0]);
            serial->println(axes.axes[2][1]);
            serial->println(axes.axes[2][2]);
            serial->println(axes.main_imu_mount_q[0]);
            serial->println(axes.main_imu_mount_q[1]);
            serial->println(axes.main_imu_mount_q[2]);
            serial->println(axes.main_imu_mount_q[3]);////
            break;
        case 'k':
            serial->println("Saving current camera head orientation as home orientation (0-pitch, 0-roll)"); /* And 0-yaw if not following */
            memcpy(control.home_q, main_ahrs->q, sizeof(control.home_q));
            memcpy(control.home_frame_q, frame_q, sizeof(control.home_frame_q));
            have_home = 1;
            break;
        case 'K':
            if (!have_home) {
                serial->println("Set home orientation first ('k')");
                break;
            }

            /*
             * Here we expect the user (TODO: document/print this to user somewhere) to roll the camera anywhere between 30 and 120 deg
             * to the *right* after setting the home orientation ('k') and then send 'K' to set a "forward" direction.  None of the other data
             * we have available so far really tells us what the user considers the forward/front direction and consequently the pitch vs.
             * the roll axis.  Yaw is easy because it's aligned with gravity.  Roll and pitch axes are perpendicular to yaw axis (i.e.
             * within horizontal plane) but since we don't assume anything about the order of joint axes or the orientation of either IMU,
             * there's just no way to know where camera's front is, or whatever device/tool we're orienting.
             *
             * So we ask for a roll motion.  The axis of rotation is going to be the forward-back axis (camera lens axis) and if the roll is
             * to the right, the axis will point forward due to the right-hand rule.  We save that and now we know how to decompose any
             * orientation into a camera yaw+pitch+roll, or compose a set of camera yaw+pitch+roll values into a specific main IMU orientation.
             *
             * The camera should point in the exact same direction as when home orientation was set, only rolled.  The user can perhaps
             * have the camera on and confirm on live view with OSD crosshair or similar, even zoomed in.
             */
            have_forward = 0;

            {
                float angle;
                float conj_q0[4] = INIT_CONJ_Q(control.home_q);
                float roll_q[4];

                quaternion_mult_to(main_ahrs->q, conj_q0, roll_q);
                quaternion_to_axis_angle(roll_q, control.forward_vec, &angle);

                if (angle < M_PI / 6 || angle > M_PI * (2.0f / 3)) {
                    serial->println("No rotation within 30-120 deg detected");
                    break;
                }
            }

            if (fabsf(control.forward_vec[0]) + fabsf(control.forward_vec[1]) < 0.001f)
                break;

            if (fabsf(control.forward_vec[2]) > 0.1f)
                serial->println("WARN: rotation axis not very level");

            serial->println("Saving the rotation axis as the forward direction");
            control.forward_vec[2] = 0.0f;
            vector_normalize(control.forward_vec);
            control.forward_az = atan2f(control.forward_vec[1], control.forward_vec[0]);
            have_forward = 1;
            break;
        case 't':
            set_use_motor = true;
            break;
        case 'S':
            /* TODO: also ensure have_axes before we power anything on */
            if (!motors[0] && !motors[1] && !motors[2]) {
                serial->println("We have no motors");
                break;
            }

            for (i = 0; i < 3; i++)
                if (motors[i] && use_motor[i] && !motors[i]->ready) {
                    serial->print("Motor ");
                    serial->print(i);
                    serial->println(" not ready");
                    break;
                }
            if (i < 3)
                break;

            for (i = 0; i < 3; i++)
                if (motors[i] && use_motor[i])
                    motors[i]->cls->set_velocity(motors[i], 0);

            motors_on_off(true);
            serial->println("Motors on");
            break;
        case 's':
            motors_on_off(false);
            serial->println("Motors off");
            break;
        case 'P':
            set_param = BLDC_PARAM_KP;
            break;
        case 'I':
            set_param = BLDC_PARAM_KI;
            break;
        case 'D':
            set_param = BLDC_PARAM_KD;
            break;
        case 'm':
            if (motors_on || !vbat_ok) {
                serial->println("Motors must be off and VBAT good");
                break;
            }

            for (i = 0; i < 3; i++) {
                struct obgc_motor_calib_data_s data;

                if (!motors[i] || !use_motor[i])
                    continue;

                if (motors[i]->cls->recalibrate(motors[i]) != 0)
                    serial->println("Motor calibration failed");
                else if (motors[i]->cls->get_calibration(motors[i], &data) != 0)
                    serial->println("Motor calibration no data");
                else {
                    char msg[200];
                    sprintf(msg, "Motor %i calibration = { .pole_pairs = %i, .zero_electric_offset = %f, .sensor_direction = %i }",
                            i, data.bldc_with_encoder.pole_pairs, data.bldc_with_encoder.zero_electric_offset,
                            data.bldc_with_encoder.sensor_direction);
                    serial->println(msg);
                }
            }

            break;
        case ' ':
            if (control_enable) {
                stop_control();
                break;
            }

            if (!have_axes || !have_home || !have_forward || !motors_on || !vbat_ok) {
                serial->println("Motors must be on and calibration complete");
                break;
            }

            control_enable = true;
            serial->println("Control on");
            break;
        case 27:
            if (!serial->available())
                delay(5);
            if (!serial->available())
                break;
            if (serial->read() != '[') /* Not an ANSI CSI sequence */
                break;
            /* Parse first parameter */
            param = 0;
            while (1) {
                cmd = serial->read();
                if (cmd < '0' || cmd > '9')
                    break;
                param = param * 10 + (cmd - '0');
            }
            /* Ignore everything until CSI command final byte */
            while (cmd < 0x40 || cmd > 0x7e)
                cmd = serial->read();

            switch (cmd) {
            case 'D': /* Cursor Back or left arrow, param is modifier */
                if (motors[0])
                    motors[0]->cls->set_velocity(motors[0], -5);
                break;
            case 'C': /* Cursor Forward or right arrow, param is modifier */
                if (motors[0])
                    motors[0]->cls->set_velocity(motors[0], 5);
                break;
            case 'A': /* Cursor Up or up arrow, param is modifier */
                if (motors[1])
                    motors[1]->cls->set_velocity(motors[1], -5);
                break;
            case 'B': /* Cursor Down or down arrow, param is modifier */
                if (motors[1])
                    motors[1]->cls->set_velocity(motors[1], 5);
                break;
            case 'H': /* Cursor Position or Home */
            case 'F': /* Cursos Previous Line or End */
                break;
            case '~': /* Private sequence: xterm keycode sequence, param is keycode */
                switch (param) {
                case 5: /* Page Up */
                    if (motors[2])
                        motors[2]->cls->set_velocity(motors[2], 5);
                    break;
                case 6: /* Page Down */
                    if (motors[2])
                        motors[2]->cls->set_velocity(motors[2], -5);
                    break;
                case 1: /* Home */
                case 7: /* Home */
                case 4: /* End */
                case 8: /* End */
                    break;
                default:
                    serial->print("Unknown xterm keycode ");
                    serial->println(param);
                }
                break;
            default:
                serial->print("Unknown CSI seq ");
                serial->println(cmd);
            }
            break;
        default:
            serial->print("Unknown cmd ");
            serial->println(cmd);
        }
    }
}
void loop_end(void) {}

void main_loop_cb_add(struct main_loop_cb_s *cb) {
    struct main_loop_cb_s **ptr;

    for (ptr = &cbs; *ptr; ptr = &(*ptr)->next);

    *ptr = cb;
}

void main_loop_cb_remove(struct main_loop_cb_s *cb) {
    struct main_loop_cb_s **ptr = &cbs;

    while (*ptr) {
        if (*ptr == cb)
            *ptr = (*ptr)->next;
        else
            ptr = &(*ptr)->next;
    }

    cb->next = NULL;
}
