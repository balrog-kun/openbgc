/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

#include "imu-mpu6050.h"
#include "sbgc32_i2c_drv.h"
#include "encoder-as5600.h"
extern "C" {
#include "motor-pwm.h"
#include "ahrs.h"
#include "axes.h"
#include "moremath.h"

#include "main.h"
}

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

#define SBGC_MOTOR0_PAIRS  11
/* It does not look like there's any Rsense connected between DRV8313's PGND{1,2,3} and GND */
/* TODO: current and temperature sensing for yaw motor */

static sbgc_imu *main_imu;
static sbgc_ahrs *main_ahrs;
static sbgc_imu *frame_imu;
static sbgc_ahrs *frame_ahrs;
static sbgc32_i2c_drv *drv_modules[2];
static sbgc_encoder *encoders[3];
static sbgc_motor *motors[3];
static TwoWire *i2c;
static HardwareSerial *serial;

static struct axes_data_s axes;
static bool have_axes;
static float home_q[4];
static bool have_home;

static int vbat;
static int vbat_ok;
static bool motors_on;

static struct motor_pwm_calib_data_s motor0_calib = { 3.7, 1 }; /* 'm' to autocalibrate and print new values */

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
    float q_tmp[4], q_est[4], q_axis[4], angles[3];
    char buf[128];
    static float q_est_prev[4] = { 1, 0, 0, 0 };

    if (quiet || !have_axes)
        return;

    /* Read encoder angles */
    for (int i = 0; i < 3; i++) {
         if (!encoders[i])
              continue;

         angles[i] = copysign((float) encoders[i]->cls->read(encoders[i]) / encoders[i]->cls->scale, axes.encoder_scale[i]);
    }

    quaternion_from_axis_angle(q_axis, axes.axes[2], angles[axes.axis_to_encoder[2]] * D2R);
    quaternion_mult_to(q_axis, axes.main_imu_mount_q, q_tmp);
    quaternion_normalize(q_tmp);
    quaternion_from_axis_angle(q_axis, axes.axes[1], angles[axes.axis_to_encoder[1]] * D2R);
    quaternion_mult_to(q_axis, q_tmp, q_est);
    quaternion_normalize(q_est);
    memcpy(q_tmp, q_est, sizeof(q_tmp));
    quaternion_from_axis_angle(q_axis, axes.axes[0], angles[axes.axis_to_encoder[0]] * D2R);
    quaternion_mult_to(q_axis, q_tmp, q_est);
    quaternion_normalize(q_est);

    q_est_prev[0] = -q_est_prev[0];
    quaternion_mult_to(q_est_prev, q_est, q_tmp);
    quaternion_to_rotvec(q_tmp, angles);
    memcpy(q_est_prev, q_est, sizeof(q_est));

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

extern HardwareSerial *error_serial;

void setup(void) {
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
    main_imu = sbgc_mpu6050_new(MPU6050_DEFAULT_ADDR, i2c);
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

    /* TODO: frame_ahrs */

    drv_modules[0] = sbgc32_i2c_drv_new(SBGC32_I2C_DRV_ADDR(1), i2c, SBGC32_I2C_DRV_ENC_TYPE_AS5600);
    drv_modules[1] = sbgc32_i2c_drv_new(SBGC32_I2C_DRV_ADDR(4), i2c, SBGC32_I2C_DRV_ENC_TYPE_AS5600);
    if (drv_modules[0] && drv_modules[1])
        serial->println("SBGC32_I2C_Drv initialized");
    else
        serial->println("SBGC32_I2C_Drv init failed");

    encoders[0] = sbgc_as5600_new(i2c);
    encoders[1] = sbgc32_i2c_drv_get_encoder(drv_modules[0]);
    encoders[2] = sbgc32_i2c_drv_get_encoder(drv_modules[1]);
    serial->println("Encoders initialized");

#define MOTOR_DEBUG
#ifdef MOTOR_DEBUG
    SimpleFOCDebug::enable(serial);
#endif
    motors[0] = sbgc_motor_pwm_new(SBGC_DRV8313_IN1, SBGC_DRV8313_IN2, SBGC_DRV8313_IN3, SBGC_DRV8313_EN123,
            SBGC_MOTOR0_PAIRS, encoders[0], &motor0_calib); /* Pass NULL to always auto-calibrate */
    if (!motors[0])
        serial->println("Motor 0 early init failed!");

    serial->println("Motors early init done");
}
void setup_end(void) {}

static void motors_on_off(bool on) {
    if (motors_on == on)
        return;

    for (int i = 0; i < 3; i++) {
        if (!motors[i])
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
    ahrs_free(main_ahrs);
    main_imu->cls->free(main_imu);
    for (i = 0; i < 3; i++)
        if (encoders[i])
            encoders[i]->cls->free(encoders[i]);
    for (i = 0; i < 3; i++)
        if (motors[i])
            motors[i]->cls->free(motors[i]);
    for (i = 0; i < 2; i++)
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
        if (!motors[i] || motors[i]->ready)
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
    main_loop_sleep();
    ahrs_update(main_ahrs);

    if (frame_ahrs)
        ahrs_update(frame_ahrs);

    if (main_ahrs->debug_print && !main_ahrs->debug_cnt)
        main_ahrs_from_encoders_debug();

    blink(); /* TODO: convert to cbs */
    vbat_update();

    for (struct main_loop_cb_s *entry = cbs; entry; entry = entry->next)
        entry->cb(entry->data);

    // imu_debug_update();
    // probe_pins_update();

    if (serial->available()) {
        uint8_t cmd = serial->read();
        int i, param;
        struct calibrate_data_s cs;
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
            motors_on_off(false);
            serial->println("Setting new MPU6050 clksource");
            mpu6050_set_clksrc(main_imu, cmd - '0');
            delay(100);
            break;
        case '6' ... '9':
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

                    scale = (have_axes ? axes.encoder_scale[i] : 1.0f) / encoders[i]->cls->scale;
                    angles[i] = (float) encoders[i]->cls->read(encoders[i]) * scale + (scale < 0 ? 360 : 0);

                    serial->print("Encoder ");
                    serial->print(i);
                    serial->print(" angle = ");
                    serial->println(angles[i]);
                }

                if (have_axes) {
                    float q[4], q_est[4], q_axis[4], q_tmp[4];
                    float angles_est[3], mapped[3];

                    if (frame_ahrs) {
                        float conj_frame_q[4] = INIT_CONJ_Q(frame_ahrs->q);
                        quaternion_mult_to(main_ahrs->q, conj_frame_q, q);
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
                    quaternion_from_axis_angle(q_axis, axes.axes[2], angles[axes.axis_to_encoder[2]] * D2R);
                    quaternion_mult_to(q_axis, axes.main_imu_mount_q, q_tmp);
                    quaternion_normalize(q_tmp);
                    quaternion_from_axis_angle(q_axis, axes.axes[1], angles[axes.axis_to_encoder[1]] * D2R);
                    quaternion_mult_to(q_axis, q_tmp, q_est);
                    quaternion_normalize(q_est);
                    memcpy(q_tmp, q_est, sizeof(q_tmp));
                    quaternion_from_axis_angle(q_axis, axes.axes[0], angles[axes.axis_to_encoder[0]] * D2R);
                    quaternion_mult_to(q_axis, q_tmp, q_est);
                    quaternion_normalize(q_est);

                    q_est[0] = -q_est[0];
                    quaternion_mult_to(main_ahrs->q, q_est, q_tmp);
                    /* TODO: frame_ahrs */
                    serial->print("Q x Q_est error angle = ");
                    serial->println(2 * acosf(q_tmp[0]) * R2D);
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
            serial->println("Saving current camera head orientation as home orientation (0-pitch, 0-roll)");
            memcpy(home_q, main_ahrs->q, sizeof(home_q));
            have_home = 1;
            break;
        case 'S':
            /* TODO: also ensure have_axes before we power anything on */
            if (!motors[0] && !motors[1] && !motors[2]) {
                serial->println("We have no motors");
                break;
            }

            for (i = 0; i < 3; i++)
                if (motors[i] && !motors[i]->ready) {
                    serial->print("Motor ");
                    serial->print(i);
                    serial->println(" not ready");
                    break;
                }
            if (i < 3)
                break;

            /* TODO: move to control */
            if (motors[0])
                motors[0]->cls->set_velocity(motors[0], 0);

            motors_on_off(true);
            serial->println("Motors on");
            break;
        case 's':
            motors_on_off(false);
            serial->println("Motors off");
            break;
        case 'm':
            if (motors[0]) {
                struct motor_pwm_calib_data_s data;

                if (motors_on && vbat_ok)
                    serial->println("Motors must be off and VBAT good");
                else if (!sbgc_motor_pwm_recalibrate(motors[0]))
                    serial->println("Motor 0 calibration failed");
                else if (!sbgc_motor_pwm_get_calibration(motors[0], &data))
                    serial->println("Motor 0 calibration no data");
                else {
                    char msg[200];
                    sprintf(msg, "Motor 0 calibration = { .zero_electric_offset = %f, .sensor_direction = %i }",
                            data.zero_electric_offset, data.sensor_direction);
                    serial->println(msg);
                }
            }

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
                    motors[0]->cls->set_velocity(motors[0], -5 * D2R);
                break;
            case 'C': /* Cursor Forward or right arrow, param is modifier */
                if (motors[0])
                    motors[0]->cls->set_velocity(motors[0], 5 * D2R);
                break;
            case 'A': /* Cursor Up or up arrow, param is modifier */
            case 'B': /* Cursor Down or down arrow, param is modifier */
            case 'H': /* Cursor Position or Home */
            case 'F': /* Cursos Previous Line or End */
                break;
            case '~': /* Private sequence: xterm keycode sequence, param is keycode */
                switch (param) {
                case 1: /* Home */
                case 7: /* Home */
                case 4: /* End */
                case 8: /* End */
                case 5: /* Page Up */
                case 6: /* Page Down */
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
