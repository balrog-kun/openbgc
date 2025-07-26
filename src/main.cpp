/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h>
#include <Wire.h>

#include "imu-mpu6050.h"
#include "encoder-as5600.h"
#include "encoder-sbgc32_i2c_drv.h"
extern "C" {
#include "ahrs.h"
#include "axes.h"
}

#define SBGC_LED_GREEN     PB12
/* Red LED apparently always-on, not software controllable */

#define SBGC_SDA           PA14
#define SBGC_SCL           PA15

#define SBGC_BAT           PA0  /* Through R17 (14k) to BAT+ and R18 to GND (4.7k)? */

#define SBGC_DRV8313_IN1   PB1
#define SBGC_DRV8313_IN2   PA2
#define SBGC_DRV8313_IN3   PA3
#define SBGC_DRV8313_EN123 PB10
/* It does not look like there's any Rsense connected between DRV8313's PGND{1,2,3} and GND */
/* TODO: current and temperature sensing for yaw motor */

static sbgc_imu *main_imu;
static sbgc_ahrs *main_ahrs;
static sbgc_imu *frame_imu;
static sbgc_ahrs *frame_ahrs;
static sbgc_encoder *encoders[3]; /* YPR */
static TwoWire *i2c;
static HardwareSerial *serial;

static struct axes_data_s axes;
static bool have_axes;
static float home_q[4];
static bool have_home;

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

static void calibrate_print(const char *str) {
    serial->print("[");
    serial->print(micros());
    serial->print("][calib] ");
    serial->print(str);
}

void main_loop_sleep(void) {
    static unsigned long next_update = 0;
    unsigned long now = micros();

#define TARGET_RATE 128
#define TARGET_INTERVAL (1e6 / TARGET_RATE)

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

    /* Initialize IMUs */
    main_imu = sbgc_mpu6050_new(MPU6050_DEFAULT_ADDR, i2c);
    if (!main_imu) {
        serial->println("Main MPU6050 initialization failed!");
        while (1);
    }

    serial->println("Main MPU6050 initialized!");

    main_ahrs = ahrs_new(main_imu, SBGC_IMU_X, SBGC_IMU_MINUS_Z);
    ahrs_set_weights(main_ahrs, 0.5f, 0.1f, 0.5f, M_PI / 0x800);
    ahrs_set_debug(main_ahrs, main_ahrs_debug_print);
    delay(100);
    ahrs_calibrate(main_ahrs);
    serial->println("Main AHRS initialized!");

    /* TODO: frame_ahrs */

    encoders[0] = sbgc_as5600_new(i2c);
    encoders[1] = sbgc32_i2c_drv_encoder_new(SBGC32_I2C_DRV_ADDR(1), i2c, SBGC32_I2C_DRV_ENC_TYPE_AS5600);
    encoders[2] = sbgc32_i2c_drv_encoder_new(SBGC32_I2C_DRV_ADDR(4), i2c, SBGC32_I2C_DRV_ENC_TYPE_AS5600);
    serial->println("Encoders initialized!");
}

void shutdown_to_bl(void) __attribute__((noreturn));
void shutdown_to_bl(void) {
    int i;

    /* Things we set up explicitly (TODO: steal_ptr() syntax) */
    ahrs_free(main_ahrs);
    main_imu->cls->free(main_imu);
    for (i = 0; i < 3; i++)
        if (encoders[i])
            encoders[i]->cls->free(encoders[i]);
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

void blink(void) {
    static uint8_t led_val = 0;

    led_val ^= 1;
    digitalWrite(SBGC_LED_GREEN, led_val);
}

void loop(void) {
    main_loop_sleep();
    ahrs_update(main_ahrs);

    if (frame_ahrs)
        ahrs_update(frame_ahrs);

    // imu_debug_update();
    // probe_pins_update();

    blink();

    if (serial->available()) {
        uint8_t cmd = serial->read();
        int i;
        struct calibrate_data_s cs;
        static uint8_t dlpf = 0;

        switch (cmd) {
        case 'q':
            serial->println("Shutting down and jumping to bootloader");
            shutdown_to_bl();
            break;
        case 'Q':
            serial->println("Crashing in loop()"); /* Crash handler test */
            delay(100);
            *(uint8_t *) -1 = 5;
            break;
        case '0' ... '5':
            serial->println("Setting new MPU6050 clksource");
            mpu6050_set_clksrc(main_imu, cmd - '0');
            delay(100);
            break;
        case '6' ... '9':
            serial->println("Setting new MPU6050 sample rate");
            mpu6050_set_srate(main_imu, (cmd - '6') ? 1 << 2 * (cmd - '6') : 0, dlpf);
            /// 1k / ((1 << 6) + 1)
            delay(100);
            break;
        case '*':
            dlpf ^= 2;
            serial->print("Will use MPU6050 DLPF setting ");
            serial->print(dlpf);
            serial->println(" on next sample rate change");
            break;
        case 'e':
            /* Read encoder angles */
            for (i = 0; i < 3; i++) {
                if (!encoders[i])
                    continue;

                serial->print("Encoder ");
                serial->print(i);
                serial->print(" angle = ");
                serial->println((float) encoders[i]->cls->read(encoders[i]) / encoders[i]->cls->scale);
            }

            break;
        case 'c':
            serial->println("Recalibrating main AHRS");
            delay(100);
            ahrs_calibrate(main_ahrs);
            break;
        case 'C':
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
        default:
            serial->print("Unknown cmd ");
            serial->println(cmd);
        }
    }
}
