/* vim: set ts=4 sw=4 sts=4 et : */
#include <stdint.h>

#include "storage.h"

#include "hw.h"

typedef struct obgc_hw_config_s::obgc_imu_hw_config_s obgc_imu_hw_config;
typedef struct obgc_hw_config_s::obgc_motor_hw_config_s obgc_motor_hw_config;
typedef struct obgc_hw_config_s::obgc_encoder_hw_config_s obgc_encoder_hw_config;

static void *hw_setup_error(const char *name, const char *error) {
    char msg[128];

    sprintf(msg, "%s: %s", name, error);
    error_print(msg);
    return NULL;
}

static void *hw_setup_ok(const char *name, void *dev) {
    char msg[128];

    sprintf(msg, "%s: Ok", name);
    error_print(msg);
    return dev;
}

static obgc_i2c *hw_get_i2c(const char *name, const struct obgc_hw_i2c_addr_s *config,
        obgc_i2c *i2c_main, obgc_i2c *i2c_int) {
    obgc_i2c *bus = (config->bus == 0 ? i2c_main : (config->bus == 1 ? i2c_int : NULL));

    return bus ?: (obgc_i2c *) hw_setup_error(name, "I2C bus not available");
}

static obgc_imu *hw_setup_imu(const char *name,
        const obgc_imu_hw_config *config,
        obgc_i2c *i2c_main, obgc_i2c *i2c_int, obgc_nt_bus_t *nt) {
    switch (config->type) {
    case obgc_imu_hw_config::OBGC_IMU_NONE:
        return NULL;

    case obgc_imu_hw_config::OBGC_IMU_I2C_MPU6050:
    case obgc_imu_hw_config::OBGC_IMU_I2C_MPU9250:
        {
            obgc_i2c *bus = hw_get_i2c(name, &config->params.i2c, i2c_main, i2c_int);
            struct obgc_imu_s *imu;

            if (!bus)
                return NULL;

            if (config->type == obgc_imu_hw_config::OBGC_IMU_I2C_MPU6050)
                imu = mpu6050_new(config->params.i2c.addr, bus);
            else
                imu = mpu9250_new(config->params.i2c.addr, bus);

            if (imu)
                return (obgc_imu *) hw_setup_ok(name, imu);
        }
        return (obgc_imu *) hw_setup_error(name, "initialization failed");

    case obgc_imu_hw_config::OBGC_IMU_NT:
        if (!nt)
            return (obgc_imu *) hw_setup_error(name, "NT bus not available");

        {
            struct obgc_imu_s *imu = nt_imu_new(nt, config->params.nt_id);

            if (imu)
                return (obgc_imu *) hw_setup_ok(name, imu);
        }
        return (obgc_imu *) hw_setup_error(name, "initialization failed");

    default:
        return (obgc_imu *) hw_setup_error(name, "Unknown type");
    }
}

void hw_storage_init(obgc_i2c *i2c_main, obgc_i2c *i2c_int) {
    /* TODO: maybe just scan the bus, if the EEPROM is there use it, if not then
     * use on-chip flash.
     */
#if BOARD_SIMPLEBGC32_REGULAR
    storage_init_i2c_eeprom(MC_24FC256_BASE_ADDR + 0, i2c_int, 0x8000);
#else
    /* No external memory so use the MCU's on-chip flash */
    storage_init_internal_flash();
#endif
}

#if BOARD_STORM32_STM32F1
/* Override USART3 pins in framework-arduinoststm32/variants/STM32F1xx/F103R\(C-D-E\)T/PeripheralPins_STORM32_V1_31_RC.c */
const PinMap PinMap_UART_TX[] = {
  {PA_9,  USART1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_USART1_DISABLE)},
  {PB_10, USART3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE)}, /* PIN_NT_TX */
  {NC,    NP,     0}
};

const PinMap PinMap_UART_RX[] = {
  {PA_10, USART1, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, AFIO_USART1_DISABLE)},
  {PB_11, USART3, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, AFIO_NONE)}, /* PIN_NT_RX */
  {NC,    NP,     0}
};
#endif

void hw_setup(const struct obgc_hw_config_s *config,
        obgc_i2c **i2c_main, obgc_i2c **i2c_int, obgc_nt_bus_t **nt,
        struct obgc_imu_s **main_imu, struct obgc_imu_s **frame_imu,
        struct obgc_foc_driver_s **motors,
        struct obgc_encoder_s **encoders) {
    int i;
    int nt_users = 0;

    if (config->main_imu.type == obgc_imu_hw_config::OBGC_IMU_NT)
        nt_users++;
    if (config->frame_imu.type == obgc_imu_hw_config::OBGC_IMU_NT)
        nt_users++;
    for (i = 0; i < 3; i++)
        if (config->motor[i].type == obgc_motor_hw_config::OBGC_MOTOR_DRV_NT)
            nt_users++;
    for (i = 0; i < 3; i++)
        if (config->encoder[i].type == obgc_encoder_hw_config::OBGC_ENCODER_NT_MOTOR_DRV)
            nt_users++;

    /* The NT Bus and "main" I2C use the same pins so chose one or the other */
    if (nt_users && PIN_NT_RX && PIN_NT_TX) {
        HardwareSerial *port = new HardwareSerial(PIN_NT_RX, PIN_NT_TX);
        port->begin(2000000);
        *nt = port;
    }

    if (!IN_SET(PIN_I2C_MAIN_SDA, PIN_NT_RX, PIN_NT_TX) || !*nt) {
        *i2c_main = new obgc_i2c_subcls<TwoWire>(PIN_I2C_MAIN_SDA, PIN_I2C_MAIN_SCL);
        (*i2c_main)->begin();
        (*i2c_main)->setClock(400000); /* 400kHz I2C */
    }

    if (!*i2c_int) { /* Note: On SimpleBGC32_regular we created this earlier */
        *i2c_int = new obgc_i2c_subcls<TwoWire>(PIN_I2C_INT_SDA, PIN_I2C_INT_SCL);
        (*i2c_int)->begin();
        (*i2c_int)->setClock(400000); /* 400kHz I2C */
    }

    *main_imu = hw_setup_imu("Main IMU", &config->main_imu, *i2c_main, *i2c_int, *nt);
    *frame_imu = hw_setup_imu("Frame IMU", &config->frame_imu, *i2c_main, *i2c_int, *nt);

#if 0
    motors[0] = hw_setup_motor("Motor 0", &config->motor[0], *i2c_main, *i2c_int, *nt);
    motors[1] = hw_setup_motor("Motor 1", &config->motor[1], *i2c_main, *i2c_int, *nt);
    motors[2] = hw_setup_motor("Motor 2", &config->motor[2], *i2c_main, *i2c_int, *nt);

    encoders[0] = hw_setup_encoder("Encoder 0", &config->encoder[0], motors[0], *i2c_main, *i2c_int, *nt);
    encoders[1] = hw_setup_encoder("Encoder 1", &config->encoder[1], motors[1], *i2c_main, *i2c_int, *nt);
    encoders[2] = hw_setup_encoder("Encoder 2", &config->encoder[2], motors[2], *i2c_main, *i2c_int, *nt);
#endif
}
