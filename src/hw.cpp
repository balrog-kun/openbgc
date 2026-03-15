/* vim: set ts=4 sw=4 sts=4 et : */
#include <stdint.h>

#include "storage.h"
extern "C" {
#include "motor-pwm.h"
}

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

static obgc_i2c *hw_get_i2c(const char *name, const struct obgc_hw_config_s::obgc_hw_i2c_addr_s *config,
        struct busses_s *bus) {
    obgc_i2c *i2c_bus = (config->bus == 0 ? bus->i2c_main : (config->bus == 1 ? bus->i2c_int : NULL));

    return i2c_bus ?: (obgc_i2c *) hw_setup_error(name, "I2C bus not available");
}

static obgc_imu *hw_setup_imu(const char *name,
        const obgc_imu_hw_config *config, struct busses_s *bus) {
    struct obgc_imu_s *imu;

    switch (config->type) {
    case obgc_imu_hw_config::OBGC_IMU_NONE:
        return NULL;

    case obgc_imu_hw_config::OBGC_IMU_I2C_MPU6050:
    case obgc_imu_hw_config::OBGC_IMU_I2C_MPU9250:
        {
            obgc_i2c *i2c = hw_get_i2c(name, &config->i2c, bus);

            if (!i2c)
                return NULL;

            if (config->type == obgc_imu_hw_config::OBGC_IMU_I2C_MPU6050)
                imu = mpu6050_new(config->i2c.addr, i2c);
            else
                imu = mpu9250_new(config->i2c.addr, i2c);

            if (imu)
                return (obgc_imu *) hw_setup_ok(name, imu);
        }
        return (obgc_imu *) hw_setup_error(name, "initialization failed");

    case obgc_imu_hw_config::OBGC_IMU_NT:
        if (!bus->nt)
            return (obgc_imu *) hw_setup_error(name, "NT bus not available");

        imu = nt_imu_new(bus->nt, config->nt_id);
        if (imu)
            return (obgc_imu *) hw_setup_ok(name, imu);

        return (obgc_imu *) hw_setup_error(name, "initialization failed");

    default:
        return (obgc_imu *) hw_setup_error(name, "Unknown type");
    }
}

const obgc_motor_hw_config::obgc_motor_drv_hw_pins_s hw_onboard_motor_pins[3] = {
    { PIN_M0_A, PIN_M0_B, PIN_M0_C, PIN_M0_EN },
    { PIN_M1_A, PIN_M1_B, PIN_M1_C, PIN_M1_EN },
    { PIN_M2_A, PIN_M2_B, PIN_M2_C, PIN_M2_EN },
};

static struct obgc_foc_driver_s *hw_setup_motor(const char *name, int num,
        const struct obgc_hw_config_s *config_all, struct drivers_s *drivers, struct busses_s *bus) {
    const obgc_motor_hw_config *config = &config_all->motor[num];
    struct obgc_foc_driver_s *drv;

    switch (config->type) {
    case obgc_motor_hw_config::OBGC_MOTOR_NONE:
        return NULL;

    case obgc_motor_hw_config::OBGC_MOTOR_DRV_ONBOARD0:
    case obgc_motor_hw_config::OBGC_MOTOR_DRV_ONBOARD1:
    case obgc_motor_hw_config::OBGC_MOTOR_DRV_ONBOARD2:
    case obgc_motor_hw_config::OBGC_MOTOR_DRV_3IN_1EN:
        {
            const obgc_motor_hw_config::obgc_motor_drv_hw_pins_s *pins;

            if (config->type == obgc_motor_hw_config::OBGC_MOTOR_DRV_3IN_1EN)
                pins = &config->pins;
            else
                pins = &hw_onboard_motor_pins[config->type - obgc_motor_hw_config::OBGC_MOTOR_DRV_ONBOARD0];

            drv = motor_drv_3pwm_new(pins->in[0], pins->in[1], pins->in[2], pins->en);
            if (!drv)
                return (struct obgc_foc_driver_s *) hw_setup_error(name, "initialization failed");

            return (struct obgc_foc_driver_s *) hw_setup_ok(name, drv);
        }

    case obgc_motor_hw_config::OBGC_MOTOR_DRV_SBGC32_I2C:
        {
            const obgc_encoder_hw_config *config_enc = &config_all->encoder[num];
            enum sbgc32_i2c_drv_encoder_type enc_type = {};
            static const obgc_hw_config_s::obgc_hw_i2c_addr_s addr = { 0, 0 };
            obgc_i2c *i2c = hw_get_i2c(name, &addr, bus);

            if (!i2c)
                return NULL;

            if (config->sbgc32_id < obgc_motor_hw_config::OBGC_SBGC32_I2C_DRV_ID_1 ||
                    config->sbgc32_id > obgc_motor_hw_config::OBGC_SBGC32_I2C_DRV_ID_4)
                return (struct obgc_foc_driver_s *) hw_setup_error(name, "bad module ID");

            if (config_enc->type == obgc_encoder_hw_config::OBGC_ENCODER_SBGC32_I2C_DRV)
                enc_type = config_enc->sbgc32_type;

            drivers->drv_module[num] = sbgc32_i2c_drv_new(
                    SBGC32_I2C_DRV_ADDR(1 + config->sbgc32_id), i2c, enc_type);
        }
        if (drivers->drv_module[num])
            error_print("SBGC32_I2C_Drv initialized");
        else
            return (struct obgc_foc_driver_s *) hw_setup_error(name, "SBGC32_I2C_Drv init failed");

        return (struct obgc_foc_driver_s *) hw_setup_ok(name,
                sbgc32_i2c_drv_get_motor_drv(drivers->drv_module[num]));

    case obgc_motor_hw_config::OBGC_MOTOR_DRV_NT:
        if (!bus->nt)
            return (struct obgc_foc_driver_s *) hw_setup_error(name, "NT bus not available");

        drv = nt_motor_drv_new(bus->nt, config->nt_id);
        if (drv)
            return (struct obgc_foc_driver_s *) hw_setup_ok(name, drv);

        return (struct obgc_foc_driver_s *) hw_setup_error(name, "initialization failed");

    default:
        return (struct obgc_foc_driver_s *) hw_setup_error(name, "Unknown type");
    }
}

static obgc_encoder *hw_setup_encoder(const char *name, int num,
        const struct obgc_hw_config_s *config_all, struct drivers_s *drivers, struct busses_s *bus) {
    const obgc_encoder_hw_config *config = &config_all->encoder[num];
    const obgc_motor_hw_config *config_motor = &config_all->motor[num];
    struct obgc_encoder_s *enc;

    switch (config->type) {
    case obgc_encoder_hw_config::OBGC_ENCODER_NONE:
        return NULL;

    case obgc_encoder_hw_config::OBGC_ENCODER_I2C_AS5600:
        {
            obgc_i2c *i2c = hw_get_i2c(name, &config->i2c, bus);

            if (!i2c)
                return NULL;

            enc = as5600_new(i2c);
        }
        if (enc)
            return (obgc_encoder *) hw_setup_ok(name, enc);
        else
            return (obgc_encoder *) hw_setup_error(name, "initialization failed");

    case obgc_encoder_hw_config::OBGC_ENCODER_SBGC32_I2C_DRV:
        /* Is there a use case for having a different motor driver with this encoder type? */
        /* Check if corresponding motor is the right type, in the future also include CAN here? */
        if (config_motor->type != obgc_motor_hw_config::OBGC_MOTOR_DRV_SBGC32_I2C)
            return (obgc_encoder *) hw_setup_error(name, "Motor driver type doesn't match");

        if (!drivers->drv_module[num])
            return NULL; /* We must have already printed an error */

        return (obgc_encoder *) hw_setup_ok(name,
                sbgc32_i2c_drv_get_encoder(drivers->drv_module[num]));

    case obgc_encoder_hw_config::OBGC_ENCODER_NT_MOTOR_DRV:
    default:
        return (obgc_encoder *) hw_setup_error(name, "Unknown type");
    }
}

void hw_storage_init(struct busses_s *bus) {
    /* TODO: maybe just scan the bus, if the EEPROM is there use it, if not then
     * use on-chip flash.
     */
#if BOARD_SIMPLEBGC32_REGULAR
    storage_init_i2c_eeprom(MC_24FC256_BASE_ADDR + 0, bus->i2c_int, 0x8000);
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

void hw_setup(const struct obgc_hw_config_s *config, struct busses_s *bus,
        struct obgc_imu_s **main_imu, struct obgc_imu_s **frame_imu,
        struct drivers_s *drivers) {
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
        bus->nt = (obgc_nt_bus_t *) malloc(sizeof(obgc_nt_bus_t));
        memset(bus->nt, 0, sizeof(obgc_nt_bus_t));
        bus->nt->port = port;
    }

    if (!IN_SET(PIN_I2C_MAIN_SDA, PIN_NT_RX, PIN_NT_TX) || !bus->nt) {
        bus->i2c_main = new obgc_i2c_subcls<TwoWire>(PIN_I2C_MAIN_SDA, PIN_I2C_MAIN_SCL);
        bus->i2c_main->begin();
        bus->i2c_main->setClock(400000); /* 400kHz I2C */
    }

    if (!bus->i2c_int) { /* Note: On SimpleBGC32_regular we created this earlier */
        bus->i2c_int = new obgc_i2c_subcls<TwoWire>(PIN_I2C_INT_SDA, PIN_I2C_INT_SCL);
        bus->i2c_int->begin();
        bus->i2c_int->setClock(400000); /* 400kHz I2C */
    }

    *main_imu = hw_setup_imu("Main IMU", &config->main_imu, bus);
    *frame_imu = hw_setup_imu("Frame IMU", &config->frame_imu, bus);

    hw_motor_low_level_poweroff(config);

    drivers->motor[0] = hw_setup_motor("Motor 0", 0, config, drivers, bus);
    drivers->motor[1] = hw_setup_motor("Motor 1", 1, config, drivers, bus);
    drivers->motor[2] = hw_setup_motor("Motor 2", 2, config, drivers, bus);

    drivers->encoder[0] = hw_setup_encoder("Encoder 0", 0, config, drivers, bus);
    drivers->encoder[1] = hw_setup_encoder("Encoder 1", 1, config, drivers, bus);
    drivers->encoder[2] = hw_setup_encoder("Encoder 2", 2, config, drivers, bus);
}
