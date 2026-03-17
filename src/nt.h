/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef NT_H
#define NT_H

#include <Arduino.h>

#include "imu.h"
#include "motor.h"
#include "util.h"

typedef struct obgc_nt_bus_s {
    Stream *port;
    unsigned long trigger_ts;
    uint16_t error_cnt;
    uint8_t motor_set_cmd[12];
} obgc_nt_bus_t;

/* Values from https://www.olliw.eu/storm32bgc-v2-wiki/NT_Bus_Protocol */

enum ntbus_id_e {
    NTBUS_ID_ALLMODULES = 0,
    NTBUS_ID_IMU1       = 1,
    NTBUS_ID_IMU2       = 2,
    NTBUS_ID_MOTORALL   = 3,
    NTBUS_ID_MOTORPITCH = 4,
    NTBUS_ID_MOTORROLL  = 5,
    NTBUS_ID_MOTORYAW   = 6,
    NTBUS_ID_CAMERA     = 7,
    NTBUS_ID_LOGGER     = 11,
    NTBUS_ID_IMU3       = 12,
};

#define NTBUS_STX                         0x80
#define NTBUS_SFMASK                      0x70
#define NTBUS_IDMASK                      0x0f

#define NTBUS_FLASH                       0x70
#define NTBUS_RESET                       0x50
#define NTBUS_SET                         0x40
#define NTBUS_GET                         0x30
#define NTBUS_TRIGGER                     0x10
#define NTBUS_CMD                         0x00

enum ntbus_cmd_e {
    NTBUS_CMD_GETSTATUS             = 1,
    NTBUS_CMD_GETVERSIONSTR         = 2, /* must be supported by any NT module */
    NTBUS_CMD_GETBOARDSTR           = 3, /* must be supported by any NT module */
    NTBUS_CMD_GETCONFIGURATION      = 4, /* must be supported by any NT module */

    /* Module-specific commands */
    NTBUS_CMD_PIDDATA               = 35,
    NTBUS_CMD_PARAMETERDATA         = 36,
    NTBUS_CMD_AHRS1DATA             = 37,
    NTBUS_CMD_ACCGYRO1RAWDATA_V2    = 40,
    NTBUS_CMD_ACCGYRO2RAWDATA_V2    = 41,
    NTBUS_CMD_ACCGYRO1DATA_V2       = 43,
    NTBUS_CMD_ACCGYRO2DATA_V2       = 44,
    NTBUS_CMD_ENCODERDATA           = 45,
    NTBUS_CMD_STORM32LINKDATA       = 46,
    NTBUS_CMD_TUNNELTX              = 47,
    NTBUS_CMD_TUNNELRXGET           = 48,
    NTBUS_CMD_AUTOPILOTSYSTEMTIME   = 49,
    NTBUS_CMD_PIDINDATA             = 50,
    NTBUS_CMD_FUNCTIONINPUTVALUES   = 51,
    NTBUS_CMD_PIDIDDATA             = 52,

    NTBUS_CMD_READLOGGERDATETIME    = 114,
    NTBUS_CMD_WRITELOGGERDATETIME   = 115,

    NTBUS_CMD_STOREMOTORCALIBRATION = 116, /* Motor encoder poles & ofs data */
    NTBUS_CMD_READMOTORCALIBRATION  = 117,
    NTBUS_CMD_STOREIMUCALIBRATION   = 118, /* gyro & acc calibration data */
    NTBUS_CMD_READIMUCALIBRATION    = 119,

    NTBUS_CMD_DEBUGDATA             = 127,
};

#define READ_TIMEOUT_MS 2

#include <LL/stm32yyxx_ll_usart.h>

class HardwareNT : public HardwareSerial {
    uint8_t nt_prio;
    uint8_t orig_prio;

public:
    HardwareNT(uint32_t _rx, uint32_t _tx, uint8_t prio) : HardwareSerial(_rx, _tx) {
        nt_prio = prio << (8 - __NVIC_PRIO_BITS);
    }

    void begin(void) {
        HardwareSerial::begin(2000000);

        /* With unmodified HardwareSerial we're losing a byte or two every
         * second on the 72MHz STM32F1 probably due to the complicated
         * interrupt-driven logic with 4 or 5 layers of indirection (at byte
         * level.)  So don't use interrupts.
         *
         * Undo what begin() --> HAL_UART_Receive_IT() just did to reset
         * RxState to READY so we can use HAL_UART_Receive().
         */
        NVIC_DisableIRQ(_serial.irq);
        HAL_UART_AbortReceive_IT(&_serial.handle);
    }

    void enter_req_resp_section(void) {
        /* Briefly disable interrupts because even without us using interrupts,
         * traffic on the other serial ports and/or USB port and possibly RC
         * inputs (unconfirmed) makes us miss a byte sometimes.
         */
        orig_prio = __get_BASEPRI();
        __set_BASEPRI(nt_prio);

        if (LL_USART_IsActiveFlag_RXNE(_serial.handle.Instance))
            (void) LL_USART_ReceiveData8(_serial.handle.Instance); /* Flush stale data if any */
        LL_USART_ClearFlag_ORE(_serial.handle.Instance); /* Clear overrun if any */
    }

    int read_resp(uint8_t *buf, uint8_t len) {
        /* Skip two abstraction layers to use the
         * framework-arduinoststm32/system/Drivers/STM32Fxxx_HAL_Driver/Src/stm32fxxx_hal_uart.c
         * API not exposed by higher layers.
         *
         * Maybe we'd be even better off using LL_USART_* macros all the way
         * for even less overhead.
         */
        int ret = HAL_UART_Receive(&_serial.handle, buf, len, READ_TIMEOUT_MS) == HAL_OK ? 0 : -1;
        __set_BASEPRI(orig_prio); /* Restore interrupts */
        return ret;
    }

    size_t write(uint8_t c) {
        /* Use sync writes for simplicity, we have all the time until the response arrives
         * available to us so no hurry to return.
         * Note ntbus_send_motor_set() uses an async write though.
         */
        while (!LL_USART_IsActiveFlag_TXE(_serial.handle.Instance)); /* Wait until TX buffer empty */
        LL_USART_TransmitData8(_serial.handle.Instance, c);
        return 0;
    }
};

static inline void ntbus_trigger(obgc_nt_bus_t *nt) {
    nt->port->write(NTBUS_STX | NTBUS_TRIGGER | NTBUS_ID_ALLMODULES);
}

static inline int ntbus_read_resp(obgc_nt_bus_t *nt, uint8_t *buf_out, uint8_t num) {
    return ((HardwareNT *) nt->port)->read_resp(buf_out, num);
}

static inline int ntbus_validate_resp(uint8_t *resp, uint8_t num) {
    uint8_t crc = 0;

    for (; num; resp++, num--)
        crc ^= *resp;

    return crc ? -1 : 0;
}

static inline int ntbus_req(obgc_nt_bus_t *nt, uint8_t id, uint8_t cmd,
        uint8_t subcmd, uint8_t *resp_out, uint8_t resp_len) {
    int missing;

    /* Enter the critical section immediately after the last byte of the
     * request because we need to be sure it starts before the remote module
     * starts sending the response.  But we also want to block interrupts for
     * the minimum necessary period only.  Actually enter it before the last
     * (CRC) byte because apparently the modules don't care about the CRC
     * byte.  We still do send it.
     */

    nt->port->write(NTBUS_STX | cmd | id);
    if (cmd == NTBUS_CMD) {
        nt->port->write(subcmd);
        ((HardwareNT *) nt->port)->enter_req_resp_section();
        nt->port->write(subcmd); /* XOR over the data bytes, ie. the ones without NTBUS_STX */
    } else
        ((HardwareNT *) nt->port)->enter_req_resp_section();

    if ((missing = ntbus_read_resp(nt, resp_out, resp_len)) < 0) {
        char msg[50];
        sprintf(msg, "NT response timeout: %i", missing);
        error_print(msg); /* TODO: ratelimit */
        nt->error_cnt++;
        return -1;
    }

    if (ntbus_validate_resp(resp_out, resp_len) < 0) {
        error_print("NT response format error");
        nt->error_cnt++;
        return -1;
    }

    return 0;
}

static inline void ntbus_check_trigger(obgc_nt_bus_t *nt, unsigned long *access_ts) {
    /* FIXME: the trigger semantics are not exactly documented.
     * Ensure that each new sensor measurement has a separate trigger instance
     * and that the trigger wasn't more than 50ms ago, assume this is enough.
     */
    unsigned long now = micros();

    if ((unsigned long) (now - nt->trigger_ts) > 50000 ||
            (signed long) (*access_ts - nt->trigger_ts) >= 0) {
        ntbus_trigger(nt);
        nt->trigger_ts = now;
    }

    *access_ts = now;
}

static inline void ntbus_send_motor_set(obgc_nt_bus_t *nt, enum ntbus_id_e target_id) {
    uint8_t *buf = nt->motor_set_cmd;
    int i;

    /* Send to just this motor but include all motors' data just in case */
    buf[0] = NTBUS_STX | NTBUS_SET | target_id;
    buf[11] = buf[1];
    for (i = 2; i < 11; i++)
        buf[11] ^= buf[i];

    /* Async write is good here because we're not waiting for a response */
    nt->port->write(buf, 12);
}

struct nt_imu_s {
    obgc_imu obj;
    obgc_nt_bus_t *nt;
    enum ntbus_id_e id;
    unsigned long access_ts;
    int16_t temp;
};

static int nt_imu_read_main(struct nt_imu_s *dev, int32_t *accel_out, int32_t *gyro_out) {
    uint8_t resp[16];

    ntbus_check_trigger(dev->nt, &dev->access_ts);

    /* Note: Maybe should wait for rx buffer to be empty because at this bit rate (2Mb/s) if
     * we hit any delay we'll lose some rx bytes */

    if (ntbus_req(dev->nt, dev->id, NTBUS_GET, 0, resp, 16) < 0)
        return -1;

    /* GET: 3xi16 acc + 3xi16 gyro + 1xi16 temp + 1xu8 imu status */
    accel_out[0] = (int16_t)  ((uint16_t) resp[ 1] << 8) | resp[ 0];
    accel_out[1] = (int16_t)  ((uint16_t) resp[ 3] << 8) | resp[ 2];
    accel_out[2] = (int16_t)  ((uint16_t) resp[ 5] << 8) | resp[ 4];
    gyro_out[0]  = (int16_t) (((uint16_t) resp[ 7] << 8) | resp[ 6]) << 2;
    gyro_out[1]  = (int16_t) (((uint16_t) resp[ 9] << 8) | resp[ 8]) << 2;
    gyro_out[2]  = (int16_t) (((uint16_t) resp[11] << 8) | resp[10]) << 2;
    dev->temp    = (int16_t)  ((uint16_t) resp[13] << 8) | resp[12];
    return 0;
}

static int nt_imu_read_temp(struct nt_imu_s *dev, int32_t *temp_out) {
    *temp_out = dev->temp / 100;
    return 0;
}

static void nt_imu_free(struct nt_imu_s *dev) {
    free(dev);
}

static inline obgc_imu *nt_imu_new(obgc_nt_bus_t *bus, uint8_t id /*enum obgc_nt_imu_id_e*/) {
    struct nt_imu_s *imu;
    static enum ntbus_id_e nt_ids[] = { NTBUS_ID_IMU1, NTBUS_ID_IMU2, NTBUS_ID_IMU3 };
    static obgc_imu_class nt_imu_class = {
        .read_main   = (int (*)(obgc_imu *imu, int32_t *accel_out, int32_t *gyro_out)) nt_imu_read_main,
        .read_temp   = (int (*)(obgc_imu *imu, int32_t *temp_out)) nt_imu_read_temp,
        .free        = (void (*)(obgc_imu *imu)) nt_imu_free,
        .accel_scale = 8192, /* LSBs per 1g */
        .gyro_scale  = 131,  /* LSBs per 1deg/s (65536 / 1000 * 4 = 131.072)  */
    };
    static const char *model[] =
        { "unknown", "MPU6050", "MPU6000", "MPU6500/ICM20602", "MPU9250", "ICM42605", "ICM42688V/P" };
    uint8_t version[17], config[3];
    char msg[50];

    if (id >= ARRAY_SIZE(nt_ids))
        return NULL;

    if (ntbus_req(bus, nt_ids[id], NTBUS_CMD, NTBUS_CMD_GETVERSIONSTR, version, 17) < 0 ||
            ntbus_req(bus, nt_ids[id], NTBUS_CMD, NTBUS_CMD_GETCONFIGURATION, config, 3) < 0)
        return NULL;

    version[17] = '\0';
    config[0] &= 7;
    if (config[0] > ARRAY_SIZE(model))
        config[0] = 0;
    sprintf(msg, "NT IMU fw %s, model %s", (const char *) version, model[config[0]]);
    error_print(msg);

    imu = (struct nt_imu_s *) malloc(sizeof(*imu));
    memset(imu, 0, sizeof(*imu));
    imu->obj.cls = &nt_imu_class;
    imu->nt = bus;
    imu->id = nt_ids[id];
    return &imu->obj;
}

struct nt_motor_drv_s {
    obgc_foc_driver motor_drv_obj;
    obgc_nt_bus_t *nt;
    enum ntbus_id_e id;
    unsigned long access_ts;
    bool has_encoder;
};

static void nt_motor_drv_set_phase_voltage(obgc_foc_driver *motor_drv, float v_q, float v_d, float theta) {
    struct nt_motor_drv_s *dev = container_of(motor_drv, struct nt_motor_drv_s, motor_drv_obj);

    ntbus_check_trigger(dev->nt, &dev->access_ts);

    uint8_t power;
    uint16_t angle;

    /* Only handle v_q or v_d but not both at once, for now, since there are no users and it will involve complex trig */
    if (v_d != 0.0f) {
        if (v_d < 0.0f)
            theta += 180.0f;
    } else {
        v_d = v_q;
        theta += v_q > 0.0f ? 90.0f : -90.0f;
    }

    power = fabsf(v_d) * 0x7f;
    angle = (uint16_t) (uint32_t) lroundf(theta * (0x4000 / 360.0f));

    uint8_t idx = dev->id - NTBUS_ID_MOTORPITCH;
    uint8_t *buf = dev->nt->motor_set_cmd;

    buf[2 + idx * 3 + 0] = power;
    buf[2 + idx * 3 + 1] = (angle >> 0) & 0x7f;
    buf[2 + idx * 3 + 2] = (angle >> 7) & 0x7f;

    ntbus_send_motor_set(dev->nt, dev->id);
}

static int nt_motor_drv_on(obgc_foc_driver *motor_drv) {
    struct nt_motor_drv_s *dev = container_of(motor_drv, struct nt_motor_drv_s, motor_drv_obj);

    uint8_t idx = dev->id - NTBUS_ID_MOTORPITCH;
    uint8_t *buf = dev->nt->motor_set_cmd;

    /* TODO: Not clear if the global flag is the OR or AND of the other flags, assume OR */
    buf[1] |= (1 << idx) | 0x10;

    /* Don't send, wait for the new phasor command */
    return 0;
}

static void nt_motor_drv_off(obgc_foc_driver *motor_drv) {
    struct nt_motor_drv_s *dev = container_of(motor_drv, struct nt_motor_drv_s, motor_drv_obj);

    uint8_t idx = dev->id - NTBUS_ID_MOTORPITCH;
    uint8_t *buf = dev->nt->motor_set_cmd;

    buf[1] &= ~(1 << idx);
    if (!(buf[1] & 7))
        buf[1] &= ~0x10;

    ntbus_send_motor_set(dev->nt, dev->id);
}

static void nt_motor_drv_free(obgc_foc_driver *motor_drv) {
    nt_motor_drv_off(motor_drv);
    free(motor_drv);
}

static inline obgc_foc_driver *nt_motor_drv_new(obgc_nt_bus_t *bus, uint8_t id /*enum obgc_nt_motor_id_e*/) {
    struct nt_motor_drv_s *drv;
    enum ntbus_id_e nt_id = (enum ntbus_id_e) (NTBUS_ID_MOTORPITCH + id);
    static obgc_foc_driver_class nt_motor_drv_class = {
        .set_phase_voltage = nt_motor_drv_set_phase_voltage,
        .on                = nt_motor_drv_on,
        .off               = nt_motor_drv_off,
        .free              = nt_motor_drv_free,
        /* TODO: .beep */
    };
    static const char *model[] =
        { "unknown", "TLE5012B", "AS5048A", "pot", "MA7325", "hall" };
    uint8_t version[17], config[3], status[2];
    char msg[50];

    if (ntbus_req(bus, nt_id, NTBUS_CMD, NTBUS_CMD_GETVERSIONSTR, version, 17) < 0 ||
            ntbus_req(bus, nt_id, NTBUS_CMD, NTBUS_CMD_GETCONFIGURATION, config, 3) < 0 ||
            ntbus_req(bus, nt_id, NTBUS_CMD, NTBUS_CMD_GETSTATUS, status, 2) < 0)
        return NULL;

    version[17] = '\0';
    config[0] &= 7;
    if (config[0] > ARRAY_SIZE(model))
        config[0] = 0;
    sprintf(msg, "NT Motor Module fw %s, encoder type %s", (const char *) version, model[config[0]]);
    error_print(msg);

    drv = (struct nt_motor_drv_s *) malloc(sizeof(*drv));
    memset(drv, 0, sizeof(*drv));
    drv->motor_drv_obj.cls = &nt_motor_drv_class;
    drv->nt = bus;
    drv->id = nt_id;
    drv->has_encoder = status[0] >> 7;
    return &drv->motor_drv_obj;
}

static inline void ntbus_scan(obgc_nt_bus_t *nt) {
    char msg[100];

    error_print("Scanning NT bus...");
    uint8_t found = 0;
    for (uint8_t id = 0; id < 16; id++) {
        uint8_t resp[17];
        int r;

        if (IN_SET(id, NTBUS_ID_ALLMODULES, NTBUS_ID_MOTORALL))
            continue;

        nt->port->write(NTBUS_STX | NTBUS_CMD | id);
        nt->port->write(NTBUS_CMD_GETBOARDSTR);
        ((HardwareNT *) nt->port)->enter_req_resp_section();
        nt->port->write(NTBUS_CMD_GETBOARDSTR); /* XOR over the data bytes, ie. the ones without NTBUS_STX */

        if (ntbus_read_resp(nt, resp, 17) < 0)
            continue;

        found++;
        r = ntbus_validate_resp(resp, 17);
        resp[16] = '\0';

        sprintf(msg, "Device found at %i: %s", id, r == 0 ? (const char *) resp : "NT response format error");
        error_print(msg);
    }

    if (found == 0) {
        error_print("No NT devices found!");
    } else {
        sprintf(msg, "%i device(s) found", found);
        error_print(msg);
    }
}

#endif /* NT_H */
