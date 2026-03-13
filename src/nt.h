/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef NT_H
#define NT_H

#include <Arduino.h>

#include "imu.h"
#include "util.h"

typedef struct obgc_nt_bus_s {
    Stream *port;
    unsigned long trigger_ts;
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

static inline void ntbus_trigger(obgc_nt_bus_t *nt) {
    nt->port->write(NTBUS_STX | NTBUS_TRIGGER | NTBUS_ID_ALLMODULES);
}

#define READ_TIMEOUT_MS 50
static inline int ntbus_read_resp(obgc_nt_bus_t *nt, uint8_t *buf_out, uint8_t num) {
    unsigned long end_ts = millis() + READ_TIMEOUT_MS;

    do {
        while (nt->port->available()) {
            *buf_out++ = nt->port->read();
            if (!--num)
                break;
        }
    } while ((long) (end_ts - millis()) > 0);

    return -num;
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

    nt->port->write(NTBUS_STX | cmd | id);
    if (cmd == NTBUS_CMD) {
        nt->port->write(subcmd);
        nt->port->write(subcmd); /* XOR over the data bytes, ie. the ones without NTBUS_STX */
    }

    if ((missing = ntbus_read_resp(nt, resp_out, resp_len)) < 0) {
        char msg[50];
        sprintf(msg, "NT response timeout: %i", missing);
        error_print(msg); /* TODO: ratelimit */
        return -1;
    }

    if (ntbus_validate_resp(resp_out, resp_len) < 0) {
        error_print("NT response format error");
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

struct nt_imu_s {
    obgc_imu obj;
    obgc_nt_bus_t *nt;
    enum ntbus_id_e id;
    unsigned long access_ts;
    int16_t temp;
};

static void nt_imu_read_main(struct nt_imu_s *dev, int32_t *accel_out, int32_t *gyro_out) {
    uint8_t resp[16];

    ntbus_check_trigger(dev->nt, &dev->access_ts);

    /* Note: Maybe should wait for rx buffer to be empty because at this bit rate (2Mb/s) if
     * we hit any delay we'll lose some rx bytes */

    if (ntbus_req(dev->nt, dev->id, NTBUS_GET, 0, resp, 16) < 0)
        return; /* TODO: report error */

    /* GET: 3xi16 acc + 3xi16 gyro + 1xi16 temp + 1xu8 imu status */
    accel_out[0] = (int16_t)  ((uint16_t) resp[ 1] << 8) | resp[ 0];
    accel_out[1] = (int16_t)  ((uint16_t) resp[ 3] << 8) | resp[ 2];
    accel_out[2] = (int16_t)  ((uint16_t) resp[ 5] << 8) | resp[ 4];
    gyro_out[0]  = (int16_t) (((uint16_t) resp[ 7] << 8) | resp[ 6]) << 2;
    gyro_out[1]  = (int16_t) (((uint16_t) resp[ 9] << 8) | resp[ 8]) << 2;
    gyro_out[2]  = (int16_t) (((uint16_t) resp[11] << 8) | resp[10]) << 2;
    dev->temp    = (int16_t)  ((uint16_t) resp[13] << 8) | resp[12];
}

static void nt_imu_read_temp(struct nt_imu_s *dev, int32_t *temp_out) {
    *temp_out = dev->temp / 100;
}

static void nt_imu_free(struct nt_imu_s *dev) {
    free(dev);
}

static inline obgc_imu *nt_imu_new(obgc_nt_bus_t *bus, uint8_t id) {
    struct nt_imu_s *imu;
    static enum ntbus_id_e nt_ids[] = { NTBUS_ID_IMU1, NTBUS_ID_IMU2, NTBUS_ID_IMU3 };
    static obgc_imu_class nt_imu_class = {
        .read_main   = (void (*)(obgc_imu *imu, int32_t *accel_out, int32_t *gyro_out)) nt_imu_read_main,
        .read_temp   = (void (*)(obgc_imu *imu, int32_t *temp_out)) nt_imu_read_temp,
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

#endif /* NT_H */
