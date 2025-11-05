/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef STORAGE_H
#define STORAGE_H

#include <Wire.h>
#include <stdbool.h>

#include "motor.h"
#include "axes.h"
#include "control.h"

#define STORAGE_CONFIG_VERSION 1

extern struct obgc_storage_config_s {
    /* Hardware setup */
    /* TODO: add this when we support anything other than PilotFly H2.
     * Local/remote encoder types, motor types, IMU types, the deprecated IMU orientation, pinout, VBAT scale, min voltage.
     */

    /* Calibration data */
    struct obgc_motor_calib_data_s motor_calib[3];
    struct axes_data_s axes;
    bool have_axes;
    /* TODO: PIDs, including motor, ahrs Kps */

    /* User settings */
    struct control_settings_s control;
} config;

void storage_init_internal_flash(void);
void storage_init_i2c_eeprom(uint8_t addr, TwoWire *i2c, uint32_t size);

int storage_read(void);
int storage_write(void);

#endif /* STORAGE_H */
