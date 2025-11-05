/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef AS5600_H
#define AS5600_H

#include "encoder.h"
#include "i2c.h"

/* Only one I2C address is defined in the spec so no need for a parameter */

obgc_encoder *as5600_new(obgc_i2c *i2c);

const char *as5600_get_error(obgc_encoder *enc);

#endif /* AS5600_H */
