/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef I2C_H
#define I2C_H

#include <Arduino.h>
#include <Wire.h>
#include <FlexWire.h>

/* Common base class for TwoWire and other I2C imlpementations (software or hardware) since
 * none of the common libraries declare a usable base class.  We could maybe have the template
 * inherit from the library classes but composition is supposedly preferred anyway.  Or we
 * could bite the bullet and make these plain C functions so the callers don't need to be
 * compiled as C++ code, only this file.
 * TODO: measure the flash space overhead of this vs. other approaches.  See if we want to
 * even use hardware I2C at all since we're synchronous.
 */

class obgc_i2c {
public:
    virtual ~obgc_i2c() = default;

    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void setClock(uint32_t) = 0;
    virtual void beginTransmission(uint8_t address) = 0;
    virtual uint8_t endTransmission(bool stop_condition = true) = 0;
    virtual uint8_t requestFrom(uint8_t address, uint8_t quantity, bool stop_condition = true) = 0;
    virtual uint8_t requestFrom(uint8_t, uint8_t, uint32_t, uint8_t, bool) = 0;
    virtual size_t write(uint8_t data) = 0;
    virtual size_t write(const uint8_t *data, size_t length) = 0;
    virtual int read() = 0;
};

template<typename wire_type>
class obgc_i2c_subcls : public obgc_i2c {
private:
    wire_type *wire;
    bool owns_wire;

public:
    /* Constructor for existing Wire instance (no ownership) */
    obgc_i2c_subcls(wire_type *wire_instance)
        : wire(wire_instance), owns_wire(false) {}

    /* Constructor that creates the Wire instance with pins (takes ownership) */
    obgc_i2c_subcls(uint8_t sda_pin, uint8_t scl_pin)
        : wire(new wire_type(sda_pin, scl_pin)), owns_wire(true) {}

    /* Destructor to clean up if we own the wire instance */
    virtual ~obgc_i2c_subcls() {
        if (owns_wire) {
            delete wire;
        }
    }

    /* Delete copy constructor and assignment to prevent ownership issues */
    obgc_i2c_subcls(const obgc_i2c_subcls&) = delete;
    obgc_i2c_subcls& operator=(const obgc_i2c_subcls&) = delete;

    /* Move constructor */
    obgc_i2c_subcls(obgc_i2c_subcls&& other)
        : wire(other.wire), owns_wire(other.owns_wire) {
        other.owns_wire = false;
        other.wire = nullptr;
    }

    void begin() override { wire->begin(); }
    void end() override { /*wire->end(); disabled until FlexWire fix it */ }
    void setClock(uint32_t frequency) { wire->setClock(frequency); }
    void beginTransmission(uint8_t address) override { wire->beginTransmission(address); }
    uint8_t endTransmission(bool stop_condition = true) override {
        return wire->endTransmission(stop_condition);
    }
    uint8_t requestFrom(uint8_t address, uint8_t quantity, bool stop_condition = true) override {
        return wire->requestFrom(address, quantity, stop_condition);
    }
    uint8_t requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize,
            bool send_stop) override {
        /* See comments in ~/.platformio/packages/framework-arduinoststm32/libraries/Wire/src/Wire.cpp */
        beginTransmission(address);
        while (isize-- > 0) {
            write((uint8_t) (iaddress >> (isize * 8)));
        }
        endTransmission(false);
        return requestFrom(address, quantity, send_stop);
    }
    size_t write(uint8_t data) override { return wire->write(data); }
    size_t write(const uint8_t *data, size_t length) override { return wire->write(data, length); }
    int read() override { return wire->read(); }
};

#endif /* I2C_H */
