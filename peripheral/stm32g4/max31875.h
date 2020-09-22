#pragma once
#include "i2c.h"

class MAX31875 {
 public:
    MAX31875(I2C &i2c) : i2c_(i2c) {}
    float read() {
        return (int8_t) i2c_.read(0);
    }
 private:
    I2C &i2c_;
};