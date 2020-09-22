#pragma once
#include "i2c.h"

class MAX31875 {
 public:
    MAX31875(I2C &i2c) : i2c_(i2c) {

    }
    float read() {
        if (first_read_) {
            first_read_ = false;
            uint8_t conf[] = {1, 0, 0xE6};
            i2c_.write(0x48, 3, conf, true);
        }

        uint8_t reg = 0;
        i2c_.write(0x48, 1, &reg);
        uint8_t raw_val[2];
        i2c_.read(0x48, 2, raw_val);
        value_ = ((raw_val[0] << 8 | raw_val[1])>>3) * (100.0/16);
        return value_;
    }
 //private:
    I2C &i2c_;
    uint32_t value_;
    bool first_read_ = true;

    template<typename, typename>
    friend class System;
};