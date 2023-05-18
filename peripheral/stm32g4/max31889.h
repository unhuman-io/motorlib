#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_MAX31889_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_MAX31889_H_

#include "i2c.h"

class MAX31889 {
 public:
    MAX31889(I2C &i2c, uint8_t address = 0x50) : i2c_(i2c), address_(address) {
        // note 400 kHz i2c max
        if (address_ < 0x50) {
            // address can either be given as 0 through 3 or 
            // as 0x50 + 0 through 3
            address_ += 0x50;
        }
    }
    float read() {
        uint8_t fifo_reg = 8;
        if (!i2c_.write(address_, 1, &fifo_reg)) {
            return 0;
        }
        uint8_t raw_val[2] = {};
        i2c_.read(address_, 2, raw_val);
        raw_value_ = (raw_val[0] << 8 | raw_val[1]);
        value_ = raw_value_ * .005;
        
        // trigger new read
        uint8_t reg_val[2] = {0x14, 1};
        i2c_.write(address_, 2, reg_val, true);

        return get_temperature();
    }
    float get_temperature() const {
        return value_;
    }

    I2C &i2c_;
    uint16_t raw_value_;
    float value_;
    uint8_t address_;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_MAX31889_H_
