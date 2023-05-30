#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_MAX31875_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_MAX31875_H_

#include "i2c_dma.h"

class MAX31875 {
 public:
    MAX31875(I2C_DMA &i2c, uint8_t address = 0x48) : i2c_(i2c), address_(address) {
        if (address_ < 0x48) {
            // address can either be given as 0 through 7 or 
            // as 0x48 + 0 through 7
            address_ += 0x48;
        }
    }
    float read() {
        if (first_read_) {
            first_read_ = false;
            uint8_t conf[] = {1, 0, 0xE6};
            i2c_.write(address_, 3, conf, true);
        }

        uint8_t reg = 0;
        int ret_val = i2c_.write(address_, 1, &reg);
        if (ret_val <= 0) {
            value_ = 0;
            logger.log_printf("i2c write error: %d", ret_val);
            return 0;
        }
        uint8_t raw_val[2] = {};
        ret_val = i2c_.read(address_, 2, raw_val);
        if (ret_val <= 0) {
            value_ = 0;
            logger.log_printf("i2c read error: %d", ret_val);
            return 0;
        }
        value_ = ((raw_val[0] << 8 | raw_val[1])>>3) * (100.0/16);
        return get_temperature();
    }
    float get_temperature() const {
        return value_*0.01;
    }
 //private:
    I2C_DMA &i2c_;
    uint32_t value_;
    bool first_read_ = true;
    uint8_t address_;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_MAX31875_H_
