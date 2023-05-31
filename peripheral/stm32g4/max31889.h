#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_MAX31889_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_MAX31889_H_

#include "i2c_dma.h"

class MAX31889 {
 public:
    MAX31889(I2C_DMA &i2c, uint8_t address = 0x50) : i2c_(i2c), address_(address) {
        // note 400 kHz i2c max
        if (address_ < 0x50) {
            // address can either be given as 0 through 3 or 
            // as 0x50 + 0 through 3
            address_ += 0x50;
        }
    }
    float read() {
        uint8_t fifo_reg = 8;
        int ret_val = i2c_.write(address_, 1, &fifo_reg);
        if (ret_val <= 0) {
            value_ = 0;
            //logger.log_printf("%x i2c write error: %d", address_, ret_val);
            return 0;
        }
        uint8_t raw_val[2] = {};
        ret_val = i2c_.read(address_, 2, raw_val);
        if (ret_val <= 0) {
            value_ = 0;
            //logger.log_printf("%x i2c read error: %d", address_, ret_val);
            return 0;
        }
        //logger.log_printf("max31889 %x %x %x",address_, raw_val[0], raw_val[1]);
        // logger.log_printf("%p %x",raw_val, I2C1->RXDR);
        
        raw_value_ = (raw_val[0] << 8 | raw_val[1]);
        value_ = raw_value_ * .005;
        
        // trigger new read
        uint8_t reg_val[2] = {0x14, 1};
        ret_val = i2c_.write(address_, 2, reg_val, true);
        if (ret_val <= 0) {
            //logger.log_printf("%x i2c write trigger error: %d", address_, ret_val);
        }

        return get_temperature();
    }
    float get_temperature() const {
        return value_;
    }

    I2C_DMA &i2c_;
    uint16_t raw_value_;
    float value_;
    uint8_t address_;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_MAX31889_H_
