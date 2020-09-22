#pragma once
#include "stm32g474xx.h"

class I2C {
 public:
    I2C(I2C_TypeDef &regs) : regs_(regs) {}
    uint8_t read(uint8_t address) {
        regs_.CR2 = address | I2C_CR2_RD_WRN | I2C_CR2_START | (1 << I2C_CR2_NBYTES_Pos);
        while(!(regs_.ISR & I2C_ISR_TC));
        regs_.CR2 = I2C_CR2_STOP;
        return regs_.RXDR;
    }
 private:
    I2C_TypeDef &regs_;
};
