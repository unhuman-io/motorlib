#pragma once
#include "stm32g474xx.h"

class I2C {
 public:
    I2C(I2C_TypeDef &regs) : regs_(regs) {}
    void write(uint8_t address, int8_t nbytes, uint8_t *data, bool stop = false) {
        regs_.TXDR = data[0];
        regs_.CR2 = (address << 1) | (nbytes << I2C_CR2_NBYTES_Pos);
        regs_.CR2 |= I2C_CR2_START;
        for (int i=1; i<nbytes; i++) {
            while(!(regs_.ISR & (I2C_ISR_TXE | I2C_ISR_NACKF)));
            regs_.TXDR = data[i];
        }
        while(!(regs_.ISR & (I2C_ISR_TXE | I2C_ISR_NACKF)));
        if (stop) {
            regs_.CR2 = I2C_CR2_STOP;
        }
    }
    void read(uint8_t address, uint8_t nbytes, uint8_t *data) {
        regs_.CR2 = (address << 1) | I2C_CR2_RD_WRN | (nbytes << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND;
        regs_.CR2 |= I2C_CR2_START;
        //while(!(regs_.ISR & I2C_ISR_TC));
        //regs_.CR2 = I2C_CR2_STOP;
        for(int i=0;i<nbytes; i++) {
            while(!(regs_.ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
            data[i] = regs_.RXDR;
        }
        while(!(regs_.ISR & I2C_ISR_STOPF));
        regs_.ICR = I2C_ICR_STOPCF;
    }
 private:
    I2C_TypeDef &regs_;
};
