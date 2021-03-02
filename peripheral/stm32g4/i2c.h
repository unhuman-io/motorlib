#pragma once
#include "stm32g474xx.h"

class I2C {
 public:
    I2C(I2C_TypeDef &regs, uint16_t speed_khz = 100) : regs_(regs) {
            switch (speed_khz) {
                case 1000:
                    regs_.TIMINGR = 0x00802172; // 1 Mbps at 170 MHz clock
                    break;
                case 400:
                    regs_.TIMINGR = 0x10802d9b; // 400 kHz at 170 MHz clock
                    break;
                default:
                case 100:
                    regs_.TIMINGR = 0x30a0a7fb; // 100 kHz at 170 MHz clock
                    break;
            }       
            regs_.CR1 |= I2C_CR1_PE;
    }
    void write(uint8_t address, int8_t nbytes, uint8_t *data, bool stop = false) {
        clear_isr();
        regs_.TXDR = data[0];
        regs_.CR2 = (address << 1) | (nbytes << I2C_CR2_NBYTES_Pos);
        regs_.CR2 |= I2C_CR2_START;
        for (int i=1; i<nbytes; i++) {
            uint32_t t_start = get_clock();
            while_timeout_ms(!(regs_.ISR & (I2C_ISR_TXE | I2C_ISR_NACKF)), 10);
            regs_.TXDR = data[i];
        }
        uint32_t t_start = get_clock();
        while_timeout_ms(!(regs_.ISR & (I2C_ISR_TXE | I2C_ISR_NACKF)), 10);
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
            uint32_t t_start = get_clock();
            while_timeout_ms(!(regs_.ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)), 10);
            data[i] = regs_.RXDR;
        }
        uint32_t t_start = get_clock();
        while_timeout_ms(!(regs_.ISR & I2C_ISR_STOPF), 10);
        regs_.ICR = I2C_ICR_STOPCF;
    }
 private:
    void clear_isr() {
        regs_.ICR |= 0x3F38;
    }
    I2C_TypeDef &regs_;
};
