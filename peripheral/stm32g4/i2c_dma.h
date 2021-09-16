#pragma once
#include "stm32g474xx.h"
#include "../../util.h"

// safe to call only at one time, i.e. don't call from interrupt and main simultaneously
class I2C_DMA {
 public:
    I2C_DMA(I2C_TypeDef &regs, DMA_Channel_TypeDef &tx_dma, DMA_Channel_TypeDef &rx_dma, uint16_t speed_khz = 100) : 
        regs_(regs), tx_dma_(tx_dma), rx_dma_(rx_dma) {
            tx_dma_.CPAR = (uint32_t) &regs_.TXDR;
            rx_dma_.CPAR = (uint32_t) &regs_.RXDR;
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
            regs_.CR1 |= I2C_CR1_RXDMAEN | I2C_CR1_TXDMAEN;

    }

    // return 1 for error, 0 for success
    int async_write(uint8_t address, int8_t nbytes, uint8_t *data, bool stop = false) {
        if (!ready()) {
            return 1;
        }
        clear_isr();
        tx_dma_.CCR = 0;
        tx_dma_.CMAR = (uint32_t) data;
        tx_dma_.CNDTR = nbytes;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
        regs_.CR2 = (address << 1) | (nbytes << I2C_CR2_NBYTES_Pos) | (stop ? I2C_CR2_AUTOEND : 0) | I2C_CR2_START;
        return 0;
    }
    // return 1 for error, 0 for success
    int async_read(uint8_t address, uint8_t nbytes, uint8_t *data) {
        if (!ready()) {
            return 1;
        }
        clear_isr();
        rx_dma_.CCR = 0;
        rx_dma_.CMAR = (uint32_t) data;
        rx_dma_.CNDTR = nbytes;
        rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
        regs_.CR2 = (address << 1) | I2C_CR2_RD_WRN | (nbytes << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
        return 0;
    }

    // return -1 or 0 for error, nbytes for success
    int write(uint8_t address, int8_t nbytes, uint8_t *data, bool stop = false, uint32_t timeout_us = 0) {
        bool retval = wait_while_true_with_timeout_us(async_write(address, nbytes, data, stop), timeout_us);
        if(retval) {
            return 0;
        }
        retval = wait_while_false_with_timeout_us(ready(), timeout_us); // todo, double timeout
        if(retval) {
            return -1;
        } else {
            return nbytes;
        }
    }

    // return -1 or 0 for error, nbytes for success
    int read(uint8_t address, int8_t nbytes, uint8_t *data, uint32_t timeout_us = 0) {
        bool retval = wait_while_true_with_timeout_us(async_read(address, nbytes, data), timeout_us);
        if(retval) {
            return 0;
        }
        retval = wait_while_false_with_timeout_us(ready(), timeout_us); // todo, double timeout
        if(retval) {
            return -1;
        } else {
            return nbytes;
        }
    }

    bool ready() const {
        return ((rx_dma_.CNDTR == 0) && (tx_dma_.CNDTR == 0)) || (regs_.ISR & (I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_ARLO | I2C_ISR_BERR));
    }
 private:
    void clear_isr() {
        regs_.ICR |= 0x3F38;
    }
    I2C_TypeDef &regs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
};
