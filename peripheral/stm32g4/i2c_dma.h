#pragma once
#include "stm32g474xx.h"

class I2C_DMA {
 public:
    I2C_DMA(I2C_TypeDef &regs, DMA_Channel_TypeDef &tx_dma, DMA_Channel_TypeDef &rx_dma) : 
        regs_(regs), tx_dma_(tx_dma), rx_dma_(rx_dma) {
            tx_dma_.CPAR = (uint32_t) &regs_.TXDR;
            rx_dma_.CPAR = (uint32_t) &regs_.RXDR;
            regs_.CR1 |= I2C_CR1_PE;
            regs_.CR1 |= I2C_CR1_RXDMAEN | I2C_CR1_TXDMAEN;
    }

    void write(uint8_t address, int8_t nbytes, uint8_t *data, bool stop = false) {
        clear_isr();
        while(!ready());
        tx_dma_.CMAR = (uint32_t) data;
        tx_dma_.CNDTR = nbytes;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
        regs_.CR2 = (address << 1) | (nbytes << I2C_CR2_NBYTES_Pos) | (stop ? I2C_CR2_AUTOEND : 0);
        regs_.CR2 |= I2C_CR2_START;
    }
    void read(uint8_t address, uint8_t nbytes, uint8_t *data) {
        while(!ready())
        regs_.CR2 = (address << 1) | I2C_CR2_RD_WRN | (nbytes << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND;
        
        rx_dma_.CMAR = (uint32_t) data;
        rx_dma_.CNDTR = nbytes;
        rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
        regs_.CR2 |= I2C_CR2_START;
    }
    bool ready() const {
        return rx_dma_.CNDTR || tx_dma_.CNDTR;
    }
 private:
    void clear_isr() {
        regs_.ICR |= 0x3F38;
    }
    I2C_TypeDef &regs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
};
