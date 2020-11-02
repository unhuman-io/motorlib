#pragma once
#include <cstdint>
#include "../st_device.h"
#include "../../gpio.h"

class SPIDMA {
 public:
    SPIDMA(SPI_TypeDef &regs, GPIO &gpio_cs, DMA_Channel_TypeDef &tx_dma, DMA_Channel_TypeDef &rx_dma) : 
        regs_(regs), gpio_cs_(gpio_cs),
        tx_dma_(tx_dma), rx_dma_(rx_dma) {
            regs_.CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
            tx_dma_.CPAR = (uint32_t) &regs_.DR;
            rx_dma_.CPAR = (uint32_t) &regs_.DR;
    }

    void readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint8_t length) {
        start_readwrite(data_out, data_in, length);
        finish_readwrite();
    }

    void start_readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint8_t length) {
        gpio_cs_.clear();
        ns_delay(start_cs_delay_ns_);
        tx_dma_.CCR = 0;
        rx_dma_.CCR = 0;
        tx_dma_.CNDTR = length;
        rx_dma_.CNDTR = length;
        tx_dma_.CMAR = (uint32_t) data_out;
        rx_dma_.CMAR = (uint32_t) data_in;        
        rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
    }

    void finish_readwrite() {
        while(rx_dma_.CNDTR);
        ns_delay(end_cs_delay_ns_);
        gpio_cs_.set();
    }

 private:
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
    uint16_t start_cs_delay_ns_ = 100;
    uint16_t end_cs_delay_ns_ = 100;

    template<typename, typename>
    friend class System;
};