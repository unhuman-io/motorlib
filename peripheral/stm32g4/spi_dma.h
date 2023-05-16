#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DMA_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DMA_H_

#include <cstdint>
#include "../st_device.h"
#include "../../gpio.h"
#include "../../util.h"

class SPIDMA {
 public:
    SPIDMA(SPI_TypeDef &regs, GPIO &gpio_cs, DMA_Channel_TypeDef &tx_dma, DMA_Channel_TypeDef &rx_dma, 
        uint16_t start_cs_delay_ns = 100, uint16_t end_cs_delay_ns = 100, volatile int *register_operation = nullptr, uint32_t regs_cr1 = 0) : 
        regs_(regs), gpio_cs_(gpio_cs),
        tx_dma_(tx_dma), rx_dma_(rx_dma),
        start_cs_delay_ns_(start_cs_delay_ns), end_cs_delay_ns_(end_cs_delay_ns),
        regs_cr1_(regs_cr1) {
            if (register_operation != nullptr) {
                register_operation_ = register_operation;
            }
            if (regs_cr1 == 0) {
                regs_cr1_ = regs.CR1;
            }
        reinit();
    }

    void reinit() {
        regs_.CR1 &= ~SPI_CR1_SPE; // disable to change settings
        regs_.CR2 = (7 << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;   // 8 bit
        tx_dma_.CPAR = (uint32_t) &regs_.DR;
        rx_dma_.CPAR = (uint32_t) &regs_.DR;
        regs_.CR1 = regs_cr1_ | SPI_CR1_SPE; // enable
    }

    void readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint8_t length) {
        start_readwrite(data_out, data_in, length);
        finish_readwrite();
    }

    void start_readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint8_t length) {
        if (!*register_operation_) {
            reinit();
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
    }

    void start_write(const uint8_t * const data_out, uint16_t length) {
        if (!*register_operation_) {
            gpio_cs_.clear();
            ns_delay(start_cs_delay_ns_);
            tx_dma_.CCR = 0;
            rx_dma_.CCR = 0;
            tx_dma_.CNDTR = length;
            rx_dma_.CNDTR = length;
            tx_dma_.CMAR = (uint32_t) data_out;
            rx_dma_.CMAR = (uint32_t) tmp_;        
            rx_dma_.CCR = DMA_CCR_EN;
            tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
        }
    }

    void finish_readwrite() {
        if (!*register_operation_) {
            while(rx_dma_.CNDTR);
            ns_delay(end_cs_delay_ns_);
            gpio_cs_.set();
        }
    }
    volatile int *register_operation_ = &register_operation_local_;
 private:
    volatile int register_operation_local_ = 0;
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
    uint16_t start_cs_delay_ns_;
    uint16_t end_cs_delay_ns_;
    uint32_t regs_cr1_;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DMA_H_
