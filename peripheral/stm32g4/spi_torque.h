#pragma once
#include <cstdint>
#include "../st_device.h"
#include "../../gpio.h"
#include "../../torque_sensor.h"

// A two reading torque source
class SPITorque final : public TorqueSensor {
 public:
    SPITorque(SPI_TypeDef &regs, GPIO &gpio_cs, DMA_Channel_TypeDef &tx_dma, DMA_Channel_TypeDef &rx_dma) : 
        regs_(regs), gpio_cs_(gpio_cs), 
        tx_dma_(tx_dma), rx_dma_(rx_dma) {}

    void init() {
        regs_.CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
        tx_dma_.CMAR = (uint32_t) data_out_;
        tx_dma_.CPAR = (uint32_t) &regs_.DR;
        rx_dma_.CMAR = (uint32_t) data_in_;
        rx_dma_.CPAR = (uint32_t) &regs_.DR;
    }

    void trigger() {

        // set CS low
        gpio_cs_.clear();
        // start dma transfer
        tx_dma_.CCR = 0;
        rx_dma_.CCR = 0;
        tx_dma_.CNDTR = 9;
        rx_dma_.CNDTR = 9;        
        rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
    }

    float read() {
        static int count = 0;
        count ++;
        // wait until dma complete
        while(rx_dma_.CNDTR);
        // set CS high
        gpio_cs_.set();
        // process result
        result0_ = (uint32_t) data_in_[4] << 24 | (uint32_t) data_in_[3] << 16 | (uint16_t) data_in_[2] << 8 | data_in_[1];
        result1_ = (uint32_t) data_in_[8] << 24 | (uint32_t) data_in_[7] << 16 | (uint16_t) data_in_[6] << 8 | data_in_[5];
        int32_t diff = result0_ - result1_;
        float sum = (float) result0_ + (float) result1_;
        float tcomp = sum * k_temp_;
        torque_ = diff/sum * gain_ + bias_ + tcomp;
        return torque_;
    }
 private:
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
    uint8_t data_out_[9] = {0x40}; // read result 0
    uint8_t data_in_[9] = {};
    uint32_t result0_ = 0;
    uint32_t result1_ = 0;
    float torque_ = 0;
    float k_temp_=0, gain_ = 1, bias_=0;
};