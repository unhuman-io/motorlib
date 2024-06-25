//6channels
#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_TORQUE_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_TORQUE_H_

#include <cstdint>
#include "st_device.h"
#include "../../gpio.h"
#include "../../torque_sensor.h"
#include "../spi_dma.h"

extern "C" {
void system_init();
}

// A two reading torque source
class SPITorque final : public TorqueSensorBase {
 public:
    SPITorque(SPI_TypeDef &regs, GPIO &gpio_cs, DMA_Channel_TypeDef &tx_dma, DMA_Channel_TypeDef &rx_dma, 
        SPIPause &spi_pause, uint8_t decimation = 50) : 
        regs_(regs), gpio_cs_(gpio_cs), 
        tx_dma_(tx_dma), rx_dma_(rx_dma), 
        spi_pause_(spi_pause), decimation_(decimation) {}

    bool init() {
        reinit();
        reset();
        return true;
    }

    void reinit() {
        regs_.CR2 = (7 << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;    // 8 bit
        regs_.CR1 = SPI_CR1_MSTR | (3 << SPI_CR1_BR_Pos) | SPI_CR1_CPHA | SPI_CR1_SSI | SPI_CR1_SSM;    // baud = clock/16 spi mode 1
        rx_dma_.CPAR = (uint32_t) &regs_.DR;
        tx_dma_.CPAR = (uint32_t) &regs_.DR;
        tx_dma_.CMAR = (uint32_t) data_out_;
        rx_dma_.CMAR = (uint32_t) data_in_;
        regs_.CR1 |= SPI_CR1_SPE; // enable
        
    }

    float get_value() const { return torque_; }

    void trigger() {
        count_++;
        if (!spi_pause_.is_paused()) {
            if (count_ < decimation_) {
                return;
            }
            count_ = 0;
            reinit();

            // set CS low
            gpio_cs_.clear();
            // start dma transfer
            tx_dma_.CCR = 0;
            rx_dma_.CCR = 0;
            tx_dma_.CNDTR = 25;//6*4+1
            rx_dma_.CNDTR = 25;        
            rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
            tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
        }
    }

    float read() {
        if (!spi_pause_.is_paused()) {
            if (count_ == 0) {
                // wait until dma complete
                while(rx_dma_.CNDTR);
                // set CS high
                gpio_cs_.set();
                
                // process result

                result0_ = (uint32_t) data_in_[4] << 24 | (uint32_t) data_in_[3] << 16 | (uint16_t) data_in_[2] << 8 | data_in_[1];
                result1_ = (uint32_t) data_in_[8] << 24 | (uint32_t) data_in_[7] << 16 | (uint16_t) data_in_[6] << 8 | data_in_[5];
                result2_ = (uint32_t) data_in_[12] << 24 | (uint32_t) data_in_[11] << 16 | (uint16_t) data_in_[10] << 8 | data_in_[9];
                result3_ = (uint32_t) data_in_[16] << 24 | (uint32_t) data_in_[15] << 16 | (uint16_t) data_in_[14] << 8 | data_in_[13];
                result4_ = (uint32_t) data_in_[20] << 24 | (uint32_t) data_in_[19] << 16 | (uint16_t) data_in_[18] << 8 | data_in_[17];
                result5_ = (uint32_t) data_in_[24] << 24 | (uint32_t) data_in_[23] << 16 | (uint16_t) data_in_[22] << 8 | data_in_[21];

                // process offset A, B & C
                float prod_A = (uint32_t) result0_ * (uint32_t) result1_;
                float prod_B = (uint32_t) result2_ * (uint32_t) result3_;
                float prod_C = (uint32_t) result4_ * (uint32_t) result5_;

                int32_t diff_A = result0_ - result1_;
                int32_t diff_B = result3_ - result2_;
                int32_t diff_C = result4_ - result5_;

                int32_t offset_A = 0;
                int32_t offset_B = 0;
                int32_t offset_C = 0;

                int32_t gain_A = 1;
                int32_t gain_B = 1;
                int32_t gain_C = 1;
                int32_t gain_T = 1;

                // process torque A, B & C
                float torq_A = offset_A +  gain_A *(diff_A/prod_A);//gain_ *
                float torq_B = offset_B +  gain_B *(diff_B/prod_B);//gain_ *
                float torq_C = offset_C +  gain_C *(diff_C/prod_C);//gain_ * 
                
                torque_ = gain_T * (torq_A + torq_B + torq_C) / 3;                              
                }
            }
        
        return torque_;
    }

    void reset(uint32_t a=1) {
        spi_pause_.pause();
        reinit();
        if (a) {
            gpio_cs_.clear();
            // start dma transfer
            data_out_[0] = 0x88;
            tx_dma_.CCR = 0;
            rx_dma_.CCR = 0;
            tx_dma_.CNDTR = 1;
            rx_dma_.CNDTR = 1;        
            rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
            tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
            // wait until dma complete
            while(rx_dma_.CNDTR);
            // set CS high
            gpio_cs_.set();
            data_out_[0] = 0x40;
        }
        spi_pause_.unpause();
    }

    uint32_t reset2() {
        return 0;
    }

 private:
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
    uint8_t data_out_[6*4+1] = {0x40}; // read result 0
    uint8_t data_in_[6*4+1] = {};
    uint32_t result0_ = 0;
    uint32_t result1_ = 0;
    uint32_t result2_ = 0;
    uint32_t result3_ = 0;
    uint32_t result4_ = 0;
    uint32_t result5_ = 0;
    uint32_t sum_;
    //float torque_ = 0;
    uint8_t count_ = 0;
    SPIPause &spi_pause_;
    uint8_t decimation_;
    

    friend class System;
    friend void system_init();
    friend void config_init();
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_TORQUE_H_
