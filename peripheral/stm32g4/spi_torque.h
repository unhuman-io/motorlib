// 6 channels tony 20240704
#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_TORQUE_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_TORQUE_H_
#define TORQUE_SENSOR_BYTES 25 // (6*4) + 1, based on  4 bytes per number of channels (6), plus an instruction byte

#include <math.h>
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
            tx_dma_.CNDTR = TORQUE_SENSOR_BYTES;
            rx_dma_.CNDTR = TORQUE_SENSOR_BYTES;
            rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
            tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
        }
    }
    float read() {
        uint32_t dma_count = 1000000;
	uint32_t temp = 0;
	uint8_t duplicates = 0;
        if (!spi_pause_.is_paused()) {
            if (count_ == 0) {
                // wait until dma complete
	        while(rx_dma_.CNDTR)  {
		  if (!(--dma_count)) {
		    clear_output();
		    reset();
		    return torque_;
		  }
                }

                // set CS high
                gpio_cs_.set();

                // process result
		temp = (uint32_t) data_in_[4] << 24 | (uint32_t) data_in_[3] << 16 | (uint16_t) data_in_[2] << 8 | data_in_[1];
		if (result0_ == temp) duplicates++;
                result0_ = temp;
		temp = (uint32_t) data_in_[8] << 24 | (uint32_t) data_in_[7] << 16 | (uint16_t) data_in_[6] << 8 | data_in_[5];
		if (result1_ == temp) duplicates++;
                result1_ = temp;
		temp = (uint32_t) data_in_[12] << 24 | (uint32_t) data_in_[11] << 16 | (uint16_t) data_in_[10] << 8 | data_in_[9];
		if (result2_ == temp) duplicates++;
                result2_ = temp;
		temp = (uint32_t) data_in_[16] << 24 | (uint32_t) data_in_[15] << 16 | (uint16_t) data_in_[14] << 8 | data_in_[13];
		if (result3_ == temp) duplicates++;
                result3_ = temp;
		temp = (uint32_t) data_in_[20] << 24 | (uint32_t) data_in_[19] << 16 | (uint16_t) data_in_[18] << 8 | data_in_[17];
		if (result4_ == temp) duplicates++;
                result4_ = temp;
		temp = (uint32_t) data_in_[24] << 24 | (uint32_t) data_in_[23] << 16 | (uint16_t) data_in_[22] << 8 | data_in_[21];
		if (result5_ == temp) duplicates++;
                result5_ = temp;

		if (duplicates == 0) duplicates_ = 0;
		duplicates_ += duplicates;

		// this is a major hack - when the torque sensor stops working, or fails to start it produces
		// duplicates. For now, we detect if there are a lot of duplicates and if so we reset.
		// Noise ensures that we rarely if ever get duplicates in good conditions.
		if (duplicates_ > 300) {
		  clear_output();
		  reset();
		  return torque_;
		}

		const float down_scale = ((float)(1<<27)) / 15.0f;

                // process offset A, B & C
		if (result0_ != 0xFFFFFFFF && result1_ != 0xFFFFFFFF && result0_ && result1_) {
		  ratio_A_ = down_scale / ((float)result1_) - down_scale / ((float)result0_);
                  torque_A_ = offset_A + gain_A * ratio_A_;
		} else {
		  ratio_A_ = NAN;
		  torque_A_ = NAN;
		}
		if (result2_ != 0xFFFFFFFF && result3_ != 0xFFFFFFFF && result2_ && result3_) {
		  ratio_B_ = down_scale / ((float)result2_) - down_scale / ((float)result3_);
                  torque_B_ = offset_B + gain_B * ratio_B_;
		} else {
		  ratio_B_ = NAN;
		  torque_B_ = NAN;
		}
		if (result4_ != 0xFFFFFFFF && result5_ != 0xFFFFFFFF && result4_ && result5_) {
		  ratio_C_ = down_scale / ((float)result5_) - down_scale / ((float)result4_);
                  torque_C_ = offset_C + gain_C * ratio_C_;
		} else {
		  ratio_C_ = NAN;
		  torque_C_ = NAN;
		}

		if (!isnan(torque_A_) && !isnan(torque_B_) && !isnan(torque_C_)) {
                  torque_ = gain_T * (torque_A_ + torque_B_ + torque_C_) / 3.0f;
		} else {
		  torque_ = NAN;
		}
	    }
	}
        return torque_;
    }
    void reset(uint32_t a=1) {
        spi_pause_.pause();
	clear_output();
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
	duplicates_ = 0;
	clear_output();
        spi_pause_.unpause();
    }
    uint32_t reset2() {
        return 0;
    }
 private:
    void clear_output() {
      result0_ = 0;
      result1_ = 0;
      result2_ = 0;
      result3_ = 0;
      result4_ = 0;
      result5_ = 0;
      ratio_A_ =  NAN;
      ratio_B_ =  NAN;
      ratio_C_ =  NAN;
      torque_A_ = NAN;
      torque_B_ = NAN;
      torque_C_ = NAN;
      torque_ =   NAN;
    }
  
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
    uint8_t data_out_[TORQUE_SENSOR_BYTES] = {0x40}; // read result 0
    uint8_t data_in_[TORQUE_SENSOR_BYTES] = {};

    uint32_t result0_ = 0;
    uint32_t result1_ = 0;
    uint32_t result2_ = 0;
    uint32_t result3_ = 0;
    uint32_t result4_ = 0;
    uint32_t result5_ = 0;

    float ratio_A_ = NAN;
    float ratio_B_ = NAN;
    float ratio_C_ = NAN;

    float torque_A_ = NAN;
    float torque_B_ = NAN;
    float torque_C_ = NAN;

    uint32_t duplicates_ = 0;

    float offset_A = 0.001879708100018f;
    float offset_B = -0.002594712102985f;
    float offset_C = -0.002387269502772f;

    float gain_A = 1.0f;
    float gain_B = 1.04f;
    float gain_C = 0.61f;
    float gain_T = 313000;
    
    //float torque_ = 0;
    uint8_t count_ = 0;
    SPIPause &spi_pause_;
    uint8_t decimation_;
    friend class System;
    friend void system_init();
    friend void config_init();
};
#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_TORQUE_H_
