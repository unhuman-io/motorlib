#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DMA_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DMA_H_

#include <cstdint>
#include "../spi_dma.h"
#include "dma.h"
#include "st_device.h"
#include "../../gpio.h"
#include "../../util.h"
#include "../../logger.h"
class SPIDMA : public SPIDMABase<SPIDMA> {
 public:
    enum SPI_INSTANCE {
        SP1,
        SP2,
        SP3,
        SP4,
        NUM_SPIS,
    };

    static constexpr SPI_TypeDef *spi_regs[NUM_SPIS] = {SPI1, SPI2, SPI3, SPI4};
    static SPIPause spi_pause[NUM_SPIS];

    SPIDMA(SPI_INSTANCE inst, GPIO &gpio_cs, DMA_CHANNEL_INSTANCE tx_channel, DMA_CHANNEL_INSTANCE rx_channel, 
        uint32_t baudrate,
        uint16_t start_cs_delay_ns = 100, uint16_t end_cs_delay_ns = 100, uint32_t regs_cr1 = 0) : 
        SPIDMABase(baudrate, spi_pause[inst]),
        regs_(*spi_regs[inst]), gpio_cs_(gpio_cs),
        tx_dma_(*dma_ch_regs[tx_channel]), rx_dma_(*dma_ch_regs[rx_channel]),
        start_cs_delay_ns_(start_cs_delay_ns), end_cs_delay_ns_(end_cs_delay_ns),
        regs_cr1_(regs_cr1) {

            if (regs_cr1 == 0) {
                regs_cr1_ = regs_.CR1;
            }
        logger.log_printf("SPI%d init, desired baudrate: %d, applied baudrate: %d", inst+1, baudrate, baudrate);
        reinit();
    }

    void reinit_impl() {
        regs_.CR1 &= ~SPI_CR1_SPE; // disable to change settings
        regs_.CR2 = (7 << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;   // 8 bit
        tx_dma_.CPAR = (uint32_t) &regs_.DR;
        rx_dma_.CPAR = (uint32_t) &regs_.DR;
        regs_.CR1 = regs_cr1_ | SPI_CR1_SPE; // enable
    }

    void start_continuous_readwrite_impl(const uint8_t * const data_out, uint8_t * const data_in, uint8_t length) {
        length_ = length;
        tx_dma_.CCR = 0;
        rx_dma_.CCR = 0;
        tx_dma_.CNDTR = length;
        rx_dma_.CNDTR = length;
        tx_dma_.CMAR = (uint32_t) data_out;
        rx_dma_.CMAR = (uint32_t) data_in;
        rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_CIRC | DMA_CCR_MINC;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
    }

    void start_continuous_write_impl(const uint8_t * const data_out, uint8_t length) {
        length_ = length;
        tx_dma_.CCR = 0;
        tx_dma_.CNDTR = length;
        tx_dma_.CMAR = (uint32_t) data_out;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
    }

    void stop_continuous_readwrite_impl() {
        tx_dma_.CCR = 0;
        rx_dma_.CCR = 0;
    }

    void start_readwrite_impl(const uint8_t * const data_out, uint8_t * const data_in, uint8_t length) {
        gpio_cs_.clear();
        ns_delay(start_cs_delay_ns_);
        length_ = length;
        tx_dma_.CCR = 0;
        rx_dma_.CCR = 0;
        tx_dma_.CNDTR = length;
        rx_dma_.CNDTR = length;
        tx_dma_.CMAR = (uint32_t) data_out;
        rx_dma_.CMAR = (uint32_t) data_in;      
        rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
        time_start_ = get_clock();
    }

    void start_write_impl(const uint8_t * const data_out, uint16_t length) {
        gpio_cs_.clear();
        ns_delay(start_cs_delay_ns_);
        length_ = length;
        tx_dma_.CCR = 0;
        rx_dma_.CCR = 0;
        tx_dma_.CNDTR = length;
        rx_dma_.CNDTR = length;
        tx_dma_.CMAR = (uint32_t) data_out;
        rx_dma_.CMAR = (uint32_t) tmp_rx_;        
        rx_dma_.CCR = DMA_CCR_EN;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
        time_start_ = get_clock();
    }

    void finish_readwrite_impl() {
        uint8_t brr = (regs_.CR1 & SPI_CR1_BR) >> SPI_CR1_BR_Pos;
        uint32_t timeout = (length_*8+3)*(2 << brr); // 3 extra bits time
        while(rx_dma_.CNDTR && (get_clock() - time_start_ < timeout)); // Busy wait with timeout
        ns_delay(end_cs_delay_ns_);
        gpio_cs_.set();
    }


    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
    uint16_t start_cs_delay_ns_;
    uint16_t end_cs_delay_ns_;
    uint32_t regs_cr1_;
    uint32_t tmp_rx_;
    uint32_t length_;
    uint32_t time_start_;

    volatile uint32_t old_cr1_, old_cr2_, old_tx_cpar_, old_rx_cpar_;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DMA_H_
