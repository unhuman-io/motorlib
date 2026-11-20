#pragma once

#include <cstdint>
#include "../spi_dma.h"
#include "dma.h"
#include "st_device.h"
#include "../../gpio.h"
#include "../../util.h"
#include "../../logger.h"
class SPIDMA_USART : public SPIDMABase<SPIDMA_USART> {
 public:
    enum SPI_USART_INSTANCE {
        SPI_USART1,
        SPI_USART2,
        SPI_USART3,
        NUM_USARTS,
    };

    USART_TypeDef * const spi_regs[NUM_USARTS] = {USART1, USART2, USART3};
    static SPIPause spi_usart_pause[NUM_USARTS];

    SPIDMA_USART(SPI_USART_INSTANCE inst, GPIO &gpio_cs, DMA_CHANNEL_INSTANCE tx_channel, DMA_CHANNEL_INSTANCE rx_channel,
        uint32_t baudrate,
        uint16_t start_cs_delay_ns = 100, uint16_t end_cs_delay_ns = 100, bool cpol = false, bool cpha = false, uint16_t interframe_delay_ns = 0) : 
        SPIDMABase(baudrate, spi_usart_pause[inst]),
        regs_(*spi_regs[inst]), gpio_cs_(gpio_cs),
        tx_dma_(*dma_ch_regs[tx_channel]), rx_dma_(*dma_ch_regs[rx_channel]),
        start_cs_delay_ns_(start_cs_delay_ns), end_cs_delay_ns_(end_cs_delay_ns),
        interframe_delay_ns_(interframe_delay_ns) {
            brr_ = CPU_FREQUENCY_HZ/baudrate;
            regs_cr2_ = 1 << USART_CR2_STOP_Pos | USART_CR2_CLKEN | cpol << USART_CR2_CPOL_Pos | cpha << USART_CR2_CPHA_Pos | USART_CR2_LBCL | USART_CR2_MSBFIRST;
        //logger.log_printf("SPI%d init, desired baudrate: %d, applied baudrate: %d", inst+1, baudrate, baudrate);
        reinit();
    }

    void reinit_impl() {
        regs_.CR1 &= ~USART_CR1_UE; // disable to change settings
        regs_.CR2 = regs_cr2_;
        regs_.CR3 = USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_ONEBIT; 
        regs_.BRR = brr_;
        tx_dma_.CPAR = (uint32_t) &regs_.TDR;
        rx_dma_.CPAR = (uint32_t) &regs_.RDR;
        regs_.CR1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // enable
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
        // Not generally controlling DMA priority. Setting 1 here above default 0 addresses one specific issue.
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR | 1 << DMA_CCR_PL_Pos;; // DIR = 1 > read from memory
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
        // Not generally controlling DMA priority. Setting 1 here above default 0 addresses one specific issue.
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR | 1 << DMA_CCR_PL_Pos; // DIR = 1 > read from memory
        time_start_ = get_clock();
    }

    void finish_readwrite_impl() {
        // bit number of cycles is brr_
        uint32_t timeout = 2*(length_*8+3)*brr_; // 3 extra bits time
        while(rx_dma_.CNDTR && (get_clock() - time_start_ < timeout)); // Busy wait with timeout
        ns_delay(end_cs_delay_ns_);
        gpio_cs_.set();
        ns_delay(interframe_delay_ns_);
    }


    USART_TypeDef &regs_;
    GPIO &gpio_cs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
    uint16_t brr_;
    uint16_t start_cs_delay_ns_;
    uint16_t end_cs_delay_ns_;
    uint32_t regs_cr2_;
    uint16_t interframe_delay_ns_;
    uint32_t tmp_rx_;
    uint32_t length_;
    uint32_t time_start_;

    template <typename T> friend class SystemBase;
};
