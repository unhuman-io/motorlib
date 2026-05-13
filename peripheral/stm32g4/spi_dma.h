#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DMA_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DMA_H_

#include <cstdint>
#include "../spi_dma.h"
#include "dma.h"
#include "st_device.h"
#include "../../gpio.h"
#include "../../util.h"
#include "../../logger.h"

struct SPIConfig {
    enum {
        SP1,
        SP2,
        SP3,
        SP4,
        NUM_SPIS,
    } inst;
    bool uses_cs = true;
    enum {A, B, C, D, E, F, G} cs_port;
    uint8_t cs_pin;
    DMA_CHANNEL_INSTANCE tx_channel, rx_channel;
    uint16_t start_cs_delay_ns = 100;
    uint16_t end_cs_delay_ns = 100;
    uint32_t regs_cr1 = 0;
    uint16_t interframe_delay_ns = 0;
};

extern SPIPause spi_pause[SPIConfig::NUM_SPIS];

template<SPIConfig spi_config>
class SPIDMA : public SPIDMABase<SPIDMA<spi_config>> {
 public:
    SPIDMA() : SPIDMABase<SPIDMA<spi_config>>(0, spi_pause[spi_config.inst]) {
        this->reinit();
    }

    void reinit_impl() {
        regs().CR1 &= ~SPI_CR1_SPE; // disable to change settings
        regs().CR2 = (7 << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;   // 8 bit
        tx_dma().CPAR = (uint32_t) &regs().DR;
        rx_dma().CPAR = (uint32_t) &regs().DR;
        regs().CR1 = regs_cr1_ | SPI_CR1_SPE; // enable
    }

    void start_continuous_readwrite_impl(const uint8_t * const data_out, uint8_t * const data_in, uint8_t length) {
        length_ = length;
        tx_dma().CCR = 0;
        rx_dma().CCR = 0;
        tx_dma().CNDTR = length;
        rx_dma().CNDTR = length;
        tx_dma().CMAR = (uint32_t) data_out;
        rx_dma().CMAR = (uint32_t) data_in;
        rx_dma().CCR = DMA_CCR_EN | DMA_CCR_CIRC | DMA_CCR_MINC;
        tx_dma().CCR = DMA_CCR_EN | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
    }

    void start_continuous_write_impl(const uint8_t * const data_out, uint8_t length) {
        length_ = length;
        tx_dma().CCR = 0;
        tx_dma().CNDTR = length;
        tx_dma().CMAR = (uint32_t) data_out;
        tx_dma().CCR = DMA_CCR_EN | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
    }

    void stop_continuous_readwrite_impl() {
        tx_dma().CCR = 0;
        rx_dma().CCR = 0;
    }

    void start_readwrite_impl(const uint8_t * const data_out, uint8_t * const data_in, uint8_t length) {
        clear_cs();
        ns_delay(start_cs_delay_ns_);
        length_ = length;
        tx_dma().CCR = 0;
        rx_dma().CCR = 0;
        tx_dma().CNDTR = length;
        rx_dma().CNDTR = length;
        tx_dma().CMAR = (uint32_t) data_out;
        rx_dma().CMAR = (uint32_t) data_in;      
        rx_dma().CCR = DMA_CCR_EN | DMA_CCR_MINC;
        // Not generally controlling DMA priority. Setting 1 here above default 0 addresses one specific issue.
        tx_dma().CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR | 1 << DMA_CCR_PL_Pos;; // DIR = 1 > read from memory
        time_start_ = get_clock();
    }

    void start_write_impl(const uint8_t * const data_out, uint16_t length) {
        clear_cs();
        ns_delay(start_cs_delay_ns_);
        length_ = length;
        tx_dma().CCR = 0;
        rx_dma().CCR = 0;
        tx_dma().CNDTR = length;
        rx_dma().CNDTR = length;
        tx_dma().CMAR = (uint32_t) data_out;
        rx_dma().CMAR = (uint32_t) tmp_rx_;        
        rx_dma().CCR = DMA_CCR_EN;
        // Not generally controlling DMA priority. Setting 1 here above default 0 addresses one specific issue.
        tx_dma().CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR | 1 << DMA_CCR_PL_Pos; // DIR = 1 > read from memory
        time_start_ = get_clock();
    }

    void finish_readwrite_impl() {
        uint8_t brr = (regs().CR1 & SPI_CR1_BR) >> SPI_CR1_BR_Pos;
        uint32_t timeout = (length_*8+3)*(2 << brr); // 3 extra bits time
        while(rx_dma().CNDTR && (get_clock() - time_start_ < timeout)); // Busy wait with timeout
        ns_delay(end_cs_delay_ns_);
        set_cs();
        tx_dma().CMAR = 0;
        rx_dma().CMAR = 0;
        ns_delay(interframe_delay_ns_);
    }

    Task<bool> finish_readwrite_async_impl(CycleScheduler& sched) {
        uint8_t brr = (regs().CR1 & SPI_CR1_BR) >> SPI_CR1_BR_Pos;
        uint32_t timeout = (length_*8+3)*(2 << brr); // 3 extra bits time
        while(rx_dma().CNDTR){// && (get_clock() - time_start_ < timeout)) {
            co_await sched.yield();
        }
        bool success = rx_dma().CNDTR == 0;
        ns_delay(end_cs_delay_ns_);
        set_cs();
        tx_dma().CMAR = 0;
        rx_dma().CMAR = 0;
        ns_delay(interframe_delay_ns_);
        co_return success;
    }

    void clear_cs() {
        if constexpr (spi_config.uses_cs) {
            gpio_cs().clear();
        }
    }

    void set_cs() {
        if constexpr (spi_config.uses_cs) {
            gpio_cs().set();
        }
    }

private:
    static constexpr uint16_t start_cs_delay_ns_ = spi_config.start_cs_delay_ns;
    static constexpr uint16_t end_cs_delay_ns_   = spi_config.end_cs_delay_ns;
    static constexpr uint16_t interframe_delay_ns_ = spi_config.interframe_delay_ns;
    static constexpr uint32_t regs_cr1_ = spi_config.regs_cr1;
    static inline GPIO gpio_cs() { 
        GPIO_TypeDef* port = (GPIO_TypeDef*)(GPIOA_BASE + (static_cast<int>(spi_config.cs_port) * 0x400));
        return GPIO(*port, spi_config.cs_pin, GPIO::OUTPUT); 
    }
    
    static inline SPI_TypeDef& regs() {
        SPI_TypeDef * const spi_regs[SPIConfig::NUM_SPIS] = {SPI1, SPI2, SPI3, SPI4};
        return *spi_regs[spi_config.inst]; 
    }
    
    static inline DMA_Channel_TypeDef& tx_dma() { 
        return *dma_ch_regs[spi_config.tx_channel]; 
    }

    static inline DMA_Channel_TypeDef& rx_dma() { 
        return *dma_ch_regs[spi_config.rx_channel]; 
    }

    uint32_t tmp_rx_;
    uint32_t length_;
    uint32_t time_start_;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DMA_H_
