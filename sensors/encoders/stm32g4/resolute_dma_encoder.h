#pragma once

#include "../resolute_encoder.h"

#define RESOLUTE_DMA_SET_DEBUG_VARIABLES(prefix, api, resolute) \
    RESOLUTE_SET_DEBUG_VARIABLES(prefix, api, resolute) \

class ResoluteDMAEncoder : public ResoluteEncoder {
 public:
    ResoluteDMAEncoder(SPIDMA &spidma, DMAMUX_Channel_TypeDef &tx_dmamux, DMAMUX_Channel_TypeDef &rx_dmamux,
        uint8_t exti_num, void(*start_cs_trigger)(), void(*stop_cs_trigger_and_wait_cs_high)()) : 
        ResoluteEncoder(spidma), spidma_(spidma), dmamux_tx_regs_(tx_dmamux), dmamux_rx_regs_(rx_dmamux),
        exti_num_(exti_num),start_cs_trigger_(start_cs_trigger),
        stop_cs_trigger_and_wait_cs_high_(stop_cs_trigger_and_wait_cs_high) {
        spidma_.pause_.start_callback_ = [this]{start_continuous_read();};
        spidma_.pause_.stop_callback_ = [this]{stop_continuous_read();};
    }
    void trigger() {}
    bool init() { start_continuous_read(); return true; }
    int32_t read() {
        return ResoluteEncoder::read(data_in_);
    }
    void start_continuous_read() {
        dmamux_tx_regs_.CCR |= exti_num_ << DMAMUX_CxCR_SYNC_ID_Pos | (length_-1) << DMAMUX_CxCR_NBREQ_Pos | 2 << DMAMUX_CxCR_SPOL_Pos | DMAMUX_CxCR_SE;
        dmamux_rx_regs_.CCR |= (length_-1) << DMAMUX_CxCR_NBREQ_Pos | DMAMUX_CxCR_EGE;
        spidma_.start_continuous_readwrite(data_out_, data_in_, length_);
        start_cs_trigger_();
    }
    void stop_continuous_read() {
        // stop automatic CS
        stop_cs_trigger_and_wait_cs_high_();
        spidma_.stop_continuous_readwrite();
        dmamux_tx_regs_.CCR &= DMAMUX_CxCR_DMAREQ_ID_Msk;
        dmamux_rx_regs_.CCR &= DMAMUX_CxCR_DMAREQ_ID_Msk;
    }
  private:
    SPIDMA &spidma_;
    DMAMUX_Channel_TypeDef &dmamux_tx_regs_;
    DMAMUX_Channel_TypeDef &dmamux_rx_regs_;
    uint8_t exti_num_;
    void (*start_cs_trigger_)();
    void (*stop_cs_trigger_and_wait_cs_high_)();
};
