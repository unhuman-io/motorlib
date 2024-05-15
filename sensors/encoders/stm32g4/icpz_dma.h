#pragma once

#include "../icpz.h"

class ICPZDMA : public ICPZBase<ICPZDMA> {
 public:
    ICPZDMA(SPIDMA &spidma, DMAMUX_Channel_TypeDef &tx_dmamux, DMAMUX_Channel_TypeDef &rx_dmamux, uint8_t exti_num,
      void(*start_cs_trigger)(), void(*stop_cs_trigger_and_wait_cs_high)(), Disk disk = Default) : 
      ICPZBase(spidma, disk), dmamux_tx_regs_(tx_dmamux), dmamux_rx_regs_(rx_dmamux), exti_num_(exti_num),
      start_cs_trigger_(start_cs_trigger), stop_cs_trigger_and_wait_cs_high_(stop_cs_trigger_and_wait_cs_high) {
      spidma_.pause_.start_callback_ = [this]{start_continuous_read();};
      spidma_.pause_.stop_callback_ = [this]{stop_continuous_read();};
    }

    bool init() {
      bool result = ICPZBase::init();
      inited_ = true;
      start_continuous_read();
      return result;
    }
    void trigger() {}
    int32_t read() {
      if (!*register_operation_) {
        // Can only read when transactions are not active. Must be guaranteed 
        // by timing setup.
        uint32_t data = ((data_[1] << 16) | (data_[2] << 8) | data_[3]) << 8;
        raw_value_ = data >> 8;
        uint32_t word = data | data_[4];
        Diag diag = {.word = data_[4]};
        uint8_t crc6_calc = ~CRC_BiSS_43_30bit(word >> 6) & 0x3f;
        error_count_ += !diag.nErr;
        warn_count_ += !diag.nWarn;
        uint8_t crc_error = diag.crc6 == crc6_calc ? 0 : 1;
        crc_error_count_ += crc_error;
        if (!crc_error) {
          int32_t diff = (data - last_data_); // rollover summing
          pos_ += diff/256;
          //pos_ = data/256;
          last_data_ = data;
        }
        if (!diag.nErr) {
          //clear_diag();
        }
        ongoing_read_ = false;
      }
      return get_value();
    }
    void start_continuous_read() {
      dmamux_tx_regs_.CCR |= exti_num_ << DMAMUX_CxCR_SYNC_ID_Pos | 4 << DMAMUX_CxCR_NBREQ_Pos | 2 << DMAMUX_CxCR_SPOL_Pos | DMAMUX_CxCR_SE;
      dmamux_rx_regs_.CCR |= 4 << DMAMUX_CxCR_NBREQ_Pos | DMAMUX_CxCR_EGE;
      spidma_.start_continuous_readwrite(command_, data_, sizeof(command_));
      // start automatic CS
      //HRTIM1->sTimerxRegs[0].TIMxDIER = HRTIM_TIMDIER_CMP1DE;
      start_cs_trigger_();
     // HRTIM1->sTimerxRegs[0].CMP1xR = 47000;
      //DMA1_Channel5->CCR = DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1;
    }
    void stop_continuous_read() {
      // stop automatic CS
     // DMA1_Channel5->CCR = 0;
     // HRTIM1->sTimerxRegs[0].CMP1xR = 0;
      //HRTIM1->sTimerxRegs[0].TIMxDIER = 0;
      stop_cs_trigger_and_wait_cs_high_();
      // wait for CS high
      //while(!(GPIOD->IDR & 0x4));
      spidma_.stop_continuous_readwrite();
      dmamux_tx_regs_.CCR &= DMAMUX_CxCR_DMAREQ_ID_Msk;
      dmamux_rx_regs_.CCR &= DMAMUX_CxCR_DMAREQ_ID_Msk;
    }

    void set_register_operation_impl() {
      (*register_operation_)++;
      if (inited_ && *register_operation_ == 1) {
        stop_continuous_read();
      }
    }
    void clear_register_operation_impl() {
      if (inited_ && *register_operation_ == 1) {
        start_continuous_read();
      }
      (*register_operation_)--;
    }

    bool inited_ = false;
    DMAMUX_Channel_TypeDef &dmamux_tx_regs_;
    DMAMUX_Channel_TypeDef &dmamux_rx_regs_;
    uint8_t exti_num_;
    void (*start_cs_trigger_)();
    void (*stop_cs_trigger_and_wait_cs_high_)();
};
