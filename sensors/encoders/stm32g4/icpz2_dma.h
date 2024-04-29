#pragma once

#include "icpz_dma.h"

class ICPZ2DMA : public EncoderBase {
 public:
    ICPZ2DMA(ICPZDMA icpz, ICPZDMA &icpz2, SPIDMA &spidma, DMAMUX_Channel_TypeDef &tx_dmamux, DMAMUX_Channel_TypeDef &rx_dmamux, uint8_t exti_num,
      void(*start_cs_trigger)(), void(*stop_cs_trigger_and_wait_cs_high)(), icpz_(icpz), icpz2_(icpz2) {

      // sequence:
      // temp1,  angle1, angle2,
      // temp2,  angle1, angle2,
      // diag1,  angle1, angle2,
      // diag2,  angle1, angle2,
      // cdiag1, angle1, angle2,
      // cdiag2, angle1, angle2,

      command_mult_[0][0][0] = 0x81; // read temperature
      command_mult_[0][0][1] = 0x4e;
      command_mult_[0][1][0] = 0xa6; // read position
      command_mult_[0][2][0] = 0xa6; // read position

      command_mult_[1][0][0] = 0x81; // read temperature
      command_mult_[1][0][1] = 0x4e;
      command_mult_[1][1][0] = 0xa6; // read position
      command_mult_[1][2][0] = 0xa6; // read position

      command_mult_[2][0][0] = 0x9c; // read diagnosis
      command_mult_[2][1][0] = 0xa6; // read position
      command_mult_[2][2][0] = 0xa6; // read position

      command_mult_[3][0][0] = 0x9c; // read diagnosis
      command_mult_[3][1][0] = 0xa6; // read position
      command_mult_[3][2][0] = 0xa6; // read position
      
      command_mult_[4][0][0] = 0xcf; // clear diagnosis
      command_mult_[4][0][1] = Addr::COMMANDS;
      command_mult_[4][0][2] = CMD::SCLEAR;
      command_mult_[4][1][0] = 0xa6; // read position
      command_mult_[4][2][0] = 0xa6; // read position
      
      command_mult_[5][0][0] = 0xcf; // clear diagnosis
      command_mult_[5][0][1] = Addr::COMMANDS;
      command_mult_[5][0][2] = CMD::SCLEAR;
      command_mult_[5][1][0] = 0xa6; // read position
      command_mult_[5][2][0] = 0xa6; // read position
    }

    bool init() {
      bool result = icpz_.init();
      result &= icpz2_.init();
      return result;
    }
    void trigger() {}

    int32_t read() {
      uint32_t current_buf_index = current_buffer_index();
      uint8_t *data_buf1 = data_mult_[current_buf_index][1];
      uint8_t *data_buf2 = data_mult_[current_buf_index][2];
      int32_t value1_ = icpz_.read_buf(data_buf1);
      int32_t value2_ = icpz2_.read_buf(data_buf2);

      return value1_;
    }


    float get_temperature(uint32_t index) {
      uint8_t *data = &data_mult_[index-1][0][3];
      return get_temperature(data);
    }

    uint32_t current_buffer_index() const {
      if (spidma_.rx_dma_.CNDTR > 5*18) {
        return 5;
      } else if (spidma_.rx_dma_.CNDTR > 4*18) {
        return 0;
      } else if (spidma_.rx_dma_.CNDTR > 3*18) {
        return 1;
      } else if (spidma_.rx_dma_.CNDTR > 2*18) {
        return 2;
      } else if (spidma_.rx_dma_.CNDTR > 1*18) {
        return 3;
      } else {
        return 4;
      }
    }
    void start_continuous_read() {
      dmamux_tx_regs_.CCR |= exti_num_ << DMAMUX_CxCR_SYNC_ID_Pos | 5 << DMAMUX_CxCR_NBREQ_Pos | 2 << DMAMUX_CxCR_SPOL_Pos | DMAMUX_CxCR_SE;
      dmamux_rx_regs_.CCR |= 5 << DMAMUX_CxCR_NBREQ_Pos | DMAMUX_CxCR_EGE;
      spidma_.start_continuous_readwrite(command_mult_[0][0], data_mult_[0][0], sizeof(command_mult_));
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

    uint8_t command_mult_[6][3][6] = {};
    uint8_t data_mult_[6][3][6] = {};
    ICPZDMA &icpz_;
    ICPZDMA &icpz2_;
    
};
