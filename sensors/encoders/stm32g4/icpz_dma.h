#pragma once

#include "../icpz.h"

#define ICPZDMA_SET_DEBUG_VARIABLES(prefix, api, icpz) \
    ICPZ_SET_DEBUG_VARIABLES(prefix "1", api, icpz);\

class ICPZDMA : public ICPZBase<ICPZDMA> {
 public:
    ICPZDMA(SPIDMA &spidma, DMAMUX_Channel_TypeDef &tx_dmamux, DMAMUX_Channel_TypeDef &rx_dmamux, uint8_t exti_num,
      void(*start_cs_trigger)(), void(*stop_cs_trigger_and_wait_cs_high)(), Disk disk = Default) : 
      ICPZBase(spidma, disk), dmamux_tx_regs_(tx_dmamux), dmamux_rx_regs_(rx_dmamux), exti_num_(exti_num),
      start_cs_trigger_(start_cs_trigger), stop_cs_trigger_and_wait_cs_high_(stop_cs_trigger_and_wait_cs_high) {
      spidma_.pause_.start_callback_ = [this]{start_continuous_read();};
      spidma_.pause_.stop_callback_ = [this]{stop_continuous_read();};

      command_mult_[1][0] = ICPZ::Opcode::READ_POS; // read position
      command_mult_[3][0] = ICPZ::Opcode::READ_POS;
      command_mult_[5][0] = ICPZ::Opcode::READ_POS;
      command_mult_[7][0] = ICPZ::Opcode::READ_POS;
      command_mult_[0][0] = ICPZ::Opcode::READ_REG; // read temperature
      command_mult_[0][1] = 0x4e;
      command_mult_[2][0] = ICPZ::Opcode::READ_DIAG; // read diagnosis
      enable_commands_impl();
      command_mult_[6][0] = ICPZ::Opcode::READ_REG; // read ai_phases
      command_mult_[6][1] = 0x28;
    }

    bool init() {
      bool result = ICPZBase::init();
      result &= set_register(7, 0, {0xFF, 0xFF, 0x00, 0xF3}); // enable all errors, report in diagnosis, except multiturn, gpio
      inited_ = true;
      start_continuous_read();
      return result;
    }
    void trigger() {}
    int32_t read() {
      if (!spidma_.pause_.is_paused()) {
        // Can only read when transactions are not active. Must be guaranteed 
        // by timing setup.
        uint8_t *data_buf = data_mult_[current_buffer_index()];
        read_buf(data_buf);
      }
      or_diag();
      return get_value();
    }
    float get_temperature() {
      uint8_t *data = &data_mult_[0][3];
      // signed value, 16 bit
      return ICPZ::get_temperature(data);
    }

    float get_ai_phases() {
      uint8_t *data = &data_mult_[6][3];
      // signed value, 16 bit
      return ICPZ::get_ai_phase(data);
    }
    void or_diag() {
      uint8_t *data = &data_mult_[2][2];
      diag_ |= data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
    }
    void clear_diag() {
      diag_ = 0;
    }
    void enable_commands_impl() {
      command_mult_[4][0] = ICPZ::Opcode::WRITE_REG; // clear diagnosis
      command_mult_[4][1] = Addr::COMMANDS;
      command_mult_[4][2] = CMD::SCLEAR;
    }
    void disable_commands_impl() {
      command_mult_[4][0] = 0;
      command_mult_[4][1] = 0;
      command_mult_[4][2] = 0;
    }

    std::string read_diagnosis() {
      std::string s = bytes_to_hex((uint8_t*) &diag_, 4);
      clear_diag();
      return s;
    }
    std::string read_diagnosis_str() {
      std::string s = diagnosis_to_str(diag_);
      clear_diag();
      return s;
    }
    uint32_t current_buffer_index() const {
      if (spidma_.rx_dma_.CNDTR > 36) {
        return 7;
      } else if (spidma_.rx_dma_.CNDTR > 24) {
        return 1;
      } else if (spidma_.rx_dma_.CNDTR > 12) {
        return 3;
      } else {
        return 5;
      }
    }
    void start_continuous_read() {
      dmamux_tx_regs_.CCR |= exti_num_ << DMAMUX_CxCR_SYNC_ID_Pos | 5 << DMAMUX_CxCR_NBREQ_Pos | 2 << DMAMUX_CxCR_SPOL_Pos | DMAMUX_CxCR_SE;
      dmamux_rx_regs_.CCR |= 5 << DMAMUX_CxCR_NBREQ_Pos | DMAMUX_CxCR_EGE;
      spidma_.start_continuous_readwrite(command_mult_[0], data_mult_[0], sizeof(command_mult_));
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

    bool inited_ = false;
    DMAMUX_Channel_TypeDef &dmamux_tx_regs_;
    DMAMUX_Channel_TypeDef &dmamux_rx_regs_;
    uint8_t exti_num_;
    void (*start_cs_trigger_)();
    void (*stop_cs_trigger_and_wait_cs_high_)();
    uint8_t command_mult_[8][6] = {};
    uint8_t data_mult_[8][6] = {};
    uint32_t diag_ = {};
};
