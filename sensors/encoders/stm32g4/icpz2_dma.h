#pragma once

#include "icpz_dma.h"
#include <cstring>

#define ICPZ2_SET_DEBUG_VARIABLES(prefix, api, icpz) \
    api.add_api_variable(prefix "1enc", new const APIInt32(&icpz.value1_));\
    api.add_api_variable(prefix "2enc", new const APIInt32(&icpz.value2_));\
    api.add_api_variable(prefix "1temp_nb", new const APICallbackFloat([]{ return icpz.get_temperature(0); }));\
    api.add_api_variable(prefix "2temp_nb", new const APICallbackFloat([]{ return icpz.get_temperature(1); }));\
    api.add_api_variable(prefix "1diag_nb", new const APICallback([]{ return icpz.get_diagnosis(0); }));\
    api.add_api_variable(prefix "2diag_nb", new const APICallback([]{ return icpz.get_diagnosis(1); }));\
    api.add_api_variable(prefix "1diag_str_nb", new const APICallback([]{ return icpz.get_diagnosis_str(0); }));\
    api.add_api_variable(prefix "2diag_str_nb", new const APICallback([]{ return icpz.get_diagnosis_str(1); }));\



class ICPZ2DMA : public EncoderBase {
 public:
    ICPZ2DMA(ICPZ &icpz, ICPZ &icpz2, DMAMUX_Channel_TypeDef &tx_dmamux, DMAMUX_Channel_TypeDef &rx_dmamux, uint8_t exti_num,
      void(*start_cs_trigger)(), void(*stop_cs_trigger_and_wait_cs_high)()) : icpz_(icpz), icpz2_(icpz2), spidma_(icpz.spidma_), 
      dmamux_tx_regs_(tx_dmamux), dmamux_rx_regs_(rx_dmamux), exti_num_(exti_num),
      start_cs_trigger_(start_cs_trigger), stop_cs_trigger_and_wait_cs_high_(stop_cs_trigger_and_wait_cs_high) {
      
      spidma_.pause_.start_callback_ = [this]{start_continuous_read();};
      spidma_.pause_.stop_callback_ = [this]{stop_continuous_read();};

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
      command_mult_[4][0][1] = ICPZ::Addr::COMMANDS;
      command_mult_[4][0][2] = ICPZ::CMD::SCLEAR;
      command_mult_[4][1][0] = 0xa6; // read position
      command_mult_[4][2][0] = 0xa6; // read position
      
      command_mult_[5][0][0] = 0xcf; // clear diagnosis
      command_mult_[5][0][1] = ICPZ::Addr::COMMANDS;
      command_mult_[5][0][2] = ICPZ::CMD::SCLEAR;
      command_mult_[5][1][0] = 0xa6; // read position
      command_mult_[5][2][0] = 0xa6; // read position
    }

    bool init() {
      bool result = icpz_.init();
      result &= icpz2_.init();
      result &= icpz_.set_register(7, 0, {0xFF, 0xFF, 0x00, 0xF3}); // enable all errors, report in diagnosis, except multiturn, gpio
      result &= icpz2_.set_register(7, 0, {0xFF, 0xFF, 0x00, 0xF3});
      start_continuous_read();
      return result;
    }
    void trigger() {}

    int32_t read() {
      if (!spidma_.pause_.is_paused()) {
        uint32_t current_buf_index = current_buffer_index();
        uint8_t *data_buf1 = data_mult_[current_buf_index][1];
        uint8_t *data_buf2 = data_mult_[current_buf_index][2];
        value1_ = icpz_.read_buf(data_buf1);
        value2_ = icpz2_.read_buf(data_buf2);
        if (current_buf_index == 1) {
          // copy temperature data to extra buffer
          std::memcpy(data_temperature_[0], &data_mult_[0][0][3], 2);
          std::memcpy(data_temperature_[1], &data_mult_[1][0][3], 2);
        } else if (current_buf_index == 3) {
          // copy diag data to extra buffer
          std::memcpy(&diag_[0], &data_mult_[2][0][2], 4);
          std::memcpy(&diag_[1], &data_mult_[3][0][2], 4);
        }
      }
      return value1_;
    }


    float get_temperature(int index) {
      return ICPZ::get_temperature(data_temperature_[index]);
    }

    std::string get_diagnosis(int index) {
      return bytes_to_hex((uint8_t*) &diag_[index], 4);
    }
    std::string get_diagnosis_str(int index) {
      return ICPZ::diagnosis_to_str(diag_[index]);
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

    uint8_t command_mult_[6][3][6] = {};
    uint8_t data_mult_[6][3][6] = {};
    uint8_t data_temperature_[2][2] = {};
    uint32_t diag_[2] = {};
    ICPZ &icpz_;
    ICPZ &icpz2_;
    SPIDMA &spidma_;
    int32_t value1_ = 0, value2_ = 0;

    bool inited_ = false;
    DMAMUX_Channel_TypeDef &dmamux_tx_regs_;
    DMAMUX_Channel_TypeDef &dmamux_rx_regs_;
    uint8_t exti_num_;
    void (*start_cs_trigger_)();
    void (*stop_cs_trigger_and_wait_cs_high_)();
};
