#pragma once

#include "icpz_dma.h"
#include <cstring>
#include "../../../control_fun.h"

#define ICPZ2_SET_DEBUG_VARIABLES(prefix, api, icpz) \
    ICPZ_SET_DEBUG_VARIABLES(prefix, api, icpz.icpz_);\
    ICPZ_SET_DEBUG_VARIABLES(prefix "2", api, icpz.icpz2_);\
    api.add_api_variable(prefix "1enc", new const APIUint32(&icpz.value1_.word));\
    api.add_api_variable(prefix "2enc", new const APIUint32(&icpz.value2_.word));\
    api.add_api_variable(prefix "3enc", new const APIUint32(&icpz.value3_.word));\
    api.add_api_variable(prefix "disagreement_error", new APIUint32(&icpz.disagreement_error_));\
    api.add_api_variable(prefix "1temp_nb", new const APICallbackFloat([]{ return icpz.get_temperature(0); }));\
    api.add_api_variable(prefix "2temp_nb", new const APICallbackFloat([]{ return icpz.get_temperature(1); }));\
    api.add_api_variable(prefix "1diag_nb", new const APICallback([]{ return icpz.get_diagnosis(0); }));\
    api.add_api_variable(prefix "2diag_nb", new const APICallback([]{ return icpz.get_diagnosis(1); }));\
    api.add_api_variable(prefix "1diag_str_nb", new const APICallback([]{ return icpz.get_diagnosis_str(0); }));\
    api.add_api_variable(prefix "2diag_str_nb", new const APICallback([]{ return icpz.get_diagnosis_str(1); }));\
    api.add_api_variable(prefix "1ai_phases_nb", new const APICallbackFloat([]{ return icpz.get_ai_phases(0); }));\
    api.add_api_variable(prefix "2ai_phases_nb", new const APICallbackFloat([]{ return icpz.get_ai_phases(1); }));\
    api.add_api_variable(prefix "diff", new const APIInt32(&icpz.diff_));\
    api.add_api_variable(prefix "value", new const APIUint32(&icpz.value_.word));\
    api.add_api_variable(prefix "ivalue", new const APICallbackInt32([] { return icpz.value_.ipos; }));\
    api.add_api_variable(prefix "use_encoder", new APIUint8(&icpz.use_encoder_));\
    api.add_api_variable(prefix "1remapped_error_count", new const APIUint32(&icpz.remapped_error_count_[0]));\
    api.add_api_variable(prefix "2remapped_error_count", new const APIUint32(&icpz.remapped_error_count_[1]));\
    api.add_api_variable(prefix "1remapped_warn_count", new const APIUint32(&icpz.remapped_warn_count_[0]));\
    api.add_api_variable(prefix "2remapped_warn_count", new const APIUint32(&icpz.remapped_warn_count_[1]));\



class ICPZ2 : public ICPZBase<ICPZ2> {
  public:
    ICPZ2(SPIDMA &spidma, Disk disk = Default) : ICPZBase(spidma, disk) {}
    void enable_commands_impl() {
      command_mult_[0] = ICPZ::Opcode::WRITE_REG; // clear diagnosis
      command_mult_[1] = Addr::COMMANDS;
      command_mult_[2] = CMD::SCLEAR;
    }
    void disable_commands_impl() {
      command_mult_[0] = 0;
      command_mult_[1] = 0;
      command_mult_[2] = 0;
    }
    void restore_bank_impl() {
      set_bank(1); // restore bank 1 to read ai_phases 1/0x28
    }
    uint8_t *command_mult_;
};

class ICPZ2DMA : public EncoderBase {
 public:
    ICPZ2DMA(ICPZ2 &icpz, ICPZ2 &icpz2, DMAMUX_Channel_TypeDef &tx_dmamux, DMAMUX_Channel_TypeDef &rx_dmamux, uint8_t exti_num,
      void(*start_cs_trigger)(), void(*stop_cs_trigger_and_wait_cs_high)()) : icpz_(icpz), icpz2_(icpz2), spidma_(icpz.spidma_), 
      dmamux_tx_regs_(tx_dmamux), dmamux_rx_regs_(rx_dmamux), exti_num_(exti_num),
      start_cs_trigger_(start_cs_trigger), stop_cs_trigger_and_wait_cs_high_(stop_cs_trigger_and_wait_cs_high) {
      
      spidma_.pause_.start_callback_ = [this]{start_continuous_read();};
      spidma_.pause_.stop_callback_ = [this]{stop_continuous_read();};
      icpz_.command_mult_ = command_mult_[4][0];
      icpz2_.command_mult_ = command_mult_[5][0];

      // sequence:
      // temp1,  angle1, angle2,
      // temp2,  angle1, angle2,
      // diag1,  angle1, angle2,
      // diag2,  angle1, angle2,
      // cdiag1, angle1, angle2,
      // cdiag2, angle1, angle2,
      // ai_phases1, angle1, angle2,
      // ai_phases2, angle1, angle2,

      command_mult_[0][0][0] = ICPZ::Opcode::READ_REG; // read temperature
      command_mult_[0][0][1] = 0x4e;
      command_mult_[0][1][0] = ICPZ::Opcode::READ_POS; // read position
      command_mult_[0][2][0] = ICPZ::Opcode::READ_POS; // read position

      command_mult_[1][0][0] = ICPZ::Opcode::READ_REG; // read temperature
      command_mult_[1][0][1] = 0x4e;
      command_mult_[1][1][0] = ICPZ::Opcode::READ_POS; // read position
      command_mult_[1][2][0] = ICPZ::Opcode::READ_POS; // read position

      command_mult_[2][0][0] = ICPZ::Opcode::READ_DIAG; // read diagnosis
      command_mult_[2][1][0] = ICPZ::Opcode::READ_POS; // read position
      command_mult_[2][2][0] = ICPZ::Opcode::READ_POS; // read position

      command_mult_[3][0][0] = ICPZ::Opcode::READ_DIAG; // read diagnosis
      command_mult_[3][1][0] = ICPZ::Opcode::READ_POS; // read position
      command_mult_[3][2][0] = ICPZ::Opcode::READ_POS; // read position
      
      command_mult_[4][0][0] = ICPZ::Opcode::WRITE_REG; // clear diagnosis
      command_mult_[4][0][1] = ICPZ::Addr::COMMANDS;
      command_mult_[4][0][2] = ICPZ::CMD::SCLEAR;
      command_mult_[4][1][0] = ICPZ::Opcode::READ_POS; // read position
      command_mult_[4][2][0] = ICPZ::Opcode::READ_POS; // read position
      
      command_mult_[5][0][0] = ICPZ::Opcode::WRITE_REG; // clear diagnosis
      command_mult_[5][0][1] = ICPZ::Addr::COMMANDS;
      command_mult_[5][0][2] = ICPZ::CMD::SCLEAR;
      command_mult_[5][1][0] = ICPZ::Opcode::READ_POS; // read position
      command_mult_[5][2][0] = ICPZ::Opcode::READ_POS; // read position

      command_mult_[6][0][0] = ICPZ::Opcode::READ_REG; // read ai_phases
      command_mult_[6][0][1] = 0x28;
      command_mult_[6][1][0] = ICPZ::Opcode::READ_POS; // read position
      command_mult_[6][2][0] = ICPZ::Opcode::READ_POS; // read position

      command_mult_[7][0][0] = ICPZ::Opcode::READ_REG; // read ai_phases
      command_mult_[7][0][1] = 0x28;
      command_mult_[7][1][0] = ICPZ::Opcode::READ_POS; // read position
      command_mult_[7][2][0] = ICPZ::Opcode::READ_POS; // read position
    }

    bool init() {
      bool result = true;
      if (!icpz_.init()) {
        logger.log("icpz1 init failed");
        result = false;
      }
      if (!icpz2_.init()) {
        logger.log("icpz2 init failed");
        //result = false;
      }
      result &= icpz_.set_register(7, 0, {0xFF, 0xFF, 0x00, 0xF3}); // enable all errors, report in diagnosis, except multiturn, gpio
      result &= icpz2_.set_register(7, 0, {0xFF, 0xFF, 0x00, 0xF3});
      clear_faults();
      icpz_.trigger();
      icpz2_.trigger();
      int32_t val1 = icpz_.read();
      int32_t val2 = icpz2_.read();
      if (std::abs(val1 - val2) > disagreement_tolerance_) {
        logger.log_printf("icpz2 disagreement error: %ld, %ld vs %ld, tol %ld", 
          std::abs(val1 - val2), val1, val2, disagreement_tolerance_);
        result = false;
      }
      start_continuous_read();
      return result;
    }
    void trigger() {}

    int32_t read() {
      if (!spidma_.pause_.is_paused()) {
        uint32_t current_buf_index = current_buffer_index();
        uint8_t *data_buf1 = data_mult_[current_buf_index][1];
        uint8_t *data_buf2 = data_mult_[current_buf_index][2];
        bool crc_error1, crc_error2;
        value1_.word = icpz_.read_raw_buf(data_buf1, crc_error1);
        value2_.pos = icpz2_.read_raw_buf(data_buf2, crc_error2) + (uint32_t) pow(2, 23);
        ICPZ::Encoder24 diff_ref = {};
        diff_ref.ipos = value1_.ipos - value2_.ipos;
        diff_ = diff_ref.ipos;
        // A way of averaging the two 24 bit rolling over values
        value3_.pos = value2_.word + diff_/2;

        if (!crc_error1 & !crc_error2) {
          switch (use_encoder_) {
            default:
            case 1:
              value_.word = value1_.word;
              break;
            case 2:
              value_.word = value2_.word;
              break;
            case 3:
              value_.ipos = value3_.ipos;
              break;
          }
          ICPZ::Encoder24 diffe = {};
          diffe.pos = value_.pos - last_value_.pos;
          int32_t diff = diffe.ipos;
          pos_ += diff;
          last_value_ = value_;

          if (std::abs(diff_ - (int32_t) pow(2, 23)) > disagreement_tolerance_) {
            disagreement_error_++;
          }
        }
        total_error_count_ = icpz_.error_count_ + icpz2_.error_count_ + disagreement_error_;
        total_crc_error_count_ = icpz_.crc_error_count_ + icpz2_.crc_error_count_;

        // save other diagnostic data to extra buffers
        if (current_buf_index == 1) {
          // copy temperature data to extra buffer
          std::memcpy(data_temperature_[0], &data_mult_[0][0][3], 2);
          std::memcpy(data_temperature_[1], &data_mult_[1][0][3], 2);
        } else if (current_buf_index == 3) {
          // copy diag data to extra buffer
          uint8_t *data = &data_mult_[2][0][2];
          last_diag_bits_[0].word = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
          data = &data_mult_[3][0][2];
          last_diag_bits_[1].word = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
          parse_diag_error();
        } else if (current_buf_index == 7) {
          // copy ai_phases data to extra buffer
          std::memcpy(data_phases_[0], &data_mult_[6][0][3], 2);
          std::memcpy(data_phases_[1], &data_mult_[7][0][3], 2);
        }
      }
      return get_value();
    }

    int32_t get_value() const { return pos_; }

    void clear_diag(int index) {
      diag_[index] = 0;
    }

    float get_temperature(int index) {
      return ICPZ::get_temperature(data_temperature_[index]);
    }

    float get_ai_phases(int index) {
      return ICPZ::get_ai_phase(data_phases_[index]);
    }

    std::string get_diagnosis(int index) {
      std::string s = bytes_to_hex((uint8_t*) &diag_[index], 4);
      clear_diag(index);
      return s;
    }

    std::string get_diagnosis_str(int index) {
      std::string s = ICPZ::diagnosis_to_str(diag_[index]);
      clear_diag(index);
      return s;
    }
    bool index_received() const { return true; }

    void clear_faults() {
      icpz_.clear_faults();
      icpz2_.clear_faults();
      clear_diag(0);
      clear_diag(1);
      disagreement_error_ = 0;
      total_error_count_ = 0;
      total_crc_error_count_ = 0;
    }

    uint32_t current_buffer_index() const {
      if (spidma_.rx_dma_.CNDTR > 7*18) {
        return 7;
      } else if (spidma_.rx_dma_.CNDTR > 6*18) {
        return 0;
      } else if (spidma_.rx_dma_.CNDTR > 5*18) {
        return 1;
      } else if (spidma_.rx_dma_.CNDTR > 4*18) {
        return 2;
      } else if (spidma_.rx_dma_.CNDTR > 3*18) {
        return 3;
      } else if (spidma_.rx_dma_.CNDTR > 2*18) {
        return 4;
      } else if (spidma_.rx_dma_.CNDTR > 1*18) {
        return 5;
      } else {
        return 6;
      }
    }
    void start_continuous_read() {
      if (!stopped_) {
        dmamux_tx_regs_.CCR |= exti_num_ << DMAMUX_CxCR_SYNC_ID_Pos | 5 << DMAMUX_CxCR_NBREQ_Pos | 2 << DMAMUX_CxCR_SPOL_Pos | DMAMUX_CxCR_SE;
        dmamux_rx_regs_.CCR |= 5 << DMAMUX_CxCR_NBREQ_Pos | DMAMUX_CxCR_EGE;
        spidma_.start_continuous_readwrite(command_mult_[0][0], data_mult_[0][0], sizeof(command_mult_));
        // start automatic CS
        start_cs_trigger_();
      }
    }
    void stop_continuous_read() {
      // stop automatic CS
      stop_cs_trigger_and_wait_cs_high_();
      // wait for CS high
      spidma_.stop_continuous_readwrite();
      dmamux_tx_regs_.CCR &= DMAMUX_CxCR_DMAREQ_ID_Msk;
      dmamux_rx_regs_.CCR &= DMAMUX_CxCR_DMAREQ_ID_Msk;
    }

    void stop() {
      stopped_ = true;
      stop_continuous_read();
    }


    void parse_diag_error() {
      diag_[0] |= last_diag_bits_[0].word;
      diag_[1] |= last_diag_bits_[1].word;
      for (int i=0; i<2; i++) {
        if (last_diag_bits_[i].word & diag_bits_error_mask_.word) {
          remapped_error_count_[i]++;
        }
        if (last_diag_bits_[i].word & diag_bits_warn_mask_.word) {
          remapped_warn_count_[i]++;
        }
      }
    }

    uint8_t command_mult_[8][3][6] = {};
    uint8_t data_mult_[8][3][6] = {};
    uint8_t data_temperature_[2][2] = {};
    uint8_t data_phases_[2][2] = {};
    uint32_t diag_[2] = {};
    ICPZ2 &icpz_;
    ICPZ2 &icpz2_;
    SPIDMA &spidma_;
    int32_t pos_ = 0;
    ICPZ::Encoder24 value_ = {}, last_value_ = {}, value1_ = {}, value2_ = {}, value3_ = {};
    int32_t diff_ = 0;
    uint32_t disagreement_error_ = 0;
    int32_t disagreement_tolerance_ = .1/2/M_PI*pow(2,24);
    uint32_t total_error_count_ = 0;
    uint32_t total_crc_error_count_ = 0;
    ICPZ::DiagBits last_diag_bits_[2] = {};
    ICPZ::DiagBits diag_bits_error_mask_ = {.word = 0xFFFFF7FF}; // no prc_sync_failed
    ICPZ::DiagBits diag_bits_warn_mask_ = {.prc_sync_failed = 1};
    uint32_t remapped_error_count_[2] = {};
    uint32_t remapped_warn_count_[2] = {};

    bool inited_ = false;
    DMAMUX_Channel_TypeDef &dmamux_tx_regs_;
    DMAMUX_Channel_TypeDef &dmamux_rx_regs_;
    uint8_t exti_num_;
    void (*start_cs_trigger_)();
    void (*stop_cs_trigger_and_wait_cs_high_)();
    uint8_t use_encoder_ = 1;
    bool stopped_ = false;
};
