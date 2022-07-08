#pragma once
#include "encoder.h"
#include "util.h"
#include <vector>

class ICPZ : public EncoderBase {
 public:
    ICPZ(SPIDMA &spidma) 
      : spidma_(spidma) {
      command_[0] = 0xa6; // read position
    }
    bool init() {
      bool success = true;
      set_register(7, 9, {0});
      success = set_register(7, 9, {0}) ? success : false; // multiturn data length = 0
      success = set_register(0, 0, {3}) ? success : false; // fast speed on port a
      success = set_register(0, 0xF, {0}) ? success : false; // 0x00 ran_fld = 0 -> never update position based on absolute track after initial

      //  command[0] = 0xcf;
      //  command[1] = 0x40;
      //  command[2] = 0x08;     // memory bank 8
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x00;
      //  command[2] = 0xbe;     // flexcode 446, 216
      //  command[3] = 0x01;
      //  command[4] = 216;
      //  command[5] = 0x00;
      //  spidma_.readwrite(command, data_in, 6);
      //  command[0] = 0xcf;
      //  command[1] = 0x40;
      //  command[2] = 0x00;     // memory bank 0
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x00;
      //  command[2] = 0x03;     
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x07;
      //  command[2] = 0x90;     // now 8 // sys_ovr = 9 bit
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x0F;
      //  command[2] = 0x00;     // 0x00 ran_fld = 0 -> never update position based on absolute track after initial
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x40;
      //  command[2] = 0x01;     // memory bank 1
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x08;
      //  command[2] = 0xc0;     // ai_phase 
      //  command[3] = 0xd1;
      //  spidma_.readwrite(command, data_in, 4);
      //  command[0] = 0xcf;
      //  command[1] = 0x0a;
      //  command[2] = 0x00;     // ai_scale
      //  command[3] = 0xcd;
      //  spidma_.readwrite(command, data_in, 4);
      //  command[0] = 0xd9;
      //  command[1] = 0x41;     // write command conf write all    
      //  spidma_.readwrite(command, data_in, 2);
      //  ms_delay(10);
      //  command[0] = 0x81;
      //  command[1] = 0x76;
      //  command[2] = 0x00;     // cmd stat
      //  command[3] = 0x00;
      //  spidma_.readwrite(command, data_in, 4);
      //  if (data_in[3] == 0) { // cmd succeeded
      //    return true;
      //  }
       return success;
    }
    void trigger() {
      if (!*register_operation_) {
        ongoing_read_ = true;
        spidma_.start_readwrite(command_, data_, 4);
      }
    }
    int32_t read() {
      if (ongoing_read_) {
        spidma_.finish_readwrite();
        uint32_t data = ((data_[1] << 16) | (data_[2] << 8) | data_[3]) << 8;
        int32_t diff = (data - last_data_); // rollover summing
        pos_ += diff/256;
        //pos_ = data/256;
        last_data_ = data;
        ongoing_read_ = false;
      }
      return get_value();
    }
    int32_t get_value() const {
      return pos_;
    }
    bool index_received() const { return true; }

    void set_register_operation() {
       (*register_operation_)++;
    }
    void clear_register_operation() {
       (*register_operation_)--;
    }

        // non interrupt context
    std::vector<uint8_t> read_register(uint8_t address, uint8_t length) {
        (*register_operation_)++;
        std::vector<uint8_t> data_out(length+3, 0);
        data_out[0] = read_register_opcode_;
        data_out[1] = address;
        uint8_t data_in[length+3];
        
        if (type_ == PZ) {
          spidma_.readwrite(data_out.data(), data_in, length+3);
          (*register_operation_)--;
          return std::vector<uint8_t>(&data_in[3], &data_in[3+length]);
        } else {
          spidma_.readwrite(data_out.data(), data_in, 2);
          data_out[0] = 0xad;
          data_out[1] = 0;
          spidma_.readwrite(data_out.data(), data_in, length+2);
          (*register_operation_)--;
          return std::vector<uint8_t>(&data_in[2], &data_in[2+length]);
        }
    }

    // non interrupt context
    bool set_register(uint8_t bank, uint8_t address, const std::vector<uint8_t> &value) {
        (*register_operation_)++;
        uint8_t data_in[std::max(value.size(),(std::size_t) 3)];
        if (bank != bank_) {
          uint8_t data_out[] = {write_register_opcode_, 0x40, bank};
          spidma_.readwrite(data_out, data_in, 3);
          if (read_register(0x40, 1) != std::vector<uint8_t>{bank}) {
            system_log("ichaus bank " + std::to_string(read_register(0x40, 1)[0]) + " not " + std::to_string(bank));
            (*register_operation_)--;
            return false;
          }
          bank_ = bank;
        }
        std::vector<uint8_t> data_out = {write_register_opcode_, address};
        data_out.insert(data_out.end(), value.begin(), value.end());
        spidma_.readwrite(data_out.data(), data_in, data_out.size());
        bool retval = read_register(address, value.size()) == value;
        (*register_operation_)--;
        return retval;
    }

 protected:
    SPIDMA &spidma_;
    uint8_t command_[4] = {};
    uint8_t data_[4] = {};
    int32_t pos_ = 0;
    uint32_t last_data_ = 0;
    volatile int register_operation_local_ = 0;
    volatile int *register_operation_ = &register_operation_local_;
    uint8_t bank_ = 255;
    bool ongoing_read_ = false;
    uint8_t read_register_opcode_ = 0x81;
    uint8_t write_register_opcode_ = 0xcf;
    enum {PZ, MU} type_ = PZ;

};
