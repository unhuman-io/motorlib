#pragma once
#include "encoder.h"
#include "util.h"

class ICPZ : public EncoderBase {
 public:
    ICPZ(SPIDMA &spidma) 
      : spidma_(spidma) {
      command_[0] = 0xa6; // read position
    }
    void init() {
       uint8_t command[6] = {0xcf, 0x40, 0x07}; // set memory bank 7
       uint8_t data_in[3];
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x09;
       command[2] = 0x00;     // multiturn data length = 0
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x40;
       command[2] = 0x08;     // memory bank 8
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x00;
       command[2] = 0;//0xbe;     // flexcode 446, 216
       command[3] = 0;//0x01;
       command[4] = 0;//216;
       command[5] = 0x00;
       spidma_.readwrite(command, data_in, 6);
       command[0] = 0xcf;
       command[1] = 0x40;
       command[2] = 0x00;     // memory bank 0
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x07;
       command[2] = 0x09;     // sys_ovr = 9 bit
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x40;
       command[2] = 0x01;     // memory bank 1
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x08;
       command[2] = 0xC0;     // ai_phase = -20
       command[3] = 0xF1;
       spidma_.readwrite(command, data_in, 4);
       command[0] = 0xcf;
       command[1] = 0x0a;
       command[2] = 0x00;     // ai_scale = .9558
       command[3] = 0xD8;
       spidma_.readwrite(command, data_in, 4);

    }
    void trigger() {
      spidma_.start_readwrite(command_, data_, 4);
    }
    int32_t read() {
      spidma_.finish_readwrite();
      uint32_t data = (data_[1] << 16) | (data_[2] << 8) | data_[3];
      pos_ += (int32_t) (data_ - last_data_); // rollover summing
      pos_ = data;
      last_data_ = data;
      return get_value();
    }
    int32_t get_value() const {
      return pos_;
    }
    bool index_received() const { return true; }

 private:
    SPIDMA &spidma_;
    uint8_t command_[4] = {};
    uint8_t data_[4] = {};
    int32_t pos_ = 0;
    uint32_t last_data_ = 0;
};
