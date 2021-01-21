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
       uint8_t command[3] = {0xcf, 0x40, 0x07}; // set memory bank 7
       uint8_t data_in[3];
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x09;
       command[2] = 0x00;     // multiturn data length = 0
       spidma_.readwrite(command, data_in, 3);
    }
    void trigger() {
      spidma_.start_readwrite(command_, data_, 4);
    }
    int32_t read() {
      spidma_.finish_readwrite();
      pos_ = signextend<int32_t, 24>(data_[2] << 16 | data_[3] << 8 | data_[4]);
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
};
