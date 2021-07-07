#pragma once
#include "ads1235.h"

class ADS1235_2 : public ADS1235 {
 public:
    ADS1235_2(SPIDMA &spidma, volatile int* register_operation = nullptr) : 
        ADS1235(spidma, register_operation) {
    }
    uint32_t init() {
      uint32_t retval = ADS1235::init();
      //retval += set_register(3, 0x11) ? 100000 : 0; // pulse conversion
      retval += set_register(2, 0x63); // 7200 SPS, sinc4 filter
      return retval;
    }
    void trigger() {
      if (count_++ > decimation_) {
        count_ = 0;
        ADS1235::trigger();
      }
    }
    float read() {
      if (count_ == 0) {
        uint8_t mux[] = {0x51, 0}; // input mux
        if (toggle_) {
          torque1_ = ADS1235::read();
          mux[1] = 0x56; // ain 2 & 3
        } else {
          torque2_ = ADS1235::read();
          mux[1] = 0x34; // ain 0 & 1
        }
        toggle_ ^= true;
        torque_diff_ = torque1_ - torque2_ + bias_;
        spidma_.readwrite(mux, data_, 2);
      }
      return torque_diff_;
    }
 private:
    float torque1_ = 0, torque2_ = 0, torque_diff_ = 0;
    uint16_t count_ = 0;
    uint16_t decimation_ = 20;
    uint16_t count2_ = 0;
    uint8_t toggle_ = true;

    friend class System;
    friend void system_init();
    friend void config_init();
};
