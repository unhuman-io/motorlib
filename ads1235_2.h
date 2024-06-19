#ifndef UNHUMAN_MOTORLIB_ADS1235_2_H_
#define UNHUMAN_MOTORLIB_ADS1235_2_H_

#include "ads1235.h"
#include "logger.h"

class ADS1235_2 : public ADS1235 {
 public:
    ADS1235_2(SPIDMA &spidma) : 
        ADS1235(spidma) {
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
        if (torque1_ == last_torque1_) {
          error_count_++;
          if (error_count_ > 10) {
            auto a = init();
            logger.log("torque_sensor_init: " + std::to_string(a));
          }
        } else {
          error_count_ = 0;
        }
        last_torque1_ = torque1_;
        toggle_ ^= true;
        torque_diff_ = torque1_ - torque2_;
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
    uint8_t error_count_ = 0;
    float last_torque1_ = 0;

    friend class System;
    friend void system_init();
    friend void config_init();
};

#endif  // UNHUMAN_MOTORLIB_ADS1235_2_H_
