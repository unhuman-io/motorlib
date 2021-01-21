#pragma once
#include "torque_sensor.h"
#include "util.h"

class ADS1235 : public TorqueSensorBase {
 public:
    ADS1235(SPIDMA &spidma) 
      : spidma_(spidma) {
      command_[0] = 0x12;
    }
    void init() {
       uint8_t command[2] = {0x42, 0x4B}; // 1200 SPS, sinc4 filter
       uint8_t data_in[2];
       spidma_.readwrite(command, data_in, 2);
       command[0] = 0x50;
       command[1] = 0x07;     // pga gain 128
       spidma_.readwrite(command, data_in, 2);
       command[0] = 0x51;
       command[1] = 0x34;     // inputs AIN0 AIN1
       spidma_.readwrite(command, data_in, 2);
    }
    void trigger() {
      spidma_.start_readwrite(command_, data_, 5);
    }
    float read() {
      spidma_.finish_readwrite();
      int32_t torque_int = signextend<int32_t, 24>((data_[2]) << 16 | (data_[3] << 8) | data_[4]);
      float torque = gain_*torque_int + bias_;
      return torque;
    }
 private:
    SPIDMA &spidma_;
    uint8_t command_[5] = {};
    uint8_t data_[5] = {};
};
