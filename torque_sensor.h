#pragma once
#include "messages.h"
#include "sensor.h"

class TorqueSensorBase : public SensorBase {
 public:
    bool init() { return true; }
    void trigger() {}
    float read() { return bias_; }
    void set_param(const TorqueSensorParam &param) {
        gain_ = param.gain;
        bias_ = param.bias;
        k_temp_ = param.k_temp;
    }
    // void zero(float current_torque_reading) {
    //     bias_ -= current_torque_reading;
    // }
 //protected:
    float gain_ = 0, bias_ = 0, k_temp_ = 0, torque_ = 0;
 //   friend class System;
};
