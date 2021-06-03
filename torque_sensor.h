#pragma once
#include "messages.h"

class TorqueSensorBase {
 public:
    void init() {}
    void trigger() {}
    float read() { return bias_; }
    void set_param(const TorqueSensorParam &param) {
        gain_ = param.gain;
        bias_ = param.bias;
        k_temp_ = param.k_temp;
    }
 protected:
    float gain_ = 0, bias_ = 0, k_temp_ = 0, torque_ = 0;
    friend class System;
};
