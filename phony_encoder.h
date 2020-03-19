#pragma once

#include <cstdint>
#include "encoder.h"
#include "util.h"
#include "control_fun.h"

class PhonyEncoder final : public Encoder {
 public:
    PhonyEncoder(float velocity) : Encoder() {
        velocity_multiplier_ = velocity/(2*M_PI*CPU_FREQUENCY_HZ);
        last_clock_ = get_clock();
    }
    virtual int32_t read() { 
      value_.add(velocity_multiplier_ * (get_clock() - last_clock_)); 
      last_clock_ = get_clock();
      return value_.value();
    }
    virtual int32_t get_value() const { return value_.value(); } __attribute__((section (".ccmram")));
    virtual bool index_received() { return true; }
 private:
    float velocity_multiplier_;
    KahanSum value_;
    uint32_t last_clock_;

};
