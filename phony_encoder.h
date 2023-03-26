#ifndef UNHUMAN_MOTORLIB_PHONY_ENCODER_H_
#define UNHUMAN_MOTORLIB_PHONY_ENCODER_H_

#include <cstdint>

#include "control_fun.h"
#include "encoder.h"
#include "util.h"

class PhonyEncoder final : public EncoderBase {
 public:
  PhonyEncoder(float velocity) : EncoderBase() {
    velocity_multiplier_ = velocity / (CPU_FREQUENCY_HZ);
    last_clock_ = get_clock();
  }
  int32_t read() {
    value_.add(velocity_multiplier_ * (get_clock() - last_clock_));
    last_clock_ = get_clock();
    return value_.value();
  }
  int32_t get_value() const { return value_.value(); }
  __attribute__((section(".ccmram")));
  bool index_received() { return true; }

 private:
  float velocity_multiplier_;
  KahanSum value_;
  uint32_t last_clock_;
};

#endif  // UNHUMAN_MOTORLIB_PHONY_ENCODER_H_
