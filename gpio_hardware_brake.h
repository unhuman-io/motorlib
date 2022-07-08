#pragma once
#include "hardware_brake.h"
#include "gpio.h"

// sets/resets a gpio to control a hardware brake
class GPIOHardwareBrake : public HardwareBrakeBase {
 public:
    GPIOHardwareBrake(GPIO &gpio) : gpio_(gpio) {}
    void on() { gpio_.clear(); }
    void off() { gpio_.set(); }
 private:
    GPIO &gpio_;
};
