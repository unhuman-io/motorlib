#ifndef UNHUMAN_MOTORLIB_GPIO_HARDWARE_BRAKE_H_
#define UNHUMAN_MOTORLIB_GPIO_HARDWARE_BRAKE_H_

#include "gpio.h"
#include "hardware_brake.h"

// sets/resets a gpio to control a hardware brake
class GPIOHardwareBrake : public HardwareBrakeBase {
 public:
  GPIOHardwareBrake(GPIO &gpio) : gpio_(gpio) {}
  void on() { gpio_.clear(); }
  void off() { gpio_.set(); }

 private:
  GPIO &gpio_;
};

#endif  // UNHUMAN_MOTORLIB_GPIO_HARDWARE_BRAKE_H_
