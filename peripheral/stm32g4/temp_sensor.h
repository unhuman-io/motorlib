#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_TEMP_SENSOR_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_TEMP_SENSOR_H_

extern "C" {
void system_init();
}

#include "pin_config.h"
class TempSensor {
 public:
  TempSensor() {}
  // temperature in C
  void set_value(float value) { bias_ = value - read(); }
  float read() {
    value_ = (110.0 - 30.0) / (*TS_CAL2 - *TS_CAL1) *
                 ((int16_t)V_TEMP_DR / 3.3 - *TS_CAL1) +
             30 + bias_;

    return value_;
  }
  float get_value() const { return value_; }

 private:
  const uint16_t* const TS_CAL1 = (const uint16_t* const)0x1fff75a8;
  const uint16_t* const TS_CAL2 = (const uint16_t* const)0x1fff75ca;
  float value_ = 0;
  float bias_ = 0;

  friend void system_init();
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_TEMP_SENSOR_H_
