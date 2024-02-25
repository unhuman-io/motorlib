#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_PWM_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_PWM_H_

#include <cstdint>

class PWMBase {
 public:
   void set_voltage(float v_abc[3]) {}
   void set_vbus(float vbus) {}
   void open_mode() {}
   void brake_mode() {}
   void voltage_mode() {}
   void set_frequency_hz(uint32_t frequency_hz) {}
   bool is_voltage_saturated() const { return false; }
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_PWM_H_
