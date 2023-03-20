#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_HRPWM_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_HRPWM_H_

#include <cstdint>
#include <vector>

// TODO: This header is board specific, should remove dependency on st_device.h.
#include "../../../boost_g474/st_device.h"
#include "../pwm.h"

extern "C" {
void system_init();
}

class HRPWM : public PWMBase {
 public:
  HRPWM(HRTIM_TypeDef &regs, volatile uint32_t &pwm_a, volatile uint32_t &pwm_b,
        volatile uint32_t &pwm_c);
  HRPWM(uint32_t frequency_hz, HRTIM_TypeDef &regs, uint8_t ch_a, uint8_t ch_b,
        uint8_t ch_c, bool pwm3_mode = false, uint16_t deadtime_ns = 50,
        uint16_t min_off_ns = 0, uint16_t min_on_ns = 0);

  void init();
  void set_deadtime(uint16_t deadtime_ns);

  void set_frequency_multiplier(uint8_t frequency_multiplier);
  uint8_t get_frequency_multiplier() const;
  void set_voltage(float v_abc[3]) __attribute__((section(".ccmram")));
  void set_vbus(float vbus);
  void open_mode();
  void brake_mode();
  void voltage_mode();
  void set_frequency_hz(uint32_t frequency_hz);
  void set_frequency_hz(uint32_t frequency_hz, uint16_t min_off_ns,
                        uint16_t min_on_ns, bool keep_prescaler);

  uint16_t period_;
  uint16_t half_period_;
  uint32_t base_frequency_hz_;
  uint32_t current_frequency_hz_;
  uint32_t min_on_ns_;
  uint32_t min_off_ns_;
  HRTIM_TypeDef &regs_;
  volatile uint32_t &pwm_a_;
  volatile uint32_t &pwm_b_;
  volatile uint32_t &pwm_c_;
  uint8_t ch_a_;
  uint8_t ch_b_;
  uint8_t ch_c_;
  float v_to_pwm_;
  bool pwm3_mode_;
  uint16_t deadtime_ns_;
  float pwm_min_ = 0;
  float pwm_max_;
  int prescaler_ = 32;
  float count_per_ns_;
};

class HRPWM3 : public HRPWM {
 public:
  HRPWM3(uint32_t frequency_hz, HRTIM_TypeDef &regs, uint8_t ch_a, uint8_t ch_b,
         uint8_t ch_c, uint16_t min_off_ns = 0, uint16_t min_on_ns = 0);
  void brake_mode();
  void voltage_mode();
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_HRPWM_H_
