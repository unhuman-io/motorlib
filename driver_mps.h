#ifndef UNHUMAN_MOTORLIB_DRIVER_MPS_H_
#define UNHUMAN_MOTORLIB_DRIVER_MPS_H_

#include "driver.h"

class DriverMPS : public DriverBase {
 public:
  DriverMPS() {}

  void disable() {
    GPIOC->BSRR = GPIO_BSRR_BR13;  // drv disable
    DriverBase::disable();
  }

  void enable() {
    GPIOC->BSRR = GPIO_BSRR_BS13;  // drv enable
    DriverBase::enable();
  }

  std::string reset() {
    disable();
    ms_delay(10);
    enable();
    return "ok";
  }
};

#endif  // UNHUMAN_MOTORLIB_DRIVER_MPS_H_
