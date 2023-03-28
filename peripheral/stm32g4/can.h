#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_CAN_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_CAN_H_

#include "st_device.h"

class CAN {
 public:
  CAN(CAN_TypeDef &regs);
  void interrupt() {}

 private:
  CAN_TypeDef &regs_;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_CAN_H_
