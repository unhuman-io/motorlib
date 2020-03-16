#pragma once
#include "st_device.h"

class CAN {
 public:
    CAN(CAN_TypeDef &regs);
    void interrupt() {}
 private:
    CAN_TypeDef &regs_;
};