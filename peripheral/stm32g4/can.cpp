#include "can.h"

CAN::CAN(CAN_TypeDef &regs) : regs_(regs) {}

void CAN::interrupt() {
}