#pragma once
// Device layer for the shared irq_init module: the interrupt lines of the
// NVIC. Their numbers are the IRQn names in STM32G564.h.
#include "st_device.h"

inline constexpr int irq_count = TIM27_IRQn + 1;
inline constexpr int irq_priority_bits = __NVIC_PRIO_BITS;
inline volatile uint8_t& irq_priority_reg(int irq) { return NVIC->IPR[irq]; }
