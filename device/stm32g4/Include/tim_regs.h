#pragma once
// Device layer for the shared tim_init module: the timers TIM1..8, 15..17
// and 20, indexed by their number. Each has its own RCC enable (RCCInit
// tim1...). TIM2 and TIM5 count 32 bits, the others 16. TIM1, 8, 15, 16, 17
// and 20 have a break input and need the main output enable.
#include <cstddef>
#include "st_device.h"

inline constexpr int tim_count = 20;    // the highest timer number
inline constexpr uint32_t tim_base[tim_count] = {TIM1_BASE, TIM2_BASE, TIM3_BASE, TIM4_BASE, TIM5_BASE, TIM6_BASE, TIM7_BASE, TIM8_BASE,
    0, 0, 0, 0, 0, 0, TIM15_BASE, TIM16_BASE, TIM17_BASE, 0, 0, TIM20_BASE};
inline constexpr int tim_channels[tim_count] = {4, 4, 4, 4, 4, 0, 0, 4, 0, 0, 0, 0, 0, 0, 2, 1, 1, 0, 0, 4};
inline constexpr bool tim_32bit[tim_count] = {false, true, false, false, true};
inline constexpr bool tim_main_output[tim_count] = {true, false, false, false, false, false, false, true,
    false, false, false, false, false, false, true, true, true, false, false, true};

// checked against the shared register struct
namespace tim_offsets {
inline constexpr uint32_t DIER  = offsetof(TIM_TypeDef, DIER);
inline constexpr uint32_t EGR   = offsetof(TIM_TypeDef, EGR);
inline constexpr uint32_t CCMR1 = offsetof(TIM_TypeDef, CCMR1);
inline constexpr uint32_t CCER  = offsetof(TIM_TypeDef, CCER);
inline constexpr uint32_t CNT   = offsetof(TIM_TypeDef, CNT);
inline constexpr uint32_t PSC   = offsetof(TIM_TypeDef, PSC);
inline constexpr uint32_t ARR   = offsetof(TIM_TypeDef, ARR);
inline constexpr uint32_t CCR1  = offsetof(TIM_TypeDef, CCR1);
inline constexpr uint32_t CCR4  = offsetof(TIM_TypeDef, CCR4);
inline constexpr uint32_t BDTR  = offsetof(TIM_TypeDef, BDTR);
}

// checked against the shared bit positions
namespace tim_bits {
inline constexpr uint32_t CR1_CEN        = TIM_CR1_CEN;
inline constexpr uint32_t CR1_ARPE       = TIM_CR1_ARPE;
inline constexpr uint32_t DIER_UIE       = TIM_DIER_UIE;
inline constexpr uint32_t DIER_UDE       = TIM_DIER_UDE;
inline constexpr uint32_t EGR_UG         = TIM_EGR_UG;
inline constexpr uint32_t CCMR1_OC1PE    = TIM_CCMR1_OC1PE;
inline constexpr uint32_t CCMR1_OC1M_Pos = TIM_CCMR1_OC1M_Pos;
inline constexpr uint32_t CCMR1_OC2M_Pos = TIM_CCMR1_OC2M_Pos;
inline constexpr uint32_t CCER_CC1E      = TIM_CCER_CC1E;
inline constexpr uint32_t CCER_CC1P      = TIM_CCER_CC1P;
inline constexpr uint32_t CCER_CC2E      = TIM_CCER_CC2E;
inline constexpr uint32_t BDTR_MOE       = TIM_BDTR_MOE;
}
