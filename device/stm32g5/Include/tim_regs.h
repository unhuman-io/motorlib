#pragma once
// Device layer for the shared tim_init module: the timers TIM1..17, 20, 26
// and 27, indexed by their number. TIM2..5 count 32 bits, the others 16.
// TIM1, 8, 15, 16, 17 and 20 have a break input and need the main output
// enable. TIM6, 7, 26 and 27 have no channels.
#include <cstddef>
#include "st_device.h"

inline constexpr int tim_count = 27;    // the highest timer number
inline constexpr uint32_t tim_base[tim_count] = {TIM1_BASE, TIM2_BASE, TIM3_BASE, TIM4_BASE, TIM5_BASE, TIM6_BASE, TIM7_BASE, TIM8_BASE,
    TIM9_BASE, TIM10_BASE, TIM11_BASE, TIM12_BASE, TIM13_BASE, TIM14_BASE, TIM15_BASE, TIM16_BASE, TIM17_BASE, 0, 0, TIM20_BASE,
    0, 0, 0, 0, 0, TIM26_BASE, TIM27_BASE};
inline constexpr int tim_channels[tim_count] = {4, 4, 4, 4, 4, 0, 0, 4, 2, 1, 1, 2, 1, 1, 2, 1, 1, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0};
inline constexpr bool tim_32bit[tim_count] = {false, true, true, true, true};
inline constexpr bool tim_main_output[tim_count] = {true, false, false, false, false, false, false, true,
    false, false, false, false, false, false, true, true, true, false, false, true};

// checked against the shared register struct
namespace tim_offsets {
inline constexpr uint32_t DIER  = offsetof(TIM3_Type, TIM3_DIER);
inline constexpr uint32_t EGR   = offsetof(TIM3_Type, TIM3_EGR);
inline constexpr uint32_t CCMR1 = offsetof(TIM3_Type, TIM3_CCMR1);
inline constexpr uint32_t CCER  = offsetof(TIM3_Type, TIM3_CCER);
inline constexpr uint32_t CNT   = offsetof(TIM3_Type, TIM3_CNT);
inline constexpr uint32_t PSC   = offsetof(TIM3_Type, TIM3_PSC);
inline constexpr uint32_t ARR   = offsetof(TIM3_Type, TIM3_ARR);
inline constexpr uint32_t CCR1  = offsetof(TIM3_Type, TIM3_CCR1);
inline constexpr uint32_t CCR4  = offsetof(TIM3_Type, TIM3_CCR4);
inline constexpr uint32_t BDTR  = offsetof(TIM1_Type, TIM_BDTR);
}

// checked against the shared bit positions
namespace tim_bits {
inline constexpr uint32_t CR1_CEN        = TIM3_TIM3_CR1_CEN_Msk;
inline constexpr uint32_t CR1_ARPE       = TIM3_TIM3_CR1_ARPE_Msk;
inline constexpr uint32_t DIER_UIE       = TIM3_TIM3_DIER_UIE_Msk;
inline constexpr uint32_t DIER_UDE       = TIM3_TIM3_DIER_UDE_Msk;
inline constexpr uint32_t EGR_UG         = TIM3_TIM3_EGR_UG_Msk;
inline constexpr uint32_t CCMR1_OC1PE    = TIM3_TIM3_CCMR1_ALTERNATE1_OC1PE_Msk;
inline constexpr uint32_t CCMR1_OC1M_Pos = TIM3_TIM3_CCMR1_ALTERNATE1_OC1M_Pos;
inline constexpr uint32_t CCMR1_OC2M_Pos = TIM3_TIM3_CCMR1_ALTERNATE1_OC2M_Pos;
inline constexpr uint32_t CCER_CC1E      = TIM3_TIM3_CCER_CC1E_Msk;
inline constexpr uint32_t CCER_CC1P      = TIM3_TIM3_CCER_CC1P_Msk;
inline constexpr uint32_t CCER_CC2E      = TIM3_TIM3_CCER_CC2E_Msk;
inline constexpr uint32_t BDTR_MOE       = TIM1_TIM_BDTR_MOE_Msk;
}
