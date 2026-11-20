#pragma once
// Device layer for the shared dac_init module. Each DAC has its own RCC
// enable (RCCInit dac1..dac4).
//
//        channel 1                     channel 2
// DAC1   PA4                           PA5
// DAC2   PA6                           -
// DAC3   OPAMP1 VINP3, OPAMP6 VINP3    OPAMP3 VINP3
// DAC4   OPAMP4 VINP3                  OPAMP5 VINP3
//
// DAC3 and DAC4 have no pins and no buffer. All channels also reach the
// comparators.
#include <cstddef>
#include "st_device.h"

inline constexpr int dac_count = 4;
inline constexpr uint32_t dac_base[] = {DAC1_BASE, DAC2_BASE, DAC3_BASE, DAC4_BASE};
inline constexpr int dac_channels[] = {2, 1, 2, 2};
inline constexpr bool dac_has_pins[] = {true, true, false, false};
inline constexpr float dac_clock_hz = CPU_FREQUENCY_HZ;   // AHB clock

// checked against the shared register struct
namespace dac_offsets {
inline constexpr uint32_t CR      = offsetof(DAC_TypeDef, CR);
inline constexpr uint32_t DHR12R1 = offsetof(DAC_TypeDef, DHR12R1);
inline constexpr uint32_t DHR12R2 = offsetof(DAC_TypeDef, DHR12R2);
inline constexpr uint32_t DOR1    = offsetof(DAC_TypeDef, DOR1);
inline constexpr uint32_t MCR     = offsetof(DAC_TypeDef, MCR);
}

namespace dac_bits {
inline constexpr uint32_t MCR_HFSEL_Pos = DAC_MCR_HFSEL_Pos;
// checked against the shared bit positions
inline constexpr uint32_t CR_EN1        = DAC_CR_EN1;
inline constexpr uint32_t MCR_MODE1_Pos = DAC_MCR_MODE1_Pos;
inline constexpr uint32_t CHANNEL2_Pos  = DAC_CR_EN2_Pos;
}
static_assert(DAC_MCR_MODE2_Pos - DAC_MCR_MODE1_Pos == DAC_CR_EN2_Pos);
