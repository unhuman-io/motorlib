#pragma once
// Device layer for the shared dac_init module. The DACs are DAC1, DAC2 and
// DAC5..DAC10, with one channel each. Each has its own RCC enable (RCCInit
// dac1..dac10) and a kernel clock selection (RCC CCIPR2 DACxSEL) that is the
// bus clock from reset.
//
// DAC5 is OPAMP1 VINP3 and DAC6 is OPAMP3 VINP3. Which DACs have an output
// pin is not known yet, so the output mode is not checked.
#include <cstddef>
#include "st_device.h"

inline constexpr int dac_count = 10;
inline constexpr uint32_t dac_base[] = {DAC1_BASE, DAC2_BASE, 0, 0, DAC5_BASE, DAC6_BASE, DAC7_BASE, DAC8_BASE, DAC9_BASE, DAC10_BASE};
inline constexpr int dac_channels[] = {1, 1, 0, 0, 1, 1, 1, 1, 1, 1};
inline constexpr bool dac_has_pins[] = {true, true, true, true, true, true, true, true, true, true};
inline constexpr float dac_clock_hz = CPU_FREQUENCY_HZ;   // bus clock

// checked against the shared register struct
namespace dac_offsets {
inline constexpr uint32_t CR      = offsetof(DAC1_Type, DAC_CR);
inline constexpr uint32_t DHR12R1 = offsetof(DAC1_Type, DAC_DHR12R1);
inline constexpr uint32_t DHR12R2 = 0x14;   // no channel 2: reserved
inline constexpr uint32_t DOR1    = offsetof(DAC1_Type, DAC_DOR1);
inline constexpr uint32_t MCR     = offsetof(DAC1_Type, DAC_MCR);
}

namespace dac_bits {
inline constexpr uint32_t MCR_HFSEL_Pos = DAC1_DAC_MCR_HFSEL_Pos;
// checked against the shared bit positions
inline constexpr uint32_t CR_EN1        = DAC1_DAC_CR_EN1_Msk;
inline constexpr uint32_t MCR_MODE1_Pos = DAC1_DAC_MCR_MODE1_Pos;
inline constexpr uint32_t CHANNEL2_Pos  = 16;   // no channel 2
}
