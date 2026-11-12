#pragma once
// Device layer for the shared opamp_init module. The opamps are clocked with
// the SYSCFG (RCCInit syscfg).
//
//          VINP0  VINP1  VINP2  VINP3      VINM0  VINM1  VOUT   internal output
// OPAMP1   PA1    PA3    PA7    DAC3 CH1   PA3    PC5    PA2    ADC1 IN13
// OPAMP2   PA7    PB14   PB0    PD14       PA5    PC5    PA6    ADC2 IN16
// OPAMP3   PB0    PB13   PA1    DAC3 CH2   PB2    PB10   PB1    ADC2 IN18, ADC3 IN13
// OPAMP4   PB13   PD11   PB11   DAC4 CH1   PB10   PD8    PB12   ADC5 IN5
// OPAMP5   PB14   PD12   PC3    DAC4 CH2   PB15   PA3    PA8    ADC5 IN3
// OPAMP6   PB12   PD9    PB13   DAC3 CH1   PA1    PB1    PB11   ADC4 IN17
#include "st_device.h"

inline constexpr int opamp_count = 6;
inline constexpr uint32_t opamp_base[] = {OPAMP1_BASE, OPAMP2_BASE, OPAMP3_BASE, OPAMP4_BASE, OPAMP5_BASE, OPAMP6_BASE};
inline constexpr uint32_t opamp_max_gain = 5;   // PGA_GAIN low bits: x64

// checked against the shared bit positions
namespace opamp_bits {
inline constexpr uint32_t CSR_OPAEN        = OPAMP_CSR_OPAMPxEN;
inline constexpr uint32_t CSR_VP_SEL_Pos   = OPAMP_CSR_VPSEL_Pos;
inline constexpr uint32_t CSR_VM_SEL_Pos   = OPAMP_CSR_VMSEL_Pos;
inline constexpr uint32_t CSR_OPAHSM       = OPAMP_CSR_HIGHSPEEDEN;
inline constexpr uint32_t CSR_OPAINTOEN    = OPAMP_CSR_OPAMPINTEN;
inline constexpr uint32_t CSR_PGA_GAIN_Pos = OPAMP_CSR_PGGAIN_Pos;
inline constexpr uint32_t CSR_LOCK         = OPAMP_CSR_LOCK;
}
