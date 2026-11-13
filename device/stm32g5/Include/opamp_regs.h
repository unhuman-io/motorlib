#pragma once
// Device layer for the shared opamp_init module. Each opamp has its own RCC
// enable (RCCInit opamp1..3).
//
// Internal outputs: OPAMP1 ADC1 IN13 and ADC3 IN19, OPAMP2 ADC1 IN19 and
// ADC2 IN16, OPAMP3 ADC2 IN19 and ADC3 IN13.
#include "st_device.h"

inline constexpr int opamp_count = 3;
inline constexpr uint32_t opamp_base[] = {OPAMP1_BASE, OPAMP2_BASE, OPAMP3_BASE};
inline constexpr uint32_t opamp_max_gain = 3;   // PGA_GAIN low bits: x16
inline constexpr uint32_t opamp_trim_delay_us = 1000;   // settling time of a trim step during the calibration

// checked against the shared bit positions
namespace opamp_bits {
inline constexpr uint32_t CSR_OPAEN        = OPAMP1_OPAMP_CSR_OPAEN_Msk;
inline constexpr uint32_t CSR_VP_SEL_Pos   = OPAMP1_OPAMP_CSR_VP_SEL_Pos;
inline constexpr uint32_t CSR_VM_SEL_Pos   = OPAMP1_OPAMP_CSR_VM_SEL_Pos;
inline constexpr uint32_t CSR_OPAHSM       = OPAMP1_OPAMP_CSR_OPAHSM_Msk;
inline constexpr uint32_t CSR_OPAINTOEN    = OPAMP1_OPAMP_CSR_OPAINTOEN_Msk;
inline constexpr uint32_t CSR_PGA_GAIN_Pos = OPAMP1_OPAMP_CSR_PGA_GAIN_Pos;
inline constexpr uint32_t CSR_LOCK         = OPAMP1_OPAMP_CSR_LOCK_Msk;
inline constexpr uint32_t CSR_USERTRIM     = OPAMP1_OPAMP_CSR_USERTRIM_Msk;
inline constexpr uint32_t CSR_CALON        = OPAMP1_OPAMP_CSR_CALON_Msk;
inline constexpr uint32_t CSR_CALSEL_Pos   = OPAMP1_OPAMP_CSR_CALSEL_Pos;
inline constexpr uint32_t CSR_TRIMOFFSETP_Pos = OPAMP1_OPAMP_CSR_TRIMOFFSETP_Pos;
inline constexpr uint32_t CSR_TRIMOFFSETN_Pos = OPAMP1_OPAMP_CSR_TRIMOFFSETN_Pos;
inline constexpr uint32_t CSR_CALOUT       = OPAMP1_OPAMP_CSR_CALOUT_Msk;
}
