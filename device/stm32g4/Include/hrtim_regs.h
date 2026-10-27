#pragma once
// Device layer for the shared hrtim_init.h: register structs, accessors and
// the bit definitions it uses.
#include "st_device.h"

using HRTIMMasterRegs = HRTIM_Master_TypeDef;  // MCR, MPER, ...
using HRTIMTimerRegs = HRTIM_Timerx_TypeDef;   // TIMxCR, PERxR, CMP1xR, ...
using HRTIMCommonRegs = HRTIM_Common_TypeDef;  // OENR, ADC1R, DLLCR, ...

inline HRTIMMasterRegs& hrtim_master() { return HRTIM1->sMasterRegs; }
inline HRTIMTimerRegs& hrtim_timer(int ch) { return HRTIM1->sTimerxRegs[ch]; }
inline HRTIMCommonRegs& hrtim_common() { return HRTIM1->sCommonRegs; }

constexpr float hrtim_clock_hz = CPU_FREQUENCY_HZ;

namespace hrtim_bits {
constexpr uint32_t TIMCR_CKPSC_Pos  = HRTIM_TIMCR_CK_PSC_Pos;
constexpr uint32_t TIMCR_CONT       = HRTIM_TIMCR_CONT;
constexpr uint32_t TIMCR_TRSTU      = HRTIM_TIMCR_TRSTU;
constexpr uint32_t TIMCR_PREEN      = HRTIM_TIMCR_PREEN;
constexpr uint32_t TIMCR2_UDM       = HRTIM_TIMCR2_UDM;
constexpr uint32_t MCR_TACEN_Pos    = HRTIM_MCR_TACEN_Pos;
constexpr uint32_t DLLCR_CALEN      = HRTIM_DLLCR_CALEN;
constexpr uint32_t DLLCR_CALRTE_Pos = HRTIM_DLLCR_CALRTE_Pos;
// checked against the shared enums
constexpr uint32_t SET1R_SST        = HRTIM_SET1R_SST;
constexpr uint32_t SET1R_PER        = HRTIM_SET1R_PER;
constexpr uint32_t SET1R_CMP1       = HRTIM_SET1R_CMP1;
constexpr uint32_t SET1R_CMP4       = HRTIM_SET1R_CMP4;
constexpr uint32_t SET1R_MSTCMP4    = HRTIM_SET1R_MSTCMP4;
constexpr uint32_t ADC1R_AD1TAC3    = HRTIM_ADC1R_AD1TAC3;
constexpr uint32_t ADC1R_AD1TFC3    = HRTIM_ADC1R_AD1TFC3;
constexpr uint32_t ADC1R_AD1TFC4    = HRTIM_ADC1R_AD1TFC4;
constexpr uint32_t ADC1R_AD1TEPER   = HRTIM_ADC1R_AD1TEPER;
}
