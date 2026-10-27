#pragma once
// Device layer for the shared hrtim_init.h. The G564 HRTIM is the G474 one
// with a wider register map (0x100 per timer, timer A at 0x100, common block
// at 0x700) and a few extra registers; the SVDConv header flattens it, so
// master/timer/common structs with the stm32g4 member names are laid over it.
#include <cstddef>
#include "st_device.h"

struct HRTIMMasterRegs {
    volatile uint32_t MCR, MISR, MICR, MDIER, MCNTR, MPER, MREP, MCMP1R, RESERVED0, MCMP2R, MCMP3R, MCMP4R;
};

struct HRTIMTimerRegs {
    volatile uint32_t TIMxCR, TIMxISR, TIMxICR, TIMxDIER, CNTxR, PERxR, REPxR,
        CMP1xR, CMP1CxR, CMP2xR, CMP3xR, CMP4xR, CPT1xR, CPT2xR, DTxR,
        SETx1R, RSTx1R, SETx2R, RSTx2R, EEFxR1, EEFxR2, RSTxR, CHPxR,
        CPT1xCR, CPT2xCR, OUTxR, FLTxR, TIMxCR2, EEFxR3,
        EESELxR, COTRGxR, COFExR, CODExR;
};

struct HRTIMCommonRegs {
    volatile uint32_t CR1, CR2, ISR, ICR, IER, OENR, ODISR, ODSR, BMCR, BMTRGR,
        BMCMPR, BMPER, EECR1, EECR2, EECR3, ADC1R, ADC2R, ADC3R, ADC4R, DLLCR,
        FLTINR1, FLTINR2, BDMUPR, BDTAUPR, BDTBUPR, BDTCUPR, BDTDUPR, BDTEUPR,
        BDMADR, BDTFUPR, ADCER, ADCUR, ADCPS1, ADCPS2, FLTINR3, FLTINR4, FLTINR5;
};

constexpr uint32_t hrtim_timer_offset = offsetof(HRTIM1_Type, HRTIM_TIMACR);
constexpr uint32_t hrtim_timer_stride = offsetof(HRTIM1_Type, HRTIM_TIMBCR) - hrtim_timer_offset;
constexpr uint32_t hrtim_common_offset = offsetof(HRTIM1_Type, HRTIM_CR1);

// Tie the overlays to the header
static_assert(offsetof(HRTIM1_Type, HRTIM_MCR) == 0);
static_assert(offsetof(HRTIM1_Type, HRTIM_MCMP4R) == offsetof(HRTIMMasterRegs, MCMP4R));
static_assert(offsetof(HRTIM1_Type, HRTIM_PERAR)  - hrtim_timer_offset == offsetof(HRTIMTimerRegs, PERxR));
static_assert(offsetof(HRTIM1_Type, HRTIM_CMP4AR) - hrtim_timer_offset == offsetof(HRTIMTimerRegs, CMP4xR));
static_assert(offsetof(HRTIM1_Type, HRTIM_DTAR)   - hrtim_timer_offset == offsetof(HRTIMTimerRegs, DTxR));
static_assert(offsetof(HRTIM1_Type, HRTIM_RSTA2R) - hrtim_timer_offset == offsetof(HRTIMTimerRegs, RSTx2R));
static_assert(offsetof(HRTIM1_Type, HRTIM_OUTAR)  - hrtim_timer_offset == offsetof(HRTIMTimerRegs, OUTxR));
static_assert(offsetof(HRTIM1_Type, HRTIM_TIMACR2) - hrtim_timer_offset == offsetof(HRTIMTimerRegs, TIMxCR2));
static_assert(offsetof(HRTIM1_Type, HRTIM_CODEAR) - hrtim_timer_offset == offsetof(HRTIMTimerRegs, CODExR));
static_assert(offsetof(HRTIM1_Type, HRTIM_TIMFCR) - hrtim_timer_offset == 5 * hrtim_timer_stride);
static_assert(offsetof(HRTIM1_Type, HRTIM_OENR)   - hrtim_common_offset == offsetof(HRTIMCommonRegs, OENR));
static_assert(offsetof(HRTIM1_Type, HRTIM_ADC3R)  - hrtim_common_offset == offsetof(HRTIMCommonRegs, ADC3R));
static_assert(offsetof(HRTIM1_Type, HRTIM_DLLCR)  - hrtim_common_offset == offsetof(HRTIMCommonRegs, DLLCR));
static_assert(offsetof(HRTIM1_Type, HRTIM_ADCPS1) - hrtim_common_offset == offsetof(HRTIMCommonRegs, ADCPS1));
static_assert(offsetof(HRTIM1_Type, HRTIM_FLTINR5) - hrtim_common_offset == offsetof(HRTIMCommonRegs, FLTINR5));

inline HRTIMMasterRegs& hrtim_master() {
    return *reinterpret_cast<HRTIMMasterRegs*>(HRTIM1_BASE);
}
inline HRTIMTimerRegs& hrtim_timer(int ch) {
    return *reinterpret_cast<HRTIMTimerRegs*>(HRTIM1_BASE + hrtim_timer_offset + ch * hrtim_timer_stride);
}
inline HRTIMCommonRegs& hrtim_common() {
    return *reinterpret_cast<HRTIMCommonRegs*>(HRTIM1_BASE + hrtim_common_offset);
}

constexpr float hrtim_clock_hz = CPU_FREQUENCY_HZ;

// Bit layout is the same for timers A-F; timer A's macros are used.
namespace hrtim_bits {
constexpr uint32_t TIMCR_CKPSC_Pos  = HRTIM1_HRTIM_TIMACR_CKPSCA_Pos;
constexpr uint32_t TIMCR_CONT       = HRTIM1_HRTIM_TIMACR_CONT_Msk;
constexpr uint32_t TIMCR_TRSTU      = HRTIM1_HRTIM_TIMACR_TARSTU_Msk;
constexpr uint32_t TIMCR_PREEN      = HRTIM1_HRTIM_TIMACR_PREEN_Msk;
constexpr uint32_t TIMCR2_UDM       = HRTIM1_HRTIM_TIMACR2_UDM_Msk;
constexpr uint32_t MCR_TACEN_Pos    = HRTIM1_HRTIM_MCR_TACEN_Pos;
constexpr uint32_t CR1_AD1USRC_Pos  = HRTIM1_HRTIM_CR1_AD1USRC_Pos;
constexpr uint32_t CR1_AD3USRC_Pos  = HRTIM1_HRTIM_CR1_AD3USRC_Pos;
constexpr uint32_t DLLCR_CALEN      = HRTIM1_HRTIM_DLLCR_CALEN_Msk;
constexpr uint32_t DLLCR_CALRTE_Pos = HRTIM1_HRTIM_DLLCR_CALRTE_Pos;
// checked against the shared enums
constexpr uint32_t SET1R_SST        = HRTIM1_HRTIM_SETA1R_SST_Msk;
constexpr uint32_t SET1R_PER        = HRTIM1_HRTIM_SETA1R_PER_Msk;
constexpr uint32_t SET1R_CMP1       = HRTIM1_HRTIM_SETA1R_CMP1_Msk;
constexpr uint32_t SET1R_CMP4       = HRTIM1_HRTIM_SETA1R_CMP4_Msk;
constexpr uint32_t SET1R_MSTCMP4    = HRTIM1_HRTIM_SETA1R_MSTCMP4_Msk;
constexpr uint32_t ADC1R_AD1TAC3    = HRTIM1_HRTIM_ADC1R_ADC1TAC3_Msk;
constexpr uint32_t ADC1R_AD1TFC3    = HRTIM1_HRTIM_ADC1R_ADC1TFC3_Msk;
constexpr uint32_t ADC1R_AD1TFC4    = HRTIM1_HRTIM_ADC1R_ADC1TFC4_Msk;
constexpr uint32_t ADC1R_AD1TEPER   = HRTIM1_HRTIM_ADC1R_ADC1TEPER_Msk;
}
