#pragma once
// Device layer for the shared adc_init module: instance addresses and the
// encodings that are specific to the stm32g5 ADC. The ADC clock comes from
// the RCC (RCCClockInit adc_clock, adc_clock_div), there is no selection in
// the ADC itself.
//
// Internal channels: OPAMP1 ADC1 IN13 and ADC3 IN19, OPAMP2 ADC1 IN19 and
// ADC2 IN16, OPAMP3 ADC2 IN19 and ADC3 IN13.
#include <cstddef>
#include "st_device.h"

inline constexpr int adc_count = 3;
inline constexpr uint32_t adc_base[] = {ADC1_BASE, ADC2_BASE, ADC3_BASE};
inline constexpr int adc_group[] = {0, 0, 1};            // converters sharing a common register: ADC1/2, ADC3
inline constexpr uint32_t adc_group_base[] = {ADCC12_BASE, ADCC3_BASE};
inline constexpr int adc_channel_count = 20;             // IN0..IN19
inline constexpr bool adc_channel_preselect = true;      // PCSEL: converted channels have to be selected
inline constexpr uint32_t adc_regulator_startup_us = 10;

enum class ADCClock : uint8_t {
    KERNEL,      // adc_ker_ck from the RCC
};
inline consteval uint32_t adc_ccr_clock(ADCClock) {
    return 0;
}

// SMPRx, sampling time in ADC clocks
enum class ADCSampleTime : uint8_t {
    CYCLES_2_5,
    CYCLES_4_5,
    CYCLES_7_5,
    CYCLES_12_5,
    CYCLES_24_5,
    CYCLES_47_5,
    CYCLES_256_5,
    CYCLES_848_5,
};

// CFGR1 EXTSEL, regular group: the sources that have the same number on all
// three instances. The timer capture/compare and remaining TRGO sources
// differ between ADC1/2 and ADC3 and some are switched by ADC_OR option bits.
enum class ADCTrigger : uint8_t {
    TIM6_TRGO     = 10,
    TIM20_TRGO    = 17,
    TIM20_TRGO2   = 18,
    TIM26_TRGO    = 19,
    LPTIM1_CH1    = 20,
    HRTIM_TRG1    = 22,
    HRTIM_TRG2    = 23,
    HRTIM_TRG3    = 24,
    HRTIM_TRG4    = 25,
    HRTIM_TRG5    = 26,
    HRTIM_TRG6    = 27,
    HRTIM_TRG7    = 28,
    HRTIM_TRG8    = 29,
    HRTIM_TRG9    = 30,
    HRTIM_TRG10   = 31,
};

// JSQR JEXTSEL, injected group, as above
enum class ADCInjectedTrigger : uint8_t {
    TIM2_TRGO     = 5,
    TIM20_TRGO    = 17,
    TIM20_TRGO2   = 18,
    TIM27_TRGO    = 19,
    LPTIM1_CH2    = 20,
    HRTIM_TRG1    = 22,
    HRTIM_TRG2    = 23,
    HRTIM_TRG3    = 24,
    HRTIM_TRG4    = 25,
    HRTIM_TRG5    = 26,
    HRTIM_TRG6    = 27,
    HRTIM_TRG7    = 28,
    HRTIM_TRG8    = 29,
    HRTIM_TRG9    = 30,
    HRTIM_TRG10   = 31,
};

// checked against the shared register layout
namespace adc_offsets {
inline constexpr uint32_t CFGR  = offsetof(ADC1_Type, ADC_CFGR1);
inline constexpr uint32_t SMPR2 = offsetof(ADC1_Type, ADC_SMPR2);
inline constexpr uint32_t SQR1  = offsetof(ADC1_Type, ADC_SQR1);
inline constexpr uint32_t DR    = offsetof(ADC1_Type, ADC_DR);
inline constexpr uint32_t JSQR  = offsetof(ADC1_Type, ADC_JSQR);
inline constexpr uint32_t JDR1  = offsetof(ADC1_Type, ADC_JDR1);
inline constexpr uint32_t CCR   = offsetof(ADCC12_Type, ADCC_CCR);
}
static_assert(offsetof(ADC1_Type, ADC_PCSEL) == 0x1C);

namespace adc_bits {
inline constexpr uint32_t CFGR_RES_Pos     = ADC1_ADC_CFGR1_RES_Pos;
// checked against the shared bit positions
inline constexpr uint32_t ISR_ADRDY        = ADC1_ADC_ISR_ADRDY_Msk;
inline constexpr uint32_t IER_EOCIE        = ADC1_ADC_IER_EOCIE_Msk;
inline constexpr uint32_t CR_ADSTART       = ADC1_ADC_CR_ADSTART_Msk;
inline constexpr uint32_t CR_JADSTART      = ADC1_ADC_CR_JADSTART_Msk;
inline constexpr uint32_t CR_ADVREGEN      = ADC1_ADC_CR_ADVREGEN_Msk;
inline constexpr uint32_t CR_ADCAL         = ADC1_ADC_CR_ADCAL_Msk;
inline constexpr uint32_t CR_ADDIS         = ADC1_ADC_CR_ADDIS_Msk;
inline constexpr uint32_t CR_ADSTP         = ADC1_ADC_CR_ADSTP_Msk;
inline constexpr uint32_t CR_JADSTP        = ADC1_ADC_CR_JADSTP_Msk;
inline constexpr uint32_t CFGR_EXTSEL_Pos  = ADC1_ADC_CFGR1_EXTSEL_Pos;
inline constexpr uint32_t CFGR_EXTEN_Pos   = ADC1_ADC_CFGR1_EXTEN_Pos;
inline constexpr uint32_t CFGR_OVRMOD      = ADC1_ADC_CFGR1_OVRMOD_Msk;
inline constexpr uint32_t CFGR_CONT        = ADC1_ADC_CFGR1_CONT_Msk;
inline constexpr uint32_t CFGR_JQDIS       = ADC1_ADC_CFGR1_JQDIS_Msk;
inline constexpr uint32_t SMPR1_SMP1_Pos   = ADC1_ADC_SMPR1_SMP1_Pos;
inline constexpr uint32_t SQR1_SQ1_Pos     = ADC1_ADC_SQR1_SQ1_Pos;
inline constexpr uint32_t SQR1_SQ2_Pos     = ADC1_ADC_SQR1_SQ2_Pos;
inline constexpr uint32_t JSQR_JEXTSEL_Pos = ADC1_ADC_JSQR_JEXTSEL_Pos;
inline constexpr uint32_t JSQR_JEXTEN_Pos  = ADC1_ADC_JSQR_JEXTEN_Pos;
inline constexpr uint32_t JSQR_JSQ1_Pos    = ADC1_ADC_JSQR_JSQ1_Pos;
inline constexpr uint32_t JSQR_JSQ2_Pos    = ADC1_ADC_JSQR_JSQ2_Pos;
inline constexpr uint32_t CCR_VREFEN       = ADCC12_ADCC_CCR_VREFEN_Msk;
inline constexpr uint32_t CCR_TSEN         = ADCC12_ADCC_CCR_TSEN_Msk;
}
