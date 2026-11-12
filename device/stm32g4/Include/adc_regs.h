#pragma once
// Device layer for the shared adc_init module: instance addresses and the
// encodings that are specific to the stm32g4 ADC.
//
// Internal channels: OPAMP1 ADC1 IN13, OPAMP2 ADC2 IN16, OPAMP3 ADC2 IN18 and
// ADC3 IN13, OPAMP4 ADC5 IN5, OPAMP5 ADC5 IN3, OPAMP6 ADC4 IN17.
#include <cstddef>
#include "st_device.h"

inline constexpr int adc_count = 5;
inline constexpr uint32_t adc_base[] = {ADC1_BASE, ADC2_BASE, ADC3_BASE, ADC4_BASE, ADC5_BASE};
inline constexpr int adc_group[] = {0, 0, 1, 1, 1};      // converters sharing a common register: ADC1/2, ADC3/4/5
inline constexpr uint32_t adc_group_base[] = {ADC12_COMMON_BASE, ADC345_COMMON_BASE};
inline constexpr int adc_channel_count = 19;             // IN0..IN18
inline constexpr bool adc_channel_preselect = false;
inline constexpr uint32_t adc_regulator_startup_us = 20;   // tADCVREG_STUP

// CCR CKMODE: the clock of an ADC group, 60 MHz max. KERNEL is adc_ker_ck
// from the RCC (CCIPR ADCxSEL), which is off at reset and not set up by
// rcc_init; the others are synchronous to the AHB clock.
enum class ADCClock : uint8_t {
    KERNEL,
    HCLK,        // only with an AHB prescaler of 1
    HCLK_DIV2,
    HCLK_DIV4,
};
inline consteval uint32_t adc_ccr_clock(ADCClock clock) {
    return static_cast<uint32_t>(clock) << ADC_CCR_CKMODE_Pos;
}

// SMPRx, sampling time in ADC clocks; a 12 bit conversion adds 12.5
enum class ADCSampleTime : uint8_t {
    CYCLES_2_5,
    CYCLES_6_5,
    CYCLES_12_5,
    CYCLES_24_5,
    CYCLES_47_5,
    CYCLES_92_5,
    CYCLES_247_5,
    CYCLES_640_5,
};

// CFGR EXTSEL, regular group. Unprefixed sources have the same number on
// every instance, the others only exist on ADC1/2 or on ADC3/4/5.
enum class ADCTrigger : uint8_t {
    TIM1_CH3      = 2,
    TIM3_TRGO     = 4,
    TIM8_TRGO     = 7,
    TIM8_TRGO2    = 8,
    TIM1_TRGO     = 9,
    TIM1_TRGO2    = 10,
    TIM2_TRGO     = 11,
    TIM4_TRGO     = 12,
    TIM6_TRGO     = 13,
    TIM15_TRGO    = 14,
    TIM20_TRGO    = 16,
    TIM20_TRGO2   = 17,
    TIM20_CH1     = 18,
    HRTIM_TRG1    = 21,
    HRTIM_TRG3    = 22,
    HRTIM_TRG5    = 23,
    HRTIM_TRG6    = 24,
    HRTIM_TRG7    = 25,
    HRTIM_TRG8    = 26,
    HRTIM_TRG9    = 27,
    HRTIM_TRG10   = 28,
    LPTIM_OUT     = 29,
    TIM7_TRGO     = 30,

    ADC12_TIM1_CH1   = 0,
    ADC12_TIM1_CH2   = 1,
    ADC12_TIM2_CH2   = 3,
    ADC12_TIM4_CH4   = 5,
    ADC12_EXTI11     = 6,
    ADC12_TIM3_CH4   = 15,
    ADC12_TIM20_CH2  = 19,
    ADC12_TIM20_CH3  = 20,

    ADC345_TIM3_CH1   = 0,
    ADC345_TIM2_CH3   = 1,
    ADC345_TIM8_CH1   = 3,
    ADC345_EXTI2      = 5,
    ADC345_TIM4_CH1   = 6,
    ADC345_TIM2_CH1   = 15,
    ADC345_HRTIM_TRG2 = 19,
    ADC345_HRTIM_TRG4 = 20,
};

// JSQR JEXTSEL, injected group
enum class ADCInjectedTrigger : uint8_t {
    TIM1_TRGO     = 0,
    TIM1_CH4      = 1,
    TIM2_TRGO     = 2,
    TIM4_TRGO     = 5,
    TIM8_CH4      = 7,
    TIM1_TRGO2    = 8,
    TIM8_TRGO     = 9,
    TIM8_TRGO2    = 10,
    TIM3_TRGO     = 12,
    TIM6_TRGO     = 14,
    TIM15_TRGO    = 15,
    TIM20_TRGO    = 16,
    TIM20_TRGO2   = 17,
    HRTIM_TRG2    = 19,
    HRTIM_TRG4    = 20,
    HRTIM_TRG5    = 21,
    HRTIM_TRG6    = 22,
    HRTIM_TRG7    = 23,
    HRTIM_TRG8    = 24,
    HRTIM_TRG9    = 25,
    HRTIM_TRG10   = 26,
    LPTIM_OUT     = 29,
    TIM7_TRGO     = 30,

    ADC12_TIM2_CH1   = 3,
    ADC12_TIM3_CH4   = 4,
    ADC12_EXTI15     = 6,
    ADC12_TIM3_CH3   = 11,
    ADC12_TIM3_CH1   = 13,
    ADC12_TIM20_CH4  = 18,
    ADC12_TIM16_CH1  = 27,

    ADC345_TIM8_CH2   = 3,
    ADC345_TIM4_CH3   = 4,
    ADC345_TIM4_CH4   = 6,
    ADC345_TIM1_CH3   = 11,
    ADC345_EXTI3      = 13,
    ADC345_TIM20_CH2  = 18,
    ADC345_HRTIM_TRG1 = 27,
    ADC345_HRTIM_TRG3 = 28,
};

// checked against the shared register layout
namespace adc_offsets {
inline constexpr uint32_t CFGR  = offsetof(ADC_TypeDef, CFGR);
inline constexpr uint32_t SMPR2 = offsetof(ADC_TypeDef, SMPR2);
inline constexpr uint32_t SQR1  = offsetof(ADC_TypeDef, SQR1);
inline constexpr uint32_t DR    = offsetof(ADC_TypeDef, DR);
inline constexpr uint32_t JSQR  = offsetof(ADC_TypeDef, JSQR);
inline constexpr uint32_t JDR1  = offsetof(ADC_TypeDef, JDR1);
inline constexpr uint32_t CCR   = offsetof(ADC_Common_TypeDef, CCR);
}

namespace adc_bits {
inline constexpr uint32_t CFGR_RES_Pos     = ADC_CFGR_RES_Pos;
// checked against the shared bit positions
inline constexpr uint32_t ISR_ADRDY        = ADC_ISR_ADRDY;
inline constexpr uint32_t CR_ADSTART       = ADC_CR_ADSTART;
inline constexpr uint32_t CR_JADSTART      = ADC_CR_JADSTART;
inline constexpr uint32_t CR_ADVREGEN      = ADC_CR_ADVREGEN;
inline constexpr uint32_t CR_ADCAL         = ADC_CR_ADCAL;
inline constexpr uint32_t CFGR_EXTSEL_Pos  = ADC_CFGR_EXTSEL_Pos;
inline constexpr uint32_t CFGR_EXTEN_Pos   = ADC_CFGR_EXTEN_Pos;
inline constexpr uint32_t CFGR_OVRMOD      = ADC_CFGR_OVRMOD;
inline constexpr uint32_t CFGR_CONT        = ADC_CFGR_CONT;
inline constexpr uint32_t CFGR_JQDIS       = ADC_CFGR_JQDIS;
inline constexpr uint32_t SMPR1_SMP1_Pos   = ADC_SMPR1_SMP1_Pos;
inline constexpr uint32_t SQR1_SQ1_Pos     = ADC_SQR1_SQ1_Pos;
inline constexpr uint32_t SQR1_SQ2_Pos     = ADC_SQR1_SQ2_Pos;
inline constexpr uint32_t JSQR_JEXTSEL_Pos = ADC_JSQR_JEXTSEL_Pos;
inline constexpr uint32_t JSQR_JEXTEN_Pos  = ADC_JSQR_JEXTEN_Pos;
inline constexpr uint32_t JSQR_JSQ1_Pos    = ADC_JSQR_JSQ1_Pos;
inline constexpr uint32_t JSQR_JSQ2_Pos    = ADC_JSQR_JSQ2_Pos;
inline constexpr uint32_t CCR_VREFEN       = ADC_CCR_VREFEN;
inline constexpr uint32_t CCR_TSEN         = ADC_CCR_VSENSESEL;
}
