#pragma once
#include "st_device.h"

enum class RCCEnable : bool {
    DISABLED,
    ENABLED
};

// Members are listed MSB first within each register, matching the stm32g4
// RCCInit so designated initializers port between devices. Defaults follow
// the register reset values from the SVD (AHB1ENR = 0xDC000100, all other
// ENRs 0) except crs, which is kept on as on stm32g4.
struct RCCInit {
    // AHB1
    RCCEnable sram1     = RCCEnable::ENABLED;
    RCCEnable dcache    = RCCEnable::ENABLED;
    RCCEnable sram3     = RCCEnable::ENABLED;
    RCCEnable sram2     = RCCEnable::ENABLED;
    RCCEnable ramcfg    = RCCEnable::DISABLED;
    RCCEnable fmac1     = RCCEnable::DISABLED;
    RCCEnable cordic    = RCCEnable::DISABLED;
    RCCEnable crc       = RCCEnable::DISABLED;
    RCCEnable flitf     = RCCEnable::ENABLED;
    RCCEnable gpdma2    = RCCEnable::DISABLED;
    RCCEnable gpdma1    = RCCEnable::DISABLED;

    // AHB2
    RCCEnable adc3      = RCCEnable::DISABLED;
    RCCEnable adc12     = RCCEnable::DISABLED;
    RCCEnable fmac2     = RCCEnable::DISABLED;
    RCCEnable dac2      = RCCEnable::DISABLED;
    RCCEnable dac1      = RCCEnable::DISABLED;

    // AHB4
    RCCEnable xspi1     = RCCEnable::DISABLED;
    RCCEnable gpioh     = RCCEnable::DISABLED;
    RCCEnable gpiog     = RCCEnable::DISABLED;
    RCCEnable gpiof     = RCCEnable::DISABLED;
    RCCEnable gpioe     = RCCEnable::DISABLED;
    RCCEnable gpiod     = RCCEnable::DISABLED;
    RCCEnable gpioc     = RCCEnable::DISABLED;
    RCCEnable gpiob     = RCCEnable::DISABLED;
    RCCEnable gpioa     = RCCEnable::DISABLED;
    RCCEnable rng       = RCCEnable::DISABLED;

    // APB1L
    RCCEnable usart6    = RCCEnable::DISABLED;
    RCCEnable crs       = RCCEnable::ENABLED;
    RCCEnable i2c2      = RCCEnable::DISABLED;
    RCCEnable i2c1      = RCCEnable::DISABLED;
    RCCEnable uart5     = RCCEnable::DISABLED;
    RCCEnable uart4     = RCCEnable::DISABLED;
    RCCEnable usart3    = RCCEnable::DISABLED;
    RCCEnable usart2    = RCCEnable::DISABLED;
    RCCEnable spi3      = RCCEnable::DISABLED;
    RCCEnable spi2      = RCCEnable::DISABLED;
    RCCEnable wwdg1     = RCCEnable::DISABLED;
    RCCEnable tim7      = RCCEnable::DISABLED;
    RCCEnable tim6      = RCCEnable::DISABLED;
    RCCEnable tim3      = RCCEnable::DISABLED;
    RCCEnable tim2      = RCCEnable::DISABLED;

    // APB1H
    RCCEnable ucpd1     = RCCEnable::DISABLED;
    RCCEnable fdcan     = RCCEnable::DISABLED;

    // APB2
    RCCEnable tim5      = RCCEnable::DISABLED;
    RCCEnable tim4      = RCCEnable::DISABLED;
    RCCEnable usbfs     = RCCEnable::DISABLED;
    RCCEnable tim17     = RCCEnable::DISABLED;
    RCCEnable tim16     = RCCEnable::DISABLED;
    RCCEnable tim15     = RCCEnable::DISABLED;
    RCCEnable usart1    = RCCEnable::DISABLED;
    RCCEnable tim8      = RCCEnable::DISABLED;
    RCCEnable spi1      = RCCEnable::DISABLED;
    RCCEnable tim1      = RCCEnable::DISABLED;
    RCCEnable playapb   = RCCEnable::DISABLED;
    RCCEnable play      = RCCEnable::DISABLED;
    RCCEnable hrtim1    = RCCEnable::DISABLED;

    // APB3
    RCCEnable comp4     = RCCEnable::DISABLED;
    RCCEnable comp3     = RCCEnable::DISABLED;
    RCCEnable comp2     = RCCEnable::DISABLED;
    RCCEnable comp1     = RCCEnable::DISABLED;
    RCCEnable rtcapb    = RCCEnable::DISABLED;
    RCCEnable vref      = RCCEnable::DISABLED;
    RCCEnable opamp3    = RCCEnable::DISABLED;
    RCCEnable opamp2    = RCCEnable::DISABLED;
    RCCEnable opamp1    = RCCEnable::DISABLED;
    RCCEnable lptim1    = RCCEnable::DISABLED;
    RCCEnable i2c4      = RCCEnable::DISABLED;
    RCCEnable i2c3      = RCCEnable::DISABLED;
    RCCEnable lpuart1   = RCCEnable::DISABLED;
    RCCEnable tim20     = RCCEnable::DISABLED;
    RCCEnable sbs       = RCCEnable::DISABLED;
};

struct RCCClockInit {
    uint32_t cpu_frequency_hz = 64'000'000;
};

// Positions come from the svdconv --fields=macro output in STM32G564.h.
template<const RCCInit r>
inline consteval uint32_t get_ahb1enr() {
    return
        static_cast<uint32_t>(r.sram1) << RCC_RCC_AHB1ENR_SRAM1EN_Pos |
        static_cast<uint32_t>(r.dcache) << RCC_RCC_AHB1ENR_DCACHEEN_Pos |
        static_cast<uint32_t>(r.sram3) << RCC_RCC_AHB1ENR_SRAM3EN_Pos |
        static_cast<uint32_t>(r.sram2) << RCC_RCC_AHB1ENR_SRAM2EN_Pos |
        static_cast<uint32_t>(r.ramcfg) << RCC_RCC_AHB1ENR_RAMCFGEN_Pos |
        static_cast<uint32_t>(r.fmac1) << RCC_RCC_AHB1ENR_FMAC1EN_Pos |
        static_cast<uint32_t>(r.cordic) << RCC_RCC_AHB1ENR_CORDICEN_Pos |
        static_cast<uint32_t>(r.crc) << RCC_RCC_AHB1ENR_CRCEN_Pos |
        static_cast<uint32_t>(r.flitf) << RCC_RCC_AHB1ENR_FLITFEN_Pos |
        static_cast<uint32_t>(r.gpdma2) << RCC_RCC_AHB1ENR_GPDMA2EN_Pos |
        static_cast<uint32_t>(r.gpdma1) << RCC_RCC_AHB1ENR_GPDMA1EN_Pos;
}

template<const RCCInit r>
inline consteval uint32_t get_ahb2enr() {
    return
        static_cast<uint32_t>(r.adc3) << RCC_RCC_AHB2ENR_ADC3EN_Pos |
        static_cast<uint32_t>(r.adc12) << RCC_RCC_AHB2ENR_ADC12EN_Pos |
        static_cast<uint32_t>(r.fmac2) << RCC_RCC_AHB2ENR_FMAC2EN_Pos |
        static_cast<uint32_t>(r.dac2) << RCC_RCC_AHB2ENR_DAC2EN_Pos |
        static_cast<uint32_t>(r.dac1) << RCC_RCC_AHB2ENR_DAC1EN_Pos;
}

template<const RCCInit r>
inline consteval uint32_t get_ahb4enr() {
    return
        static_cast<uint32_t>(r.xspi1) << RCC_RCC_AHB4ENR_XSPI1EN_Pos |
        static_cast<uint32_t>(r.gpioh) << RCC_RCC_AHB4ENR_GPIOHEN_Pos |
        static_cast<uint32_t>(r.gpiog) << RCC_RCC_AHB4ENR_GPIOGEN_Pos |
        static_cast<uint32_t>(r.gpiof) << RCC_RCC_AHB4ENR_GPIOFEN_Pos |
        static_cast<uint32_t>(r.gpioe) << RCC_RCC_AHB4ENR_GPIOEEN_Pos |
        static_cast<uint32_t>(r.gpiod) << RCC_RCC_AHB4ENR_GPIODEN_Pos |
        static_cast<uint32_t>(r.gpioc) << RCC_RCC_AHB4ENR_GPIOCEN_Pos |
        static_cast<uint32_t>(r.gpiob) << RCC_RCC_AHB4ENR_GPIOBEN_Pos |
        static_cast<uint32_t>(r.gpioa) << RCC_RCC_AHB4ENR_GPIOAEN_Pos |
        static_cast<uint32_t>(r.rng) << RCC_RCC_AHB4ENR_RNGEN_Pos;
}

template<const RCCInit r>
inline consteval uint32_t get_apb1lenr() {
    return
        static_cast<uint32_t>(r.usart6) << RCC_RCC_APB1LENR_USART6EN_Pos |
        static_cast<uint32_t>(r.crs) << RCC_RCC_APB1LENR_CRSEN_Pos |
        static_cast<uint32_t>(r.i2c2) << RCC_RCC_APB1LENR_I2C2EN_Pos |
        static_cast<uint32_t>(r.i2c1) << RCC_RCC_APB1LENR_I2C1EN_Pos |
        static_cast<uint32_t>(r.uart5) << RCC_RCC_APB1LENR_UART5EN_Pos |
        static_cast<uint32_t>(r.uart4) << RCC_RCC_APB1LENR_UART4EN_Pos |
        static_cast<uint32_t>(r.usart3) << RCC_RCC_APB1LENR_USART3EN_Pos |
        static_cast<uint32_t>(r.usart2) << RCC_RCC_APB1LENR_USART2EN_Pos |
        static_cast<uint32_t>(r.spi3) << RCC_RCC_APB1LENR_SPI3EN_Pos |
        static_cast<uint32_t>(r.spi2) << RCC_RCC_APB1LENR_SPI2EN_Pos |
        static_cast<uint32_t>(r.wwdg1) << RCC_RCC_APB1LENR_WWDG1EN_Pos |
        static_cast<uint32_t>(r.tim7) << RCC_RCC_APB1LENR_TIM7EN_Pos |
        static_cast<uint32_t>(r.tim6) << RCC_RCC_APB1LENR_TIM6EN_Pos |
        static_cast<uint32_t>(r.tim3) << RCC_RCC_APB1LENR_TIM3EN_Pos |
        static_cast<uint32_t>(r.tim2) << RCC_RCC_APB1LENR_TIM2EN_Pos;
}

template<const RCCInit r>
inline consteval uint32_t get_apb1henr() {
    return
        static_cast<uint32_t>(r.ucpd1) << RCC_RCC_APB1HENR_UCPD1EN_Pos |
        static_cast<uint32_t>(r.fdcan) << RCC_RCC_APB1HENR_FDCANEN_Pos;
}

template<const RCCInit r>
inline consteval uint32_t get_apb2enr() {
    return
        static_cast<uint32_t>(r.tim5) << RCC_RCC_APB2ENR_TIM5EN_Pos |
        static_cast<uint32_t>(r.tim4) << RCC_RCC_APB2ENR_TIM4EN_Pos |
        static_cast<uint32_t>(r.usbfs) << RCC_RCC_APB2ENR_USBFSEN_Pos |
        static_cast<uint32_t>(r.tim17) << RCC_RCC_APB2ENR_TIM17EN_Pos |
        static_cast<uint32_t>(r.tim16) << RCC_RCC_APB2ENR_TIM16EN_Pos |
        static_cast<uint32_t>(r.tim15) << RCC_RCC_APB2ENR_TIM15EN_Pos |
        static_cast<uint32_t>(r.usart1) << RCC_RCC_APB2ENR_USART1EN_Pos |
        static_cast<uint32_t>(r.tim8) << RCC_RCC_APB2ENR_TIM8EN_Pos |
        static_cast<uint32_t>(r.spi1) << RCC_RCC_APB2ENR_SPI1EN_Pos |
        static_cast<uint32_t>(r.tim1) << RCC_RCC_APB2ENR_TIM1EN_Pos |
        static_cast<uint32_t>(r.playapb) << RCC_RCC_APB2ENR_PLAYAPBEN_Pos |
        static_cast<uint32_t>(r.play) << RCC_RCC_APB2ENR_PLAYEN_Pos |
        static_cast<uint32_t>(r.hrtim1) << RCC_RCC_APB2ENR_HRTIM1EN_Pos;
}

template<const RCCInit r>
inline consteval uint32_t get_apb3enr() {
    return
        static_cast<uint32_t>(r.comp4) << RCC_RCC_APB3ENR_COMP4EN_Pos |
        static_cast<uint32_t>(r.comp3) << RCC_RCC_APB3ENR_COMP3EN_Pos |
        static_cast<uint32_t>(r.comp2) << RCC_RCC_APB3ENR_COMP2EN_Pos |
        static_cast<uint32_t>(r.comp1) << RCC_RCC_APB3ENR_COMP1EN_Pos |
        static_cast<uint32_t>(r.rtcapb) << RCC_RCC_APB3ENR_RTCAPBEN_Pos |
        static_cast<uint32_t>(r.vref) << RCC_RCC_APB3ENR_VREFEN_Pos |
        static_cast<uint32_t>(r.opamp3) << RCC_RCC_APB3ENR_OPAMP3EN_Pos |
        static_cast<uint32_t>(r.opamp2) << RCC_RCC_APB3ENR_OPAMP2EN_Pos |
        static_cast<uint32_t>(r.opamp1) << RCC_RCC_APB3ENR_OPAMP1EN_Pos |
        static_cast<uint32_t>(r.lptim1) << RCC_RCC_APB3ENR_LPTIM1EN_Pos |
        static_cast<uint32_t>(r.i2c4) << RCC_RCC_APB3ENR_I2C4EN_Pos |
        static_cast<uint32_t>(r.i2c3) << RCC_RCC_APB3ENR_I2C3EN_Pos |
        static_cast<uint32_t>(r.lpuart1) << RCC_RCC_APB3ENR_LPUART1EN_Pos |
        static_cast<uint32_t>(r.tim20) << RCC_RCC_APB3ENR_TIM20EN_Pos |
        static_cast<uint32_t>(r.sbs) << RCC_RCC_APB3ENR_SBSEN_Pos;
}

template<const RCCInit r>
inline void init_rcc() {
    if constexpr (get_ahb1enr<r>() != get_ahb1enr<RCCInit{}>()) {
        RCC->RCC_AHB1ENR = get_ahb1enr<r>();
    }
    if constexpr (get_ahb2enr<r>() != get_ahb2enr<RCCInit{}>()) {
        RCC->RCC_AHB2ENR = get_ahb2enr<r>();
    }
    if constexpr (get_ahb4enr<r>() != get_ahb4enr<RCCInit{}>()) {
        RCC->RCC_AHB4ENR = get_ahb4enr<r>();
    }
    if constexpr (get_apb1lenr<r>() != get_apb1lenr<RCCInit{}>()) {
        RCC->RCC_APB1LENR = get_apb1lenr<r>();
    }
    if constexpr (get_apb1henr<r>() != get_apb1henr<RCCInit{}>()) {
        RCC->RCC_APB1HENR = get_apb1henr<r>();
    }
    if constexpr (get_apb2enr<r>() != get_apb2enr<RCCInit{}>()) {
        RCC->RCC_APB2ENR = get_apb2enr<r>();
    }
    if constexpr (get_apb3enr<r>() != get_apb3enr<RCCInit{}>()) {
        RCC->RCC_APB3ENR = get_apb3enr<r>();
    }
}

template<const RCCClockInit c>
inline void init_rcc_clock() {
    static_assert(c.cpu_frequency_hz == 64'000'000, "only the 64 MHz HSI reset clock is supported so far");

    // ensure cpu clock is started for us_delay
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // TODO: PLL1 from HSE, flash latency, VOS
}

template<RCCInit rcc_init>
inline void device_rcc_config() {
    init_rcc<rcc_init>();
}

template<RCCClockInit rcc_clock_init>
inline void device_rcc_clock_config() {
    init_rcc_clock<rcc_clock_init>();
}
