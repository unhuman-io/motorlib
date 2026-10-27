module;
#include "st_device.h"

export module device.rcc_init;

export {
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

enum class RCCKernelClock : uint8_t {   // CCIPRx xxxSEL encodings
    LSE,
    LSI,
    PLL2P,
    HSE,
    HSI,
    HSI48,
};

enum class RCCPLLSource : uint8_t {   // PLL1CFGR PLL1SRC encodings
    NONE,                             // stay on the 64 MHz HSI
    HSI,
    HSE,
};

enum class RCCVoltageScale : uint8_t {   // PWR_VOSCR VOS encodings
    SCALE2 = 1,
    SCALE1 = 2,
    SCALE0 = 3,                          // up to 280 MHz
};

struct RCCClockInit {
    uint32_t cpu_frequency_hz = 64'000'000;            // must match CPU_FREQUENCY_HZ
    RCCPLLSource pll_source = RCCPLLSource::NONE;      // with a PLL source, sysclk = PLL1 P output at cpu_frequency_hz
    uint32_t hse_frequency_hz = 0;
    uint32_t pll2_frequency_hz = 0;                    // PLL2 P output from the same source, 0 = off; kernel clock for the PLAY etc.
    RCCVoltageScale voltage_scale = RCCVoltageScale::SCALE0;
    uint8_t ahb_div_log2 = 0;                          // CFGR2 HPRE, divide by 2^n
    uint8_t apb1_div_log2 = 0;                         // CFGR2 PPRE1
    uint8_t apb2_div_log2 = 0;                         // CFGR2 PPRE2, HRTIM clock
    uint8_t apb3_div_log2 = 0;                         // CFGR2 PPRE3
    RCCKernelClock play_clock = RCCKernelClock::LSE;   // CCIPR3 PLAYSEL, reset selects the (usually absent) LSE
    uint8_t play_clock_div = 0;                        // CCIPR3 PLAYDIV, divide by 2^n
};

void rcc_pll_config_not_possible();   // not constexpr: calling it fails the constant evaluation

struct RCCPLLConfig {
    uint32_t m, n, input_hz, range;
};

// PLL1: source / M -> 2..16 MHz (4..8 preferred) -> x N -> VCO 128..560 MHz
// (wide range) -> / 2 -> sysclk. Q and R outputs stay off.
inline consteval RCCPLLConfig get_rcc_pll_config(uint32_t source_hz, uint32_t sysclk_hz) {
    const uint32_t vco_hz = 2 * sysclk_hz;
    if (vco_hz < 128'000'000 || vco_hz > 560'000'000) rcc_pll_config_not_possible();
    RCCPLLConfig best{};
    int best_score = 0;
    for (uint32_t m = 1; m <= 63; m++) {
        if (source_hz % m) continue;
        const uint32_t in = source_hz / m;
        if (in < 2'000'000 || in > 16'000'000 || vco_hz % in) continue;
        const uint32_t n = vco_hz / in;
        if (n < 4 || n > 512) continue;
        const int score = (in >= 4'000'000 && in <= 8'000'000 ? 3'000 : in > 8'000'000 ? 2'000 : 1'000) + in / 100'000;
        if (score > best_score) {
            best_score = score;
            best = {m, n, in, in < 4'000'000 ? 1u : in < 8'000'000 ? 2u : 3u};
        }
    }
    if (best_score == 0) rcc_pll_config_not_possible();
    return best;
}

// Flash read latency, assumed 42 MHz per wait state at scale 0 as on the
// STM32H5 (250 MHz needs 5), and the programming delay ranges from the LL
// driver (<=68 MHz: 0, <=136: 1, above: 2).
inline consteval uint32_t get_flash_acr(uint32_t hclk_hz) {
    const uint32_t latency = (hclk_hz + 42'000'000 - 1) / 42'000'000 - 1;
    const uint32_t wrhighfreq = hclk_hz <= 68'000'000 ? 0 : hclk_hz <= 136'000'000 ? 1 : 2;
    return latency << FLASH_FLASH_ACR_LATENCY_Pos | wrhighfreq << FLASH_FLASH_ACR_WRHIGHFREQ_Pos | FLASH_FLASH_ACR_PRFTEN_Msk;
}

// CFGR2 prescaler encodings: 0 = /1, otherwise base | (log2 - 1)
inline consteval uint32_t rcc_prescaler(uint8_t div_log2, uint32_t base) {
    return div_log2 ? base | (div_log2 - 1) : 0;
}

template<const RCCClockInit c>
inline consteval uint32_t get_rcc_cfgr2() {
    return rcc_prescaler(c.ahb_div_log2, 0x8) << RCC_RCC_CFGR2_HPRE_Pos |
        rcc_prescaler(c.apb1_div_log2, 0x4) << RCC_RCC_CFGR2_PPRE1_Pos |
        rcc_prescaler(c.apb2_div_log2, 0x4) << RCC_RCC_CFGR2_PPRE2_Pos |
        rcc_prescaler(c.apb3_div_log2, 0x4) << RCC_RCC_CFGR2_PPRE3_Pos;
}


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
    static_assert(c.cpu_frequency_hz == static_cast<uint32_t>(CPU_FREQUENCY_HZ), "CPU_FREQUENCY_HZ in st_device.h must match the clock configuration");
    static_assert(c.pll_source != RCCPLLSource::NONE || c.cpu_frequency_hz == 64'000'000, "without a PLL the cpu runs on the 64 MHz HSI");
    static_assert(c.pll_source != RCCPLLSource::HSE || c.hse_frequency_hz != 0, "hse_frequency_hz is needed with an HSE PLL source");

    // ensure cpu clock is started for us_delay
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if constexpr (c.pll_source != RCCPLLSource::NONE) {
        constexpr uint32_t source_hz = c.pll_source == RCCPLLSource::HSE ? c.hse_frequency_hz : 64'000'000;
        constexpr RCCPLLConfig pll = get_rcc_pll_config(source_hz, c.cpu_frequency_hz);
        constexpr uint32_t hclk_hz = c.cpu_frequency_hz >> c.ahb_div_log2;

        PWR->PWR_VOSCR = static_cast<uint32_t>(c.voltage_scale) << PWR_PWR_VOSCR_VOS_Pos;
        while (!(PWR->PWR_VOSSR & PWR_PWR_VOSSR_VOSRDY_Msk));

        if constexpr (c.pll_source == RCCPLLSource::HSE) {
            RCC->RCC_CR |= RCC_RCC_CR_HSEON_Msk;
            while (!(RCC->RCC_CR & RCC_RCC_CR_HSERDY_Msk));
        }
        RCC->RCC_PLL1CFGR = static_cast<uint32_t>(c.pll_source) << RCC_RCC_PLL1CFGR_PLL1SRC_Pos |
            pll.range << RCC_RCC_PLL1CFGR_PLL1RGE_Pos |
            0 << RCC_RCC_PLL1CFGR_PLL1VCOSEL_Pos |      // wide VCO range
            pll.m << RCC_RCC_PLL1CFGR_PLL1M_Pos |
            RCC_RCC_PLL1CFGR_PLL1PEN_Msk;
        RCC->RCC_PLL1DIVR = (pll.n - 1) << RCC_RCC_PLL1DIVR_PLL1N_Pos |
            (2 - 1) << RCC_RCC_PLL1DIVR_PLL1P_Pos |
            (2 - 1) << RCC_RCC_PLL1DIVR_PLL1Q_Pos |
            (2 - 1) << RCC_RCC_PLL1DIVR_PLL1R_Pos;
        RCC->RCC_CR |= RCC_RCC_CR_PLL1ON_Msk;
        while (!(RCC->RCC_CR & RCC_RCC_CR_PLL1RDY_Msk));

        FLASH->FLASH_ACR = get_flash_acr(hclk_hz);
        RCC->RCC_CFGR2 = get_rcc_cfgr2<c>();
        RCC->RCC_CFGR1 = 2 << RCC_RCC_CFGR1_SYSSW_Pos;   // PLL1
        while ((RCC->RCC_CFGR1 & RCC_RCC_CFGR1_SYSSWS_Msk) != 2 << RCC_RCC_CFGR1_SYSSWS_Pos);

        if constexpr (c.pll2_frequency_hz != 0) {
            constexpr RCCPLLConfig pll2 = get_rcc_pll_config(source_hz, c.pll2_frequency_hz);
            RCC->RCC_PLL2CFGR = static_cast<uint32_t>(c.pll_source) << RCC_RCC_PLL2CFGR_PLL2SRC_Pos |
                pll2.range << RCC_RCC_PLL2CFGR_PLL2RGE_Pos |
                0 << RCC_RCC_PLL2CFGR_PLL2VCOSEL_Pos |
                pll2.m << RCC_RCC_PLL2CFGR_PLL2M_Pos |
                1 << RCC_RCC_PLL2CFGR_PLL2PEN_Pos;
            RCC->RCC_PLL2DIVR = (pll2.n - 1) << RCC_RCC_PLL2DIVR_PLL2N_Pos |
                (2 - 1) << RCC_RCC_PLL2DIVR_PLL2P_Pos |
                (2 - 1) << RCC_RCC_PLL2DIVR_PLL2Q_Pos |
                (2 - 1) << RCC_RCC_PLL2DIVR_PLL2R_Pos;
            RCC->RCC_CR |= RCC_RCC_CR_PLL2ON_Msk;
            while (!(RCC->RCC_CR & RCC_RCC_CR_PLL2RDY_Msk));
        }
    }
    static_assert(c.play_clock != RCCKernelClock::PLL2P || c.pll2_frequency_hz != 0, "PLL2 must be configured to clock the PLAY from it");

    constexpr uint32_t ccipr3 = static_cast<uint32_t>(c.play_clock) << RCC_RCC_CCIPR3_PLAYSEL_Pos |
        c.play_clock_div << RCC_RCC_CCIPR3_PLAYDIV_Pos;
    if constexpr (ccipr3 != 0) {
        RCC->RCC_CCIPR3 = ccipr3;
    }
}

template<RCCInit rcc_init>
inline void device_rcc_config() {
    init_rcc<rcc_init>();
}

template<RCCClockInit rcc_clock_init>
inline void device_rcc_clock_config() {
    init_rcc_clock<rcc_clock_init>();
}
}
