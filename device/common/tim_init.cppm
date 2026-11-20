module;
#include <cstddef>
#include <cstdint>
#include "tim_regs.h"

export module device.tim_init;

export {
// Compile-time configuration of the general purpose and advanced timers,
// shared between devices: the registers used here are the same on the
// stm32g4 and stm32g5; the device's tim_regs.h supplies the addresses and
// what each timer has. One TIMInit holds all of them. This covers the time
// base and the compare outputs of channels 1 to 4: no input capture, slave
// modes, complementary outputs or dead time.

enum class TIMEnable : bool {
    DISABLED,
    ENABLED
};

enum class TIMChannelMode : uint8_t {   // CCMRx OCxM: what the output does
    FROZEN,                 // nothing: the compare value only raises flags
    ACTIVE_ON_MATCH,        // goes active when the counter reaches the compare value
    INACTIVE_ON_MATCH,
    TOGGLE,                 // one edge per period, at the compare value
    FORCE_INACTIVE,
    FORCE_ACTIVE,
    PWM1,                   // active while counter < compare
    PWM2,                   // inactive while counter < compare
};

// Defaults are the register reset values
struct TIMChannelInit {
    TIMEnable output = TIMEnable::DISABLED;         // CCER CCxE: drives the pin, which is otherwise released
    TIMChannelMode mode = TIMChannelMode::FROZEN;
    TIMEnable preload = TIMEnable::DISABLED;        // CCMRx OCxPE: a new compare value takes effect at the next update
    TIMEnable inverted = TIMEnable::DISABLED;       // CCER CCxP: active low
    uint32_t compare = 0;                           // CCRx
};

struct TIMTimerInit {
    TIMEnable enable = TIMEnable::DISABLED;         // CR1 CEN
    uint16_t prescaler = 0;                         // PSC: the counter runs at the timer clock / (prescaler + 1)
    uint32_t period = 0;                            // counts per period, ARR + 1. 0 leaves ARR at reset: the full range.
    TIMEnable period_preload = TIMEnable::DISABLED; // CR1 ARPE: a new period takes effect at the next update
    TIMEnable update_interrupt = TIMEnable::DISABLED;   // DIER UIE
    TIMEnable update_dma = TIMEnable::DISABLED;     // DIER UDE: a DMA request at every update
    TIMEnable main_output = TIMEnable::DISABLED;    // BDTR MOE: timers with a break input need it for their outputs
    TIMChannelInit ch1, ch2, ch3, ch4;
};

struct TIMInit {
    TIMTimerInit tim1, tim2, tim3, tim4, tim5, tim6, tim7, tim8;
    TIMTimerInit tim9, tim10, tim11, tim12, tim13, tim14;      // stm32g5 only
    TIMTimerInit tim15, tim16, tim17, tim20;
    TIMTimerInit tim26, tim27;                                  // stm32g5 only
};

// Registers common to the timers of the stm32g4 and stm32g5, stm32g4 names.
// A timer without channels or without a break input has those reserved.
struct TIMRegs {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR,
        CCR1, CCR2, CCR3, CCR4, BDTR;
};
static_assert(offsetof(TIMRegs, DIER) == tim_offsets::DIER && offsetof(TIMRegs, EGR) == tim_offsets::EGR &&
    offsetof(TIMRegs, CCMR1) == tim_offsets::CCMR1 && offsetof(TIMRegs, CCER) == tim_offsets::CCER &&
    offsetof(TIMRegs, CNT) == tim_offsets::CNT && offsetof(TIMRegs, PSC) == tim_offsets::PSC &&
    offsetof(TIMRegs, ARR) == tim_offsets::ARR && offsetof(TIMRegs, CCR1) == tim_offsets::CCR1 &&
    offsetof(TIMRegs, CCR4) == tim_offsets::CCR4 && offsetof(TIMRegs, BDTR) == tim_offsets::BDTR);

// Channels of a timer, -1 where the device has no such timer
inline consteval int tim_channel_count(int tim) {
    return tim >= 1 && tim <= tim_count && tim_base[tim - 1] != 0 ? tim_channels[tim - 1] : -1;
}

// tim_regs<3>() is TIM3
template<int tim>
inline TIMRegs& tim_regs() {
    static_assert(tim_channel_count(tim) >= 0, "no such timer on this device");
    return *reinterpret_cast<TIMRegs*>(tim_base[tim - 1]);
}

// Bit positions that are the same on both devices, checked against the
// device header. Channel n has its CCER bits 4 * (n - 1) up, and its CCMR
// bits 8 up for the second channel of each register.
namespace tim_bit {
inline constexpr uint32_t CR1_CEN = 1u << 0, CR1_ARPE = 1u << 7, DIER_UIE = 1u << 0, DIER_UDE = 1u << 8, EGR_UG = 1u << 0;
inline constexpr uint32_t CCMR_OCPE = 1u << 3, CCMR_OCM_Pos = 4, CCMR_CHANNEL_Width = 8;
inline constexpr uint32_t CCER_CCE = 1u << 0, CCER_CCP = 1u << 1, CCER_CHANNEL_Width = 4, BDTR_MOE = 1u << 15;
}
static_assert(tim_bit::CR1_CEN == tim_bits::CR1_CEN && tim_bit::CR1_ARPE == tim_bits::CR1_ARPE &&
    tim_bit::DIER_UIE == tim_bits::DIER_UIE && tim_bit::DIER_UDE == tim_bits::DIER_UDE && tim_bit::EGR_UG == tim_bits::EGR_UG);
static_assert(tim_bit::CCMR_OCPE == tim_bits::CCMR1_OC1PE && tim_bit::CCMR_OCM_Pos == tim_bits::CCMR1_OC1M_Pos &&
    tim_bit::CCMR_OCM_Pos + tim_bit::CCMR_CHANNEL_Width == tim_bits::CCMR1_OC2M_Pos);
static_assert(tim_bit::CCER_CCE == tim_bits::CCER_CC1E && tim_bit::CCER_CCP == tim_bits::CCER_CC1P &&
    tim_bit::CCER_CCE << tim_bit::CCER_CHANNEL_Width == tim_bits::CCER_CC2E && tim_bit::BDTR_MOE == tim_bits::BDTR_MOE);

inline consteval bool tim_channel_used(const TIMChannelInit& c) {
    return c.output == TIMEnable::ENABLED || c.mode != TIMChannelMode::FROZEN || c.preload == TIMEnable::ENABLED ||
        c.inverted == TIMEnable::ENABLED || c.compare != 0;
}
inline consteval int tim_channels_used(const TIMTimerInit& c) {
    return tim_channel_used(c.ch4) ? 4 : tim_channel_used(c.ch3) ? 3 : tim_channel_used(c.ch2) ? 2 : tim_channel_used(c.ch1) ? 1 : 0;
}
inline consteval bool tim_used(const TIMTimerInit& c) {
    return c.enable == TIMEnable::ENABLED || c.prescaler != 0 || c.period != 0 || c.period_preload == TIMEnable::ENABLED ||
        c.update_interrupt == TIMEnable::ENABLED || c.update_dma == TIMEnable::ENABLED || c.main_output == TIMEnable::ENABLED ||
        tim_channels_used(c) != 0;
}

inline consteval uint32_t get_tim_channel_ccmr(const TIMChannelInit& c) {
    using namespace tim_bit;
    return static_cast<uint32_t>(c.mode) << CCMR_OCM_Pos | (c.preload == TIMEnable::ENABLED ? CCMR_OCPE : 0);
}
// CCMR1 (n = 0) holds channels 1 and 2, CCMR2 (n = 1) channels 3 and 4
inline consteval uint32_t get_tim_ccmr(const TIMTimerInit& c, int n) {
    return get_tim_channel_ccmr(n == 0 ? c.ch1 : c.ch3) | get_tim_channel_ccmr(n == 0 ? c.ch2 : c.ch4) << tim_bit::CCMR_CHANNEL_Width;
}
inline consteval uint32_t get_tim_channel_ccer(const TIMChannelInit& c) {
    using namespace tim_bit;
    return (c.output == TIMEnable::ENABLED ? CCER_CCE : 0) | (c.inverted == TIMEnable::ENABLED ? CCER_CCP : 0);
}
inline consteval uint32_t get_tim_ccer(const TIMTimerInit& c) {
    using namespace tim_bit;
    return get_tim_channel_ccer(c.ch1) | get_tim_channel_ccer(c.ch2) << CCER_CHANNEL_Width |
        get_tim_channel_ccer(c.ch3) << 2 * CCER_CHANNEL_Width | get_tim_channel_ccer(c.ch4) << 3 * CCER_CHANNEL_Width;
}
inline consteval uint32_t get_tim_dier(const TIMTimerInit& c) {
    using namespace tim_bit;
    return (c.update_interrupt == TIMEnable::ENABLED ? DIER_UIE : 0) | (c.update_dma == TIMEnable::ENABLED ? DIER_UDE : 0);
}
inline consteval uint32_t get_tim_cr1(const TIMTimerInit& c) {
    using namespace tim_bit;
    return (c.enable == TIMEnable::ENABLED ? CR1_CEN : 0) | (c.period_preload == TIMEnable::ENABLED ? CR1_ARPE : 0);
}

// The timer's clock (RCC enable) must be running. Only registers that differ
// from reset are written. The prescaler is loaded by a software update
// before the requests and the counter are enabled.
template<const TIMTimerInit c, int tim>
inline void init_tim_timer() {
    using namespace tim_bit;
    static_assert(!tim_used(c) || tim_channel_count(tim) >= 0, "no such timer on this device");
    static_assert(!tim_used(c) || tim_channels_used(c) <= tim_channel_count(tim), "this timer does not have that channel");
    if constexpr (tim_used(c)) {
        static_assert(tim_32bit[tim - 1] || c.period <= 0x10000, "the period of this timer is at most 65536 counts");
        static_assert(tim_main_output[tim - 1] || c.main_output == TIMEnable::DISABLED, "this timer has no main output enable");
        TIMRegs& regs = tim_regs<tim>();
        if constexpr (c.prescaler != 0) regs.PSC = c.prescaler;
        if constexpr (c.period != 0) regs.ARR = c.period - 1;
        if constexpr (c.ch1.compare != 0) regs.CCR1 = c.ch1.compare;
        if constexpr (c.ch2.compare != 0) regs.CCR2 = c.ch2.compare;
        if constexpr (c.ch3.compare != 0) regs.CCR3 = c.ch3.compare;
        if constexpr (c.ch4.compare != 0) regs.CCR4 = c.ch4.compare;
        if constexpr (get_tim_ccmr(c, 0) != 0) regs.CCMR1 = get_tim_ccmr(c, 0);
        if constexpr (get_tim_ccmr(c, 1) != 0) regs.CCMR2 = get_tim_ccmr(c, 1);
        if constexpr (c.prescaler != 0) {
            regs.EGR = EGR_UG;
            regs.SR = 0;
        }
        if constexpr (get_tim_ccer(c) != 0) regs.CCER = get_tim_ccer(c);
        if constexpr (c.main_output == TIMEnable::ENABLED) regs.BDTR = BDTR_MOE;
        if constexpr (get_tim_dier(c) != 0) regs.DIER = get_tim_dier(c);
        if constexpr (get_tim_cr1(c) != 0) regs.CR1 = get_tim_cr1(c);
    }
}

template<const TIMInit h>
inline void init_tim() {
    init_tim_timer<h.tim1, 1>();
    init_tim_timer<h.tim2, 2>();
    init_tim_timer<h.tim3, 3>();
    init_tim_timer<h.tim4, 4>();
    init_tim_timer<h.tim5, 5>();
    init_tim_timer<h.tim6, 6>();
    init_tim_timer<h.tim7, 7>();
    init_tim_timer<h.tim8, 8>();
    init_tim_timer<h.tim9, 9>();
    init_tim_timer<h.tim10, 10>();
    init_tim_timer<h.tim11, 11>();
    init_tim_timer<h.tim12, 12>();
    init_tim_timer<h.tim13, 13>();
    init_tim_timer<h.tim14, 14>();
    init_tim_timer<h.tim15, 15>();
    init_tim_timer<h.tim16, 16>();
    init_tim_timer<h.tim17, 17>();
    init_tim_timer<h.tim20, 20>();
    init_tim_timer<h.tim26, 26>();
    init_tim_timer<h.tim27, 27>();
}

template<TIMInit tim_init>
inline void device_tim_config() {
    init_tim<tim_init>();
}

// Counter value: tim_counter<3>() is TIM3
template<int tim>
inline uint32_t tim_counter() {
    return tim_regs<tim>().CNT;
}

// New compare value of a channel: tim_compare<3, 4>(value) is TIM3 channel 4
template<int tim, int channel>
inline void tim_compare(uint32_t value) {
    static_assert(channel >= 1 && channel <= tim_channel_count(tim), "this timer does not have that channel");
    (&tim_regs<tim>().CCR1)[channel - 1] = value;
}
}
