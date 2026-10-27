#pragma once
// Compile-time HRTIM configuration, shared between devices. Register values
// are built at compile time and only registers that differ from reset are
// written. Device specifics (register map, bit names, clock) come from the
// device's hrtim_regs.h.
#include <cstdint>
#include "hrtim_regs.h"

enum class HRTIMEnable : bool {
    DISABLED,
    ENABLED
};

enum class HRTIMMode : uint8_t {
    EDGE_ALIGNED,     // counts up to period
    CENTER_ALIGNED,   // counts up to period then down; set/reset events swap on the way down
};

enum class HRTIMRollover : uint8_t {   // which roll-over of a center aligned counter generates the event
    BOTH,                              // counter = 0 and counter = period
    VALLEY,                            // counter = 0
    CREST,                             // counter = period, the middle of the pwm pulse
};

// Output set/reset sources (SETxyR / RSTxyR bits), combinable with |
enum class HRTIMEvent : uint32_t {
    NONE          = 0,
    SOFTWARE      = 1u << 0,
    RESYNC        = 1u << 1,
    PERIOD        = 1u << 2,
    CMP1          = 1u << 3,
    CMP2          = 1u << 4,
    CMP3          = 1u << 5,
    CMP4          = 1u << 6,
    MASTER_PERIOD = 1u << 7,
    MASTER_CMP1   = 1u << 8,
    MASTER_CMP2   = 1u << 9,
    MASTER_CMP3   = 1u << 10,
    MASTER_CMP4   = 1u << 11,
};
constexpr HRTIMEvent operator|(HRTIMEvent a, HRTIMEvent b) {
    return static_cast<HRTIMEvent>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

// ADC trigger 1 and 3 sources (ADC1R / ADC3R bits; triggers 2 and 4 have a
// different layout and aren't covered), combinable with |
enum class HRTIMADCTrigger : uint32_t {
    NONE          = 0,
    MASTER_CMP1   = 1u << 0,
    MASTER_CMP2   = 1u << 1,
    MASTER_CMP3   = 1u << 2,
    MASTER_CMP4   = 1u << 3,
    MASTER_PERIOD = 1u << 4,
    TIMF_CMP2     = 1u << 10,
    TIMA_CMP3     = 1u << 11,
    TIMA_CMP4     = 1u << 12,
    TIMA_PERIOD   = 1u << 13,
    TIMA_RESET    = 1u << 14,
    TIMF_CMP3     = 1u << 15,
    TIMB_CMP3     = 1u << 16,
    TIMB_CMP4     = 1u << 17,
    TIMB_PERIOD   = 1u << 18,
    TIMB_RESET    = 1u << 19,
    TIMF_CMP4     = 1u << 20,
    TIMC_CMP3     = 1u << 21,
    TIMC_CMP4     = 1u << 22,
    TIMC_PERIOD   = 1u << 23,
    TIMF_PERIOD   = 1u << 24,
    TIMD_CMP3     = 1u << 25,
    TIMD_CMP4     = 1u << 26,
    TIMD_PERIOD   = 1u << 27,
    TIMF_RESET    = 1u << 28,
    TIME_CMP3     = 1u << 29,
    TIME_CMP4     = 1u << 30,
    TIME_PERIOD   = 1u << 31,
};
constexpr HRTIMADCTrigger operator|(HRTIMADCTrigger a, HRTIMADCTrigger b) {
    return static_cast<HRTIMADCTrigger>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

enum class HRTIMTimer : uint8_t {
    MASTER, A, B, C, D, E, F
};

static_assert(static_cast<uint32_t>(HRTIMEvent::SOFTWARE) == hrtim_bits::SET1R_SST);
static_assert(static_cast<uint32_t>(HRTIMEvent::PERIOD) == hrtim_bits::SET1R_PER);
static_assert(static_cast<uint32_t>(HRTIMEvent::CMP1) == hrtim_bits::SET1R_CMP1);
static_assert(static_cast<uint32_t>(HRTIMEvent::CMP4) == hrtim_bits::SET1R_CMP4);
static_assert(static_cast<uint32_t>(HRTIMEvent::MASTER_CMP4) == hrtim_bits::SET1R_MSTCMP4);
static_assert(static_cast<uint32_t>(HRTIMADCTrigger::TIMA_CMP3) == hrtim_bits::ADC1R_AD1TAC3);
static_assert(static_cast<uint32_t>(HRTIMADCTrigger::TIMF_CMP3) == hrtim_bits::ADC1R_AD1TFC3);
static_assert(static_cast<uint32_t>(HRTIMADCTrigger::TIMF_CMP4) == hrtim_bits::ADC1R_AD1TFC4);
static_assert(static_cast<uint32_t>(HRTIMADCTrigger::TIME_PERIOD) == hrtim_bits::ADC1R_AD1TEPER);

// Defaults are the register reset values
struct HRTIMTimerInit {
    HRTIMEnable enable = HRTIMEnable::DISABLED;            // MCR TxCEN, set last
    HRTIMMode mode = HRTIMMode::EDGE_ALIGNED;              // TIMxCR2 UDM
    HRTIMRollover adc_rollover = HRTIMRollover::BOTH;      // TIMxCR2 ADROM, for the TIMx_PERIOD adc triggers
    HRTIMEnable cmp1_greater_than = HRTIMEnable::DISABLED; // TIMxCR2 GTCMP1: event when counter > cmp1, once per period when center aligned
    HRTIMEnable cmp3_greater_than = HRTIMEnable::DISABLED; // TIMxCR2 GTCMP3
    uint8_t prescaler = 0;                                 // TIMxCR CKPSC: timer clock = 32 * hrtim_clock_hz >> prescaler
    HRTIMEnable continuous = HRTIMEnable::DISABLED;        // TIMxCR CONT
    HRTIMEnable preload = HRTIMEnable::DISABLED;           // TIMxCR PREEN
    HRTIMEnable update_on_reset = HRTIMEnable::DISABLED;   // TIMxCR TxRSTU
    uint16_t period = 0xFFDF;                              // PERxR
    uint16_t cmp1 = 0;                                     // CMP1xR
    uint16_t cmp2 = 0;
    uint16_t cmp3 = 0;
    uint16_t cmp4 = 0;
    HRTIMEvent out1_set = HRTIMEvent::NONE;                // SETx1R
    HRTIMEvent out1_reset = HRTIMEvent::NONE;              // RSTx1R
    HRTIMEvent out2_set = HRTIMEvent::NONE;                // SETx2R
    HRTIMEvent out2_reset = HRTIMEvent::NONE;              // RSTx2R
    HRTIMEnable out1 = HRTIMEnable::DISABLED;              // OENR
    HRTIMEnable out2 = HRTIMEnable::DISABLED;
    HRTIMEnable reset_on_master_period = HRTIMEnable::DISABLED;   // RSTxR MSTPER: counter restarts from 0 on the master period, keeps timers in phase
};

struct HRTIMMasterInit {
    HRTIMEnable enable = HRTIMEnable::DISABLED;            // MCR MCEN
    uint8_t prescaler = 0;                                 // MCR CKPSC
    HRTIMEnable continuous = HRTIMEnable::DISABLED;        // MCR CONT
    HRTIMEnable preload = HRTIMEnable::DISABLED;           // MCR PREEN
    uint16_t period = 0xFFDF;                              // MPER
};

struct HRTIMInit {
    HRTIMMasterInit master;
    HRTIMTimerInit a, b, c, d, e, f;
    HRTIMADCTrigger adc_trigger1 = HRTIMADCTrigger::NONE;  // ADC1R
    HRTIMADCTrigger adc_trigger3 = HRTIMADCTrigger::NONE;  // ADC3R
    // ADCxR are preloaded; the active register is only loaded on the update
    // event of this timer, so it must be one that runs and updates.
    HRTIMTimer adc_trigger1_update = HRTIMTimer::MASTER;   // CR1 AD1USRC
    HRTIMTimer adc_trigger3_update = HRTIMTimer::MASTER;   // CR1 AD3USRC
    HRTIMEnable dll_calibration = HRTIMEnable::ENABLED;    // DLLCR periodic calibration, required for the x32 clock
};

void hrtim_period_out_of_range();   // not constexpr: calling it fails the constant evaluation

// Period register value for a pwm frequency. Center aligned counts up and
// down, so the register holds half the pwm period.
consteval uint16_t hrtim_period(uint32_t pwm_frequency_hz, uint8_t prescaler = 0, HRTIMMode mode = HRTIMMode::EDGE_ALIGNED) {
    float timer_hz = hrtim_clock_hz * 32.f / (1u << prescaler);
    float counts = timer_hz / pwm_frequency_hz / (mode == HRTIMMode::CENTER_ALIGNED ? 2.f : 1.f);
    if (counts > 0xFFDF || counts < 3) hrtim_period_out_of_range();
    return counts;
}

struct HRTIMTimerRegsInit {
    uint32_t cr, cr2, per, cmp1, cmp2, cmp3, cmp4, set1, rst1, set2, rst2, rst;
};

inline consteval HRTIMTimerRegsInit get_hrtim_timer_regs_init(const HRTIMTimerInit& t) {
    using namespace hrtim_bits;
    HRTIMTimerRegsInit r{};
    r.cr = t.prescaler << TIMCR_CKPSC_Pos |
        (t.continuous == HRTIMEnable::ENABLED ? TIMCR_CONT : 0) |
        (t.preload == HRTIMEnable::ENABLED ? TIMCR_PREEN : 0) |
        (t.update_on_reset == HRTIMEnable::ENABLED ? TIMCR_TRSTU : 0);
    r.cr2 = (t.mode == HRTIMMode::CENTER_ALIGNED ? TIMCR2_UDM : 0) |
        static_cast<uint32_t>(t.adc_rollover) << TIMCR2_ADROM_Pos |
        (t.cmp1_greater_than == HRTIMEnable::ENABLED ? TIMCR2_GTCMP1 : 0) |
        (t.cmp3_greater_than == HRTIMEnable::ENABLED ? TIMCR2_GTCMP3 : 0);
    r.per = t.period;
    r.cmp1 = t.cmp1;
    r.cmp2 = t.cmp2;
    r.cmp3 = t.cmp3;
    r.cmp4 = t.cmp4;
    r.set1 = static_cast<uint32_t>(t.out1_set);
    r.rst1 = static_cast<uint32_t>(t.out1_reset);
    r.set2 = static_cast<uint32_t>(t.out2_set);
    r.rst2 = static_cast<uint32_t>(t.out2_reset);
    r.rst = t.reset_on_master_period == HRTIMEnable::ENABLED ? RSTR_MSTPER : 0;
    return r;
}

// TIMxCR (with the preload enable) is written last so that period, compares
// and output events take effect immediately instead of at the timer's first
// roll-over of the reset period, which would leave the timers out of phase.
template<const HRTIMTimerRegsInit r, const HRTIMTimerRegsInit r_default>
inline void init_hrtim_timer_regs(HRTIMTimerRegs& regs) {
    if constexpr (r.cr2 != r_default.cr2) regs.TIMxCR2 = r.cr2;
    if constexpr (r.per != r_default.per) regs.PERxR = r.per;
    if constexpr (r.cmp1 != r_default.cmp1) regs.CMP1xR = r.cmp1;
    if constexpr (r.cmp2 != r_default.cmp2) regs.CMP2xR = r.cmp2;
    if constexpr (r.cmp3 != r_default.cmp3) regs.CMP3xR = r.cmp3;
    if constexpr (r.cmp4 != r_default.cmp4) regs.CMP4xR = r.cmp4;
    if constexpr (r.set1 != r_default.set1) regs.SETx1R = r.set1;
    if constexpr (r.rst1 != r_default.rst1) regs.RSTx1R = r.rst1;
    if constexpr (r.set2 != r_default.set2) regs.SETx2R = r.set2;
    if constexpr (r.rst2 != r_default.rst2) regs.RSTx2R = r.rst2;
    if constexpr (r.rst != r_default.rst) regs.RSTxR = r.rst;
    if constexpr (r.cr != r_default.cr) regs.TIMxCR = r.cr;
}

template<const HRTIMInit h>
inline consteval uint32_t get_hrtim_oenr() {
    uint32_t v = 0;
    const HRTIMTimerInit timers[] = {h.a, h.b, h.c, h.d, h.e, h.f};
    for (int ch = 0; ch < 6; ch++) {
        v |= static_cast<uint32_t>(timers[ch].out1) << 2 * ch;
        v |= static_cast<uint32_t>(timers[ch].out2) << (2 * ch + 1);
    }
    return v;
}

template<const HRTIMInit h>
inline consteval uint32_t get_hrtim_mcr() {
    using namespace hrtim_bits;
    uint32_t v = h.master.prescaler << MCR_CKPSC_Pos |
        (h.master.continuous == HRTIMEnable::ENABLED ? MCR_CONT : 0) |
        (h.master.preload == HRTIMEnable::ENABLED ? MCR_PREEN : 0) |
        (h.master.enable == HRTIMEnable::ENABLED ? MCR_MCEN : 0);
    const HRTIMTimerInit timers[] = {h.a, h.b, h.c, h.d, h.e, h.f};
    for (int ch = 0; ch < 6; ch++) {
        v |= static_cast<uint32_t>(timers[ch].enable) << (hrtim_bits::MCR_TACEN_Pos + ch);
    }
    return v;
}

template<const HRTIMInit h>
inline void init_hrtim() {
    using namespace hrtim_bits;
    constexpr HRTIMTimerRegsInit d = get_hrtim_timer_regs_init(HRTIMTimerInit{});
    init_hrtim_timer_regs<get_hrtim_timer_regs_init(h.a), d>(hrtim_timer(0));
    init_hrtim_timer_regs<get_hrtim_timer_regs_init(h.b), d>(hrtim_timer(1));
    init_hrtim_timer_regs<get_hrtim_timer_regs_init(h.c), d>(hrtim_timer(2));
    init_hrtim_timer_regs<get_hrtim_timer_regs_init(h.d), d>(hrtim_timer(3));
    init_hrtim_timer_regs<get_hrtim_timer_regs_init(h.e), d>(hrtim_timer(4));
    init_hrtim_timer_regs<get_hrtim_timer_regs_init(h.f), d>(hrtim_timer(5));

    auto& common = hrtim_common();
    constexpr uint32_t cr1 = static_cast<uint32_t>(h.adc_trigger1_update) << CR1_AD1USRC_Pos |
        static_cast<uint32_t>(h.adc_trigger3_update) << CR1_AD3USRC_Pos;
    if constexpr (cr1 != 0) common.CR1 = cr1;
    if constexpr (h.adc_trigger1 != HRTIMADCTrigger::NONE) common.ADC1R = static_cast<uint32_t>(h.adc_trigger1);
    if constexpr (h.adc_trigger3 != HRTIMADCTrigger::NONE) common.ADC3R = static_cast<uint32_t>(h.adc_trigger3);
    if constexpr (h.dll_calibration == HRTIMEnable::ENABLED) common.DLLCR = DLLCR_CALEN | 3 << DLLCR_CALRTE_Pos;  // every 2048 hrtim clocks
    if constexpr (get_hrtim_oenr<h>() != 0) common.OENR = get_hrtim_oenr<h>();
    if constexpr (h.master.period != HRTIMMasterInit{}.period) hrtim_master().MPER = h.master.period;
    if constexpr (get_hrtim_mcr<h>() != 0) hrtim_master().MCR = get_hrtim_mcr<h>();
}

template<HRTIMInit hrtim_init>
inline void device_hrtim_config() {
    init_hrtim<hrtim_init>();
}
