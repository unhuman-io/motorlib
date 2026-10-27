module;
#include <cstddef>
#include <utility>
#include "st_device.h"

export module device.play_init;

export {
// Compile-time configuration of the PLAY (programmable logic array): 16 input
// filters, 16 four-input look-up-table logic elements, 16 outputs. Defaults
// are the register reset values; only registers that differ are written.
// The combinational path works from the bus clock alone; registered outputs,
// filters, software inputs and edge flags need the kernel clock, which resets
// to LSE (RCCClockInit::play_clock).
// The pre-multiplexer sources (FILTxCFG PREMUXSEL) are a per-mux table that
// isn't in the SVD, so those stay raw numbers.

enum class PLAYEnable : bool {
    DISABLED,
    ENABLED
};

enum class PLAYEdgeDetect : uint8_t {   // FILTxCFG EDGEDET
    BYPASSED,
    RISING,
    FALLING,
    BOTH,
};

// Logic element input selects (LExCFG2 INn_SEL / CK_SEL)
consteval uint8_t play_in_le_direct(int le) { return 0x00 + le; }
consteval uint8_t play_in_le_registered(int le) { return 0x10 + le; }
consteval uint8_t play_in_filter(int filter) { return 0x20 + filter; }
consteval uint8_t play_in_sw(int sw_input) { return 0x30 + sw_input; }
inline constexpr uint8_t play_in_none = 0x3F;   // software input 15, the reset value

// Registered output clock gate (LExCFG2 CK_SEL): off, always on, or any play_in_* select
inline constexpr uint8_t play_clock_off = 0x00;
inline constexpr uint8_t play_clock_on = 0x01;
inline constexpr uint8_t play_clock_sw_write = 0x02;

// Output selects (OUTxCFG SEL), the OSR bit number of the source
consteval uint8_t play_out_le_direct(int le) { return PLAY_PLAY_OSR_LEOUTD0_Pos + le; }
consteval uint8_t play_out_le_registered(int le) { return PLAY_PLAY_OSR_LEOUTR0_Pos + le; }

// LUT value for the inverse of one input (in0..in3)
consteval uint16_t play_lut_invert(int input) {
    uint16_t lut = 0;
    for (int i = 0; i < 16; i++) {
        if (!(i & 1 << input)) lut |= 1 << i;
    }
    return lut;
}

// LUT value for a plain copy of one input (in0..in3) to the output
consteval uint16_t play_lut_passthrough(int input) {
    uint16_t lut = 0;
    for (int i = 0; i < 16; i++) {
        if (i & 1 << input) lut |= 1 << i;
    }
    return lut;
}

enum class PLAYFlagEdge : uint8_t {
    RISING,     // FLCTL edge bit 0
    FALLING,    // FLCTL edge bit 1
};

struct PLAYFilterInit {                 // FILTxCFG, reset 0
    uint8_t width = 0;                  // minimum pulse width
    PLAYEdgeDetect edge_detect = PLAYEdgeDetect::BYPASSED;
    uint8_t premux_select = 0;          // 3 bits, which of the mux's 8 sources
};

struct PLAYLogicElementInit {           // LExCFG1 / LExCFG2, reset 0 / 0x00FFFFFF
    uint16_t lut = 0;                   // output for each of the 16 input combinations, bit = in3:in2:in1:in0
    uint8_t in0 = play_in_none;         // play_in_* selects
    uint8_t in1 = play_in_none;
    uint8_t in2 = play_in_none;
    uint8_t in3 = play_in_none;
    uint8_t clock = play_clock_off;     // play_clock_* or play_in_* select for the registered output
    PLAYFlagEdge direct_flag_edge = PLAYFlagEdge::RISING;       // FLCTL FLAGD_EDGEx
    PLAYFlagEdge registered_flag_edge = PLAYFlagEdge::RISING;   // FLCTL FLAGR_EDGEx
    PLAYEnable direct_flag_irq = PLAYEnable::DISABLED;          // FLIER FLAGD_IENx
    PLAYEnable registered_flag_irq = PLAYEnable::DISABLED;      // FLIER FLAGR_IENx
};

struct PLAYOutputInit {                 // OUTxCFG, reset 0
    uint8_t select = 0;                 // play_out_* select
};

struct PLAYInit {
    PLAYFilterInit filter[16];
    PLAYLogicElementInit le[16];
    PLAYOutputInit out[16];
    uint16_t sw_inputs = 0;                                 // SWIN initial state, bit per software input
    PLAYEnable sw_input_write_irq = PLAYEnable::DISABLED;   // IER SWINWC_IEN
    PLAYEnable flag_ctl_write_irq = PLAYEnable::DISABLED;   // IER FLCTLWC_IEN
    PLAYEnable lock = PLAYEnable::ENABLED;                  // CFGCR UNLOCK = 0: the PLAY only runs while its configuration is locked
};

struct PLAYRegsInit {
    uint32_t filt[16];
    uint32_t le1[16];
    uint32_t le2[16];
    uint32_t out[16];
    uint32_t flctl;
    uint32_t flier;
    uint32_t swin;
    uint32_t ier;
};

inline consteval PLAYRegsInit get_play_regs_init(const PLAYInit& p) {
    PLAYRegsInit r{};
    for (int i = 0; i < 16; i++) {
        r.filt[i] = p.filter[i].width << PLAY_PLAY_FILT0CFG_WIDTH_Pos |
            static_cast<uint32_t>(p.filter[i].edge_detect) << PLAY_PLAY_FILT0CFG_EDGEDET_Pos |
            p.filter[i].premux_select << PLAY_PLAY_FILT0CFG_PREMUXSEL_Pos;
        r.le1[i] = p.le[i].lut << PLAY_PLAY_LE0CFG1_LUT_Pos;
        r.le2[i] = p.le[i].in0 << PLAY_PLAY_LE0CFG2_IN0_SEL_Pos |
            p.le[i].in1 << PLAY_PLAY_LE0CFG2_IN1_SEL_Pos |
            p.le[i].in2 << PLAY_PLAY_LE0CFG2_IN2_SEL_Pos |
            p.le[i].in3 << PLAY_PLAY_LE0CFG2_IN3_SEL_Pos |
            p.le[i].clock << PLAY_PLAY_LE0CFG2_CK_SEL_Pos;
        r.out[i] = p.out[i].select << PLAY_PLAY_OUT0CFG_SEL_Pos;
        r.flctl |= static_cast<uint32_t>(p.le[i].direct_flag_edge) << (PLAY_PLAY_FLCTL_FLAGD_EDGE0_Pos + i) |
            static_cast<uint32_t>(p.le[i].registered_flag_edge) << (PLAY_PLAY_FLCTL_FLAGR_EDGE0_Pos + i);
        r.flier |= static_cast<uint32_t>(p.le[i].direct_flag_irq) << (PLAY_PLAY_FLIER_FLAGD_IEN0_Pos + i) |
            static_cast<uint32_t>(p.le[i].registered_flag_irq) << (PLAY_PLAY_FLIER_FLAGR_IEN0_Pos + i);
    }
    r.swin = p.sw_inputs;
    r.ier = static_cast<uint32_t>(p.sw_input_write_irq) << PLAY_PLAY_IER_SWINWC_IEN_Pos |
        static_cast<uint32_t>(p.flag_ctl_write_irq) << PLAY_PLAY_IER_FLCTLWC_IEN_Pos;
    return r;
}

// The per-filter/element/output registers are contiguous in the header
}
static_assert(offsetof(PLAY_Type, PLAY_FILT15CFG) - offsetof(PLAY_Type, PLAY_FILT0CFG) == 15 * 4);
static_assert(offsetof(PLAY_Type, PLAY_LE15CFG1) - offsetof(PLAY_Type, PLAY_LE0CFG1) == 15 * 4);
static_assert(offsetof(PLAY_Type, PLAY_LE15CFG2) - offsetof(PLAY_Type, PLAY_LE0CFG2) == 15 * 4);
static_assert(offsetof(PLAY_Type, PLAY_OUT15CFG) - offsetof(PLAY_Type, PLAY_OUT0CFG) == 15 * 4);
export {

template<uint32_t v, uint32_t v_default>
inline void play_write_if_changed(volatile uint32_t& reg) {
    if constexpr (v != v_default) reg = v;
}

template<const PLAYRegsInit r, const PLAYRegsInit d, size_t... I>
inline void init_play_regs(std::index_sequence<I...>) {
    (play_write_if_changed<r.filt[I], d.filt[I]>((&PLAY->PLAY_FILT0CFG)[I]), ...);
    (play_write_if_changed<r.le1[I], d.le1[I]>((&PLAY->PLAY_LE0CFG1)[I]), ...);
    (play_write_if_changed<r.le2[I], d.le2[I]>((&PLAY->PLAY_LE0CFG2)[I]), ...);
    (play_write_if_changed<r.out[I], d.out[I]>((&PLAY->PLAY_OUT0CFG)[I]), ...);
}

template<const PLAYInit p>
inline void init_play() {
    constexpr PLAYRegsInit r = get_play_regs_init(p);
    constexpr PLAYRegsInit d = get_play_regs_init(PLAYInit{});
    // Configuration and lock survive a system reset, and config writes are
    // ignored while locked, so always unlock first.
    PLAY->PLAY_CFGCR = PLAY_PLAY_CFGCR_UNLOCK_Msk;
    init_play_regs<r, d>(std::make_index_sequence<16>{});
    play_write_if_changed<r.flctl, d.flctl>(PLAY->PLAY_FLCTL);
    play_write_if_changed<r.flier, d.flier>(PLAY->PLAY_FLIER);
    play_write_if_changed<r.ier, d.ier>(PLAY->PLAY_IER);
    if constexpr (r.swin != d.swin) {
        PLAY->PLAY_SWINSET = r.swin;
    }
    if constexpr (p.lock == PLAYEnable::ENABLED) {
        PLAY->PLAY_CFGCR = 0;
    }
}

template<PLAYInit play_init>
inline void device_play_config() {
    init_play<play_init>();
}

// Runtime software inputs. SWIN writes are double-buffered: MSR SWINWBFS is
// set until the write has taken effect.
inline void play_set_inputs(uint16_t mask) { PLAY->PLAY_SWINSET = mask; }
inline void play_clear_inputs(uint16_t mask) { PLAY->PLAY_SWINCLR = mask; }
inline uint32_t play_outputs() { return PLAY->PLAY_OSR; }
}
