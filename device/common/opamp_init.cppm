module;
#include <cstdint>
#include "opamp_regs.h"
#include "../../task.h"

export module device.opamp_init;

export {
// Compile-time OPAMP configuration, shared between devices: the control
// register (CSR) is the same on the stm32g4 and stm32g5; the device's
// opamp_regs.h supplies the addresses and lists the pins and ADC channels of
// each amplifier. One OPAMPInit holds all of them.

enum class OPAMPEnable : bool {
    DISABLED,
    ENABLED
};

enum class OPAMPInput : uint8_t {   // CSR VP_SEL: non inverting input
    VINP0,
    VINP1,
    VINP2,
    VINP3,                          // a pin or a DAC channel, depending on the instance
};

enum class OPAMPMode : uint8_t {    // CSR VM_SEL: inverting input
    VINM0,                          // standalone, feedback through the VINM0 pin
    VINM1,                          // standalone, feedback through the VINM1 pin
    PGA,                            // internal feedback resistors, gain below
    FOLLOWER,                       // output fed back: gain 1
};

enum class OPAMPGain : uint8_t {    // CSR PGA_GAIN low bits, PGA mode
    X2,
    X4,
    X8,
    X16,
    X32,                            // stm32g4 only
    X64,                            // stm32g4 only
};

enum class OPAMPPGAInput : uint8_t {   // CSR PGA_GAIN high bits, PGA mode: use of the VINM pins
    NONE,                              // non inverting gain, VINM pins free
    BIAS,                              // VINM0 is the low end of the resistors: inverting input or bias, gains become -1/2, -3/4, ...
    FILTER,                            // VINM0 is the amplifier's inverting node, for a filter capacitor
    BIAS_FILTER,                       // VINM0 bias, VINM1 filter
};

// Defaults are the register reset values
struct OPAMPAmplifierInit {
    OPAMPEnable enable = OPAMPEnable::DISABLED;             // CSR OPAEN
    OPAMPInput input = OPAMPInput::VINP0;
    OPAMPMode mode = OPAMPMode::VINM0;
    OPAMPEnable high_speed = OPAMPEnable::DISABLED;         // CSR OPAHSM: faster slew and settling, more supply current
    OPAMPEnable internal_output = OPAMPEnable::DISABLED;    // CSR OPAINTOEN: output goes to the ADC channel and leaves the VOUT pin free
    OPAMPGain gain = OPAMPGain::X2;
    OPAMPPGAInput pga_input = OPAMPPGAInput::NONE;
    OPAMPEnable lock = OPAMPEnable::DISABLED;               // CSR LOCK: read only until reset
};

struct OPAMPInit {
    OPAMPAmplifierInit opamp1, opamp2, opamp3;
    OPAMPAmplifierInit opamp4, opamp5, opamp6;              // stm32g4 only
};

// opamp_csr<3>() is the OPAMP3 control register
template<int opamp>
inline volatile uint32_t& opamp_csr() {
    static_assert(opamp >= 1 && opamp <= opamp_count, "no such opamp on this device");
    return *reinterpret_cast<volatile uint32_t*>(opamp_base[opamp - 1]);
}

void opamp_gain_not_available();   // not constexpr: calling it fails the constant evaluation

// Same bit positions on both devices, checked against the device header.
// The init leaves the factory offset trim in use (USERTRIM 0):
// opamp_calibrate_async replaces it.
namespace opamp_bit {
inline constexpr uint32_t CSR_OPAEN = 1u << 0, CSR_VP_SEL_Pos = 2, CSR_USERTRIM = 1u << 4, CSR_VM_SEL_Pos = 5, CSR_OPAHSM = 1u << 7,
    CSR_OPAINTOEN = 1u << 8, CSR_CALON = 1u << 11, CSR_CALSEL_Pos = 12, CSR_PGA_GAIN_Pos = 14, CSR_PGA_INPUT_Pos = 17,
    CSR_TRIMOFFSETP_Pos = 19, CSR_TRIMOFFSETN_Pos = 24, CSR_TRIMOFFSET_Width = 5, CSR_CALOUT = 1u << 30, CSR_LOCK = 1u << 31;
}
static_assert(opamp_bit::CSR_USERTRIM == opamp_bits::CSR_USERTRIM && opamp_bit::CSR_CALON == opamp_bits::CSR_CALON &&
    opamp_bit::CSR_CALSEL_Pos == opamp_bits::CSR_CALSEL_Pos && opamp_bit::CSR_TRIMOFFSETP_Pos == opamp_bits::CSR_TRIMOFFSETP_Pos &&
    opamp_bit::CSR_TRIMOFFSETN_Pos == opamp_bits::CSR_TRIMOFFSETN_Pos && opamp_bit::CSR_CALOUT == opamp_bits::CSR_CALOUT);
static_assert(opamp_bit::CSR_OPAEN == opamp_bits::CSR_OPAEN && opamp_bit::CSR_VP_SEL_Pos == opamp_bits::CSR_VP_SEL_Pos &&
    opamp_bit::CSR_VM_SEL_Pos == opamp_bits::CSR_VM_SEL_Pos && opamp_bit::CSR_OPAHSM == opamp_bits::CSR_OPAHSM &&
    opamp_bit::CSR_OPAINTOEN == opamp_bits::CSR_OPAINTOEN && opamp_bit::CSR_PGA_GAIN_Pos == opamp_bits::CSR_PGA_GAIN_Pos &&
    opamp_bit::CSR_LOCK == opamp_bits::CSR_LOCK);

inline consteval uint32_t get_opamp_csr(const OPAMPAmplifierInit& c) {
    using namespace opamp_bit;
    if (static_cast<uint32_t>(c.gain) > opamp_max_gain) opamp_gain_not_available();
    return (c.enable == OPAMPEnable::ENABLED ? CSR_OPAEN : 0) |
        static_cast<uint32_t>(c.input) << CSR_VP_SEL_Pos |
        static_cast<uint32_t>(c.mode) << CSR_VM_SEL_Pos |
        (c.high_speed == OPAMPEnable::ENABLED ? CSR_OPAHSM : 0) |
        (c.internal_output == OPAMPEnable::ENABLED ? CSR_OPAINTOEN : 0) |
        static_cast<uint32_t>(c.gain) << CSR_PGA_GAIN_Pos |
        static_cast<uint32_t>(c.pga_input) << CSR_PGA_INPUT_Pos |
        (c.lock == OPAMPEnable::ENABLED ? CSR_LOCK : 0);
}

// One write configures and enables an amplifier; its output settles within
// the wake up time (tens of microseconds). Only amplifiers that differ from
// reset are written.
template<uint32_t csr, int opamp>
inline void init_opamp_csr() {
    if constexpr (csr != 0) opamp_csr<opamp>() = csr;
}

template<const OPAMPInit h>
inline void init_opamp() {
    init_opamp_csr<get_opamp_csr(h.opamp1), 1>();
    init_opamp_csr<get_opamp_csr(h.opamp2), 2>();
    init_opamp_csr<get_opamp_csr(h.opamp3), 3>();
    init_opamp_csr<get_opamp_csr(h.opamp4), 4>();
    init_opamp_csr<get_opamp_csr(h.opamp5), 5>();
    init_opamp_csr<get_opamp_csr(h.opamp6), 6>();
}

template<OPAMPInit opamp_init>
inline void device_opamp_config() {
    init_opamp<opamp_init>();
}

// GlobalTask of the calibration of an amplifier: instance opamp_task_id +
// opamp. The frame is static and the first call hangs if it does not fit:
// gcc needs 100 bytes, clang 56.
inline constexpr int opamp_task_id = 110;
inline constexpr int opamp_task_size = 128;

// Offset trim of an amplifier set up by init_opamp, in place of the factory
// trim: opamp_calibrate_async<3>(sched) is OPAMP3. In calibration mode both
// inputs are on an internal reference and CALOUT tells which way the output
// goes. Each input pair has a 5 bit trim: the N pair is trimmed with the
// reference at 0.9 VDDA, the P pair at 0.1 VDDA. A binary search finds the
// first trim value at which CALOUT differs from that at trim 0, with the
// settling time between steps (12 steps in all).
//
// The calibration needs the output on the VOUT pin: the amplifier does not
// work and drives VOUT to either rail meanwhile, also when its configuration
// uses the internal output. The result is false, and the factory trim stays,
// if a pair has no such trim value. A locked CSR ignores all of this.
template<int opamp>
[[gnu::cold]] inline GlobalTask<bool, opamp_task_id + opamp, opamp_task_size> opamp_calibrate_async(CycleScheduler& sched) {
    using namespace opamp_bit;
    constexpr uint32_t trim_count = 1u << CSR_TRIMOFFSET_Width;
    constexpr uint32_t trim_mask = (trim_count - 1) << CSR_TRIMOFFSETN_Pos | (trim_count - 1) << CSR_TRIMOFFSETP_Pos;
    volatile uint32_t& csr = opamp_csr<opamp>();
    const uint32_t functional = csr & ~(CSR_USERTRIM | trim_mask);
    const uint32_t calibration = (functional & CSR_OPAHSM) | CSR_OPAEN | CSR_USERTRIM | CSR_CALON;
    uint32_t trim = 0;
    bool found = true;
    for (int pair = 0; pair < 2; pair++) {
        const uint32_t select = (pair == 0 ? 3u : 1u) << CSR_CALSEL_Pos;
        const uint32_t pos = pair == 0 ? CSR_TRIMOFFSETN_Pos : CSR_TRIMOFFSETP_Pos;
        // CALOUT at low is that at trim 0, at high it differs. trim_count stands for not seen.
        uint32_t low = 0, high = trim_count, value = 0, at_zero = 0;
        while (true) {
            csr = calibration | select | value << pos;
            co_await sched.async_delay_us(opamp_trim_delay_us);
            const uint32_t out = csr & CSR_CALOUT;
            if (value == 0) at_zero = out;
            else if (out == at_zero) low = value;
            else high = value;
            if (high - low == 1) break;
            value = (low + high) / 2;
        }
        if (high == trim_count) found = false;
        trim |= high << pos;
    }
    csr = found ? functional | CSR_USERTRIM | trim : functional;
    co_return found;
}
}
