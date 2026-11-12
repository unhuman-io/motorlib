module;
#include <cstdint>
#include "opamp_regs.h"

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
// The factory offset trim is used: USERTRIM stays 0.
namespace opamp_bit {
inline constexpr uint32_t CSR_OPAEN = 1u << 0, CSR_VP_SEL_Pos = 2, CSR_VM_SEL_Pos = 5, CSR_OPAHSM = 1u << 7,
    CSR_OPAINTOEN = 1u << 8, CSR_PGA_GAIN_Pos = 14, CSR_PGA_INPUT_Pos = 17, CSR_LOCK = 1u << 31;
}
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
}
