module;
#include <cstddef>
#include <cstdint>
#include "dac_regs.h"

export module device.dac_init;

export {
// Compile-time DAC configuration, shared between devices: the registers used
// here are the same on the stm32g4 and stm32g5; the device's dac_regs.h
// supplies the addresses, the channels of each DAC and where their outputs
// go. One DACInit holds all of them. This sets up constant outputs that
// dac_write can change: no triggers, waveforms, DMA or sample and hold.

enum class DACEnable : bool {
    DISABLED,
    ENABLED
};

enum class DACOutput : uint8_t {    // MCR MODEx, normal mode
    PIN_BUFFERED,                   // output pin, through the buffer
    PIN_BUFFERED_INTERNAL,          // output pin through the buffer, and on chip peripherals
    PIN,                            // output pin, no buffer
    INTERNAL,                       // on chip peripherals (opamp, comparator), no buffer
};

// Output voltage = VREF+ * value / dac_range
inline constexpr uint32_t dac_range = 4096;

// Defaults are the register reset values
struct DACChannelInit {
    DACEnable enable = DACEnable::DISABLED;         // CR ENx
    DACOutput output = DACOutput::PIN_BUFFERED;     // MCR MODEx
    uint16_t value = 0;                             // DHR12Rx
};

struct DACConverterInit {
    DACChannelInit ch1;
    DACChannelInit ch2;                             // stm32g4 only, not on dac2
};

struct DACInit {
    DACConverterInit dac1, dac2;
    DACConverterInit dac3, dac4;                               // stm32g4 only
    DACConverterInit dac5, dac6, dac7, dac8, dac9, dac10;      // stm32g5 only
};

// Registers common to the stm32g4 and stm32g5, stm32g4 names. The channel 2
// registers are reserved where a DAC has one channel.
struct DACRegs {
    volatile uint32_t CR, SWTRIGR, DHR12R1, DHR12L1, DHR8R1, DHR12R2, DHR12L2, DHR8R2,
        DHR12RD, DHR12LD, DHR8RD, DOR1, DOR2, SR, CCR, MCR;
};
static_assert(offsetof(DACRegs, CR) == dac_offsets::CR && offsetof(DACRegs, DHR12R1) == dac_offsets::DHR12R1 &&
    offsetof(DACRegs, DHR12R2) == dac_offsets::DHR12R2 && offsetof(DACRegs, DOR1) == dac_offsets::DOR1 &&
    offsetof(DACRegs, MCR) == dac_offsets::MCR);

// Channels of a DAC, 0 where the device has no such DAC
inline consteval int dac_channel_count(int dac) {
    return dac >= 1 && dac <= dac_count ? dac_channels[dac - 1] : 0;
}

// dac_regs<3>() is DAC3
template<int dac>
inline DACRegs& dac_regs() {
    static_assert(dac_channel_count(dac) != 0, "no such dac on this device");
    return *reinterpret_cast<DACRegs*>(dac_base[dac - 1]);
}

// Bit positions that are the same on both devices, checked against the
// device header. The channel 2 fields of CR and MCR sit 16 bits above the
// channel 1 fields.
namespace dac_bit {
inline constexpr uint32_t CR_EN1 = 1u << 0, MCR_MODE1_Pos = 0, CHANNEL2_Pos = 16;
}
static_assert(dac_bit::CR_EN1 == dac_bits::CR_EN1 && dac_bit::MCR_MODE1_Pos == dac_bits::MCR_MODE1_Pos &&
    dac_bit::CHANNEL2_Pos == dac_bits::CHANNEL2_Pos);

inline consteval bool dac_channel_used(const DACChannelInit& c) {
    return c.enable == DACEnable::ENABLED || c.output != DACOutput::PIN_BUFFERED || c.value != 0;
}
inline consteval bool dac_used(const DACConverterInit& c) {
    return dac_channel_used(c.ch1) || dac_channel_used(c.ch2);
}
inline consteval bool dac_channels_exist(const DACConverterInit& c, int dac) {
    return (!dac_channel_used(c.ch1) || dac_channel_count(dac) >= 1) && (!dac_channel_used(c.ch2) || dac_channel_count(dac) >= 2);
}
// An enabled channel of a DAC without output pins can only be INTERNAL
inline consteval bool dac_outputs_exist(const DACConverterInit& c, int dac) {
    if (dac_channel_count(dac) == 0 || dac_has_pins[dac - 1]) return true;
    return (c.ch1.enable == DACEnable::DISABLED || c.ch1.output == DACOutput::INTERNAL) &&
        (c.ch2.enable == DACEnable::DISABLED || c.ch2.output == DACOutput::INTERNAL);
}

// MCR HFSEL: the interface timing follows the DAC clock frequency
inline consteval uint32_t get_dac_hfsel() {
    return dac_clock_hz > 160e6f ? 2 : dac_clock_hz > 80e6f ? 1 : 0;
}

inline consteval uint32_t get_dac_mcr(const DACConverterInit& c) {
    using namespace dac_bit;
    bool enabled = c.ch1.enable == DACEnable::ENABLED || c.ch2.enable == DACEnable::ENABLED;
    return static_cast<uint32_t>(c.ch1.output) << MCR_MODE1_Pos |
        static_cast<uint32_t>(c.ch2.output) << (MCR_MODE1_Pos + CHANNEL2_Pos) |
        (enabled ? get_dac_hfsel() << dac_bits::MCR_HFSEL_Pos : 0);
}

inline consteval uint32_t get_dac_cr(const DACConverterInit& c) {
    using namespace dac_bit;
    return (c.ch1.enable == DACEnable::ENABLED ? CR_EN1 : 0) |
        (c.ch2.enable == DACEnable::ENABLED ? CR_EN1 << CHANNEL2_Pos : 0);
}

// The mode can only be written while the channels are disabled, and the
// value is written before the enable so that the output starts there. It
// settles within the wake up time (under 10 us). Only registers that differ
// from reset are written. The DAC's clock (RCC enable) must be running.
template<const DACConverterInit c, int dac>
inline void init_dac_converter() {
    static_assert(dac_channels_exist(c, dac), "no such dac channel on this device");
    static_assert(dac_outputs_exist(c, dac), "this dac has no output pins: use DACOutput::INTERNAL");
    static_assert(c.ch1.value < dac_range && c.ch2.value < dac_range, "dac values are 12 bit");
    if constexpr (dac_used(c)) {
        DACRegs& regs = dac_regs<dac>();
        if constexpr (get_dac_mcr(c) != 0) regs.MCR = get_dac_mcr(c);
        if constexpr (c.ch1.value != 0) regs.DHR12R1 = c.ch1.value;
        if constexpr (c.ch2.value != 0) regs.DHR12R2 = c.ch2.value;
        if constexpr (get_dac_cr(c) != 0) regs.CR = get_dac_cr(c);
    }
}

template<const DACInit h>
inline void init_dac() {
    init_dac_converter<h.dac1, 1>();
    init_dac_converter<h.dac2, 2>();
    init_dac_converter<h.dac3, 3>();
    init_dac_converter<h.dac4, 4>();
    init_dac_converter<h.dac5, 5>();
    init_dac_converter<h.dac6, 6>();
    init_dac_converter<h.dac7, 7>();
    init_dac_converter<h.dac8, 8>();
    init_dac_converter<h.dac9, 9>();
    init_dac_converter<h.dac10, 10>();
}

template<DACInit dac_init>
inline void device_dac_config() {
    init_dac<dac_init>();
}

// New output value of a channel: dac_write<3, 2>(value) is DAC3 channel 2
template<int dac, int channel>
inline void dac_write(uint32_t value) {
    static_assert(channel >= 1 && channel <= dac_channel_count(dac), "no such dac channel on this device");
    if constexpr (channel == 1) dac_regs<dac>().DHR12R1 = value;
    else dac_regs<dac>().DHR12R2 = value;
}

// Value a channel is putting out
template<int dac, int channel>
inline uint32_t dac_read() {
    static_assert(channel >= 1 && channel <= dac_channel_count(dac), "no such dac channel on this device");
    if constexpr (channel == 1) return dac_regs<dac>().DOR1;
    else return dac_regs<dac>().DOR2;
}
}
