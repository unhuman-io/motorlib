module;
#include <cstddef>
#include <cstdint>
#include "adc_regs.h"
#include "../../task.h"

export module device.adc_init;

// device specific encodings, from the device's adc_regs.h
export using ::ADCClock;
export using ::ADCSampleTime;
export using ::ADCTrigger;
export using ::ADCInjectedTrigger;

export {
// Compile-time ADC configuration, shared between devices. The stm32g4 and
// stm32g5 ADCs have the same control, sequence and data registers; what
// differs (clocking, trigger and sample time encodings, the g5 channel
// preselection) comes from the device's adc_regs.h. Register values are built
// at compile time. One ADCInit holds every converter:
// device_adc_config<init>() powers up, calibrates, configures and arms the
// enabled ones, adc_read<3>() returns the last ADC3 result.

enum class ADCEnable : bool {
    DISABLED,
    ENABLED
};

enum class ADCResolution : uint8_t {   // CFGR RES
    BITS_12,
    BITS_10,
    BITS_8,
    BITS_6,
};

enum class ADCTriggerEdge : uint8_t {   // CFGR EXTEN / JSQR JEXTEN
    SOFTWARE,                           // no hardware trigger: adc_start()
    RISING,
    FALLING,
    BOTH,
};

// Defaults are the register reset values
struct ADCConverterInit {
    ADCEnable enable = ADCEnable::DISABLED;                         // CR ADEN, after the power up and calibration

    // common register of the converter's group (ADC1/2, ADC3/4/5, ...): the
    // enabled converters of a group must ask for the same clock
    ADCClock clock = ADCClock{};                                    // CCR CKMODE, where the device has it
    ADCEnable vrefint = ADCEnable::DISABLED;                        // CCR VREFEN
    ADCEnable temperature_sensor = ADCEnable::DISABLED;             // CCR VSENSESEL / TSEN

    // regular group, result in DR
    ADCResolution resolution = ADCResolution::BITS_12;
    ADCTrigger trigger = ADCTrigger{};                              // CFGR EXTSEL
    ADCTriggerEdge trigger_edge = ADCTriggerEdge::SOFTWARE;         // CFGR EXTEN
    ADCEnable continuous = ADCEnable::DISABLED;                     // CFGR CONT
    ADCEnable overwrite = ADCEnable::DISABLED;                      // CFGR OVRMOD: an unread DR is replaced by the newer conversion instead of kept
    ADCEnable end_of_conversion_interrupt = ADCEnable::DISABLED;    // IER EOCIE: interrupt after every regular conversion, cleared by reading DR
    uint8_t sequence_length = 1;                                    // SQR1 L + 1: conversions per trigger
    uint8_t sequence[16] = {};                                      // SQRx: channel numbers in conversion order

    // injected group, results in JDR1..4. Unused while the length is 0.
    ADCInjectedTrigger injected_trigger = ADCInjectedTrigger{};     // JSQR JEXTSEL
    ADCTriggerEdge injected_trigger_edge = ADCTriggerEdge::SOFTWARE;   // JSQR JEXTEN
    uint8_t injected_sequence_length = 0;                           // JSQR JL + 1
    uint8_t injected_sequence[4] = {};                              // JSQR JSQ1..4

    ADCSampleTime sample_time[adc_channel_count] = {};              // SMPRx, indexed by channel number
};

struct ADCInit {
    ADCConverterInit adc1, adc2, adc3;
    ADCConverterInit adc4, adc5;                                    // stm32g4 only
};

// Registers common to the stm32g4 and stm32g5, stm32g4 names
struct ADCRegs {
    volatile uint32_t ISR, IER, CR, CFGR, CFGR2, SMPR1, SMPR2,
        PCSEL,   // channel preselection, only where adc_channel_preselect
        RESERVED1[4], SQR1, SQR2, SQR3, SQR4, DR, RESERVED2[2], JSQR, RESERVED3[12],
        JDR1, JDR2, JDR3, JDR4;
};
static_assert(offsetof(ADCRegs, CFGR) == adc_offsets::CFGR && offsetof(ADCRegs, SMPR2) == adc_offsets::SMPR2 &&
    offsetof(ADCRegs, SQR1) == adc_offsets::SQR1 && offsetof(ADCRegs, DR) == adc_offsets::DR &&
    offsetof(ADCRegs, JSQR) == adc_offsets::JSQR && offsetof(ADCRegs, JDR1) == adc_offsets::JDR1);

struct ADCCommonRegs {
    volatile uint32_t CSR, RESERVED, CCR, CDR;
};
static_assert(offsetof(ADCCommonRegs, CCR) == adc_offsets::CCR);

// adc_regs<3>() is ADC3
template<int adc>
inline ADCRegs& adc_regs() {
    static_assert(adc >= 1 && adc <= adc_count, "no such adc on this device");
    return *reinterpret_cast<ADCRegs*>(adc_base[adc - 1]);
}
template<int group>
inline ADCCommonRegs& adc_common_regs() {
    return *reinterpret_cast<ADCCommonRegs*>(adc_group_base[group]);
}

// Bit positions that are the same on both devices, checked against the device header
namespace adc_bit {
inline constexpr uint32_t ISR_ADRDY = 1u << 0, IER_EOCIE = 1u << 2;
inline constexpr uint32_t CR_ADEN = 1u << 0, CR_ADDIS = 1u << 1, CR_ADSTART = 1u << 2, CR_JADSTART = 1u << 3,
    CR_STOP_Shift = 2,   // ADSTP and JADSTP sit this far above ADSTART and JADSTART
    CR_ADVREGEN = 1u << 28, CR_ADCAL = 1u << 31;
inline constexpr uint32_t CFGR_EXTSEL_Pos = 5, CFGR_EXTEN_Pos = 10, CFGR_OVRMOD = 1u << 12, CFGR_CONT = 1u << 13, CFGR_JQDIS = 1u << 31;
inline constexpr uint32_t SQR1_SQ1_Pos = 6, SQR_SQ_Width = 6, SMPR_SMP_Width = 3;
inline constexpr uint32_t JSQR_JEXTSEL_Pos = 2, JSQR_JEXTEN_Pos = 7, JSQR_JSQ1_Pos = 9, JSQR_JSQ_Width = 6;
inline constexpr uint32_t CCR_VREFEN = 1u << 22, CCR_TSEN = 1u << 23;
}
static_assert(adc_bit::ISR_ADRDY == adc_bits::ISR_ADRDY && adc_bit::IER_EOCIE == adc_bits::IER_EOCIE && adc_bit::CR_ADSTART == adc_bits::CR_ADSTART &&
    adc_bit::CR_JADSTART == adc_bits::CR_JADSTART && adc_bit::CR_ADVREGEN == adc_bits::CR_ADVREGEN &&
    adc_bit::CR_ADCAL == adc_bits::CR_ADCAL && adc_bit::CR_ADDIS == adc_bits::CR_ADDIS &&
    adc_bit::CR_ADSTART << adc_bit::CR_STOP_Shift == adc_bits::CR_ADSTP && adc_bit::CR_JADSTART << adc_bit::CR_STOP_Shift == adc_bits::CR_JADSTP);
static_assert(adc_bit::CFGR_EXTSEL_Pos == adc_bits::CFGR_EXTSEL_Pos && adc_bit::CFGR_EXTEN_Pos == adc_bits::CFGR_EXTEN_Pos &&
    adc_bit::CFGR_OVRMOD == adc_bits::CFGR_OVRMOD && adc_bit::CFGR_CONT == adc_bits::CFGR_CONT &&
    adc_bit::CFGR_JQDIS == adc_bits::CFGR_JQDIS);
static_assert(adc_bit::SQR1_SQ1_Pos == adc_bits::SQR1_SQ1_Pos && adc_bit::SQR1_SQ1_Pos + adc_bit::SQR_SQ_Width == adc_bits::SQR1_SQ2_Pos &&
    adc_bit::SMPR_SMP_Width == adc_bits::SMPR1_SMP1_Pos);
static_assert(adc_bit::JSQR_JEXTSEL_Pos == adc_bits::JSQR_JEXTSEL_Pos && adc_bit::JSQR_JEXTEN_Pos == adc_bits::JSQR_JEXTEN_Pos &&
    adc_bit::JSQR_JSQ1_Pos == adc_bits::JSQR_JSQ1_Pos && adc_bit::JSQR_JSQ1_Pos + adc_bit::JSQR_JSQ_Width == adc_bits::JSQR_JSQ2_Pos);
static_assert(adc_bit::CCR_VREFEN == adc_bits::CCR_VREFEN && adc_bit::CCR_TSEN == adc_bits::CCR_TSEN);

// Common register of a group, from the enabled converters in it
template<const ADCInit h>
inline consteval uint32_t get_adc_ccr(int group) {
    using namespace adc_bit;
    const ADCConverterInit converters[] = {h.adc1, h.adc2, h.adc3, h.adc4, h.adc5};
    uint32_t v = 0;
    for (int i = 0; i < adc_count; i++) {
        const ADCConverterInit& c = converters[i];
        if (adc_group[i] != group || c.enable != ADCEnable::ENABLED) continue;
        v |= adc_ccr_clock(c.clock) |
            (c.vrefint == ADCEnable::ENABLED ? CCR_VREFEN : 0) |
            (c.temperature_sensor == ADCEnable::ENABLED ? CCR_TSEN : 0);
    }
    return v;
}

template<const ADCInit h>
inline consteval bool adc_group_clocks_agree() {
    const ADCConverterInit converters[] = {h.adc1, h.adc2, h.adc3, h.adc4, h.adc5};
    for (int i = 0; i < adc_count; i++) {
        for (int j = i + 1; j < adc_count; j++) {
            if (adc_group[i] == adc_group[j] && converters[i].enable == ADCEnable::ENABLED &&
                converters[j].enable == ADCEnable::ENABLED && converters[i].clock != converters[j].clock) return false;
        }
    }
    return true;
}

// The injected queue stays disabled (JQDIS, the reset state): JSQR is used directly
template<const ADCConverterInit c>
inline consteval uint32_t get_adc_cfgr() {
    using namespace adc_bit;
    return static_cast<uint32_t>(c.resolution) << adc_bits::CFGR_RES_Pos |
        static_cast<uint32_t>(c.trigger) << CFGR_EXTSEL_Pos |
        static_cast<uint32_t>(c.trigger_edge) << CFGR_EXTEN_Pos |
        (c.overwrite == ADCEnable::ENABLED ? CFGR_OVRMOD : 0) |
        (c.continuous == ADCEnable::ENABLED ? CFGR_CONT : 0) |
        CFGR_JQDIS;
}

// SMPR1 (n = 0) holds channels 0..9, SMPR2 (n = 1) the rest
template<const ADCConverterInit c>
inline consteval uint32_t get_adc_smpr(int n) {
    uint32_t v = 0;
    for (int ch = 10 * n; ch < 10 * (n + 1) && ch < adc_channel_count; ch++) {
        v |= static_cast<uint32_t>(c.sample_time[ch]) << adc_bit::SMPR_SMP_Width * (ch - 10 * n);
    }
    return v;
}

// SQR1 (n = 0) holds the length and ranks 1..4, SQR2..4 five ranks each
template<const ADCConverterInit c>
inline consteval uint32_t get_adc_sqr(int n) {
    using namespace adc_bit;
    uint32_t v = n == 0 ? c.sequence_length - 1 : 0;
    for (int rank = 0; rank < c.sequence_length; rank++) {
        const int reg = rank < 4 ? 0 : (rank - 4) / 5 + 1;
        const int field = rank < 4 ? rank + 1 : (rank - 4) % 5;
        if (reg == n) v |= static_cast<uint32_t>(c.sequence[rank]) << SQR_SQ_Width * field;
    }
    return v;
}

template<const ADCConverterInit c>
inline consteval uint32_t get_adc_jsqr() {
    using namespace adc_bit;
    uint32_t v = static_cast<uint32_t>(c.injected_sequence_length - 1) |
        static_cast<uint32_t>(c.injected_trigger) << JSQR_JEXTSEL_Pos |
        static_cast<uint32_t>(c.injected_trigger_edge) << JSQR_JEXTEN_Pos;
    for (int rank = 0; rank < c.injected_sequence_length; rank++) {
        v |= static_cast<uint32_t>(c.injected_sequence[rank]) << (JSQR_JSQ1_Pos + JSQR_JSQ_Width * rank);
    }
    return v;
}

// One bit per converted channel, for the devices that need them preselected
template<const ADCConverterInit c>
inline consteval uint32_t get_adc_pcsel() {
    uint32_t v = 0;
    for (int rank = 0; rank < c.sequence_length; rank++) v |= 1u << c.sequence[rank];
    for (int rank = 0; rank < c.injected_sequence_length; rank++) v |= 1u << c.injected_sequence[rank];
    return v;
}

template<const ADCConverterInit c>
inline consteval bool adc_channels_valid() {
    for (int rank = 0; rank < c.sequence_length; rank++) if (c.sequence[rank] >= adc_channel_count) return false;
    for (int rank = 0; rank < c.injected_sequence_length; rank++) if (c.injected_sequence[rank] >= adc_channel_count) return false;
    return true;
}

// Hardware triggered and continuous groups are started by the init, software
// triggered ones by adc_start()
template<const ADCConverterInit c>
inline consteval uint32_t get_adc_start() {
    using namespace adc_bit;
    return (c.trigger_edge != ADCTriggerEdge::SOFTWARE || c.continuous == ADCEnable::ENABLED ? CR_ADSTART : 0) |
        (c.injected_sequence_length != 0 && c.injected_trigger_edge != ADCTriggerEdge::SOFTWARE ? CR_JADSTART : 0);
}

// At least us microseconds: every pass of the loop takes more than one cpu cycle
inline void adc_delay_us(uint32_t us) {
    for (uint32_t i = static_cast<uint32_t>(CPU_FREQUENCY_HZ / 1e6f) * us; i != 0; i--) __asm__ volatile("");
}

template<const ADCConverterInit c, int adc>
inline void init_adc_converter() {
    using namespace adc_bit;
    static_assert(c.sequence_length >= 1 && c.sequence_length <= 16, "the regular sequence is 1 to 16 conversions");
    static_assert(c.injected_sequence_length <= 4, "the injected sequence is up to 4 conversions");
    static_assert(adc_channels_valid<c>(), "channel number out of range");
    auto& regs = adc_regs<adc>();

    regs.CR = 0;                          // out of deep power down, the reset state
    regs.CR = CR_ADVREGEN;
    adc_delay_us(adc_regulator_startup_us);
    regs.CR = CR_ADVREGEN | CR_ADEN;
    while (!(regs.ISR & ISR_ADRDY));

    regs.CFGR = get_adc_cfgr<c>();
    if constexpr (c.end_of_conversion_interrupt == ADCEnable::ENABLED) regs.IER = IER_EOCIE;
    if constexpr (get_adc_smpr<c>(0) != 0) regs.SMPR1 = get_adc_smpr<c>(0);
    if constexpr (get_adc_smpr<c>(1) != 0) regs.SMPR2 = get_adc_smpr<c>(1);
    if constexpr (adc_channel_preselect) regs.PCSEL = get_adc_pcsel<c>();
    if constexpr (get_adc_sqr<c>(0) != 0) regs.SQR1 = get_adc_sqr<c>(0);
    if constexpr (get_adc_sqr<c>(1) != 0) regs.SQR2 = get_adc_sqr<c>(1);
    if constexpr (get_adc_sqr<c>(2) != 0) regs.SQR3 = get_adc_sqr<c>(2);
    if constexpr (get_adc_sqr<c>(3) != 0) regs.SQR4 = get_adc_sqr<c>(3);
    if constexpr (c.injected_sequence_length != 0) regs.JSQR = get_adc_jsqr<c>();
    if constexpr (get_adc_start<c>() != 0) regs.CR = CR_ADVREGEN | get_adc_start<c>();
}

// The converters' clocks (RCC enable, and the kernel clock where the device
// selects it there) must be running: the enable waits on the ADC. The
// converters are not calibrated here: adc_calibrate_async does that.
template<const ADCInit h>
inline void init_adc() {
    static_assert(adc_count >= 5 || (h.adc4.enable == ADCEnable::DISABLED && h.adc5.enable == ADCEnable::DISABLED), "no adc4 or adc5 on this device");
    static_assert(adc_group_clocks_agree<h>(), "converters sharing a common register need the same clock");
    if constexpr (get_adc_ccr<h>(0) != 0) adc_common_regs<0>().CCR = get_adc_ccr<h>(0);
    if constexpr (get_adc_ccr<h>(1) != 0) adc_common_regs<1>().CCR = get_adc_ccr<h>(1);
    if constexpr (h.adc1.enable == ADCEnable::ENABLED) init_adc_converter<h.adc1, 1>();
    if constexpr (h.adc2.enable == ADCEnable::ENABLED) init_adc_converter<h.adc2, 2>();
    if constexpr (h.adc3.enable == ADCEnable::ENABLED) init_adc_converter<h.adc3, 3>();
    if constexpr (h.adc4.enable == ADCEnable::ENABLED) init_adc_converter<h.adc4, 4>();
    if constexpr (h.adc5.enable == ADCEnable::ENABLED) init_adc_converter<h.adc5, 5>();
}

template<ADCInit adc_init>
inline void device_adc_config() {
    init_adc<adc_init>();
}

// GlobalTask of the calibration of a converter: instance adc_task_id + adc. The
// frame is static and the first call hangs if it does not fit: gcc needs 56
// bytes, clang 40.
inline constexpr int adc_task_id = 100;
inline constexpr int adc_task_size = 80;

// Offset calibration of a converter set up by init_adc, single ended inputs:
// adc_calibrate_async<3>(sched) is ADC3. The calibration only runs on a
// disabled converter, so one that is enabled or converting is stopped and
// disabled first and put back afterwards. Its results are stale meanwhile.
template<int adc>
[[gnu::cold]] inline GlobalTask<void, adc_task_id + adc, adc_task_size> adc_calibrate_async(CycleScheduler& sched) {
    using namespace adc_bit;
    ADCRegs& regs = adc_regs<adc>();
    const uint32_t enabled = regs.CR & CR_ADEN;
    const uint32_t started = regs.CR & (CR_ADSTART | CR_JADSTART);
    // Stop, disable, calibrate: the converter clears each command bit when it is done
    for (int step = 0; step < 3; step++) {
        const uint32_t command = step == 0 ? started << CR_STOP_Shift : step == 1 ? (enabled != 0 ? CR_ADDIS : 0) : CR_ADCAL;
        if (command == 0) continue;
        regs.CR = CR_ADVREGEN | command;
        while ((regs.CR & command) != 0) co_await sched.yield();
    }
    if (enabled != 0) {
        regs.ISR = ISR_ADRDY;                   // clear the flag of the previous enable
        do {
            regs.CR = CR_ADVREGEN | CR_ADEN;    // ignored for 4 adc clocks after the calibration: repeat until it takes
            co_await sched.yield();
        } while ((regs.ISR & ISR_ADRDY) == 0);
        if (started != 0) regs.CR = CR_ADVREGEN | started;
    }
}

// Last regular conversion of a converter: adc_read<3>() is ADC3
template<int adc>
inline uint32_t adc_read() {
    return adc_regs<adc>().DR;
}

// Last injected conversion of a rank, 0 = JDR1
template<int adc>
inline uint32_t adc_read_injected(int rank) {
    return (&adc_regs<adc>().JDR1)[rank];
}

// Software trigger of the regular / injected group. The start bits are set
// only: writing the others as 0 leaves them alone.
template<int adc>
inline void adc_start() {
    adc_regs<adc>().CR = adc_bit::CR_ADVREGEN | adc_bit::CR_ADSTART;
}
template<int adc>
inline void adc_start_injected() {
    adc_regs<adc>().CR = adc_bit::CR_ADVREGEN | adc_bit::CR_JADSTART;
}
}
