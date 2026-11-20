module;
#include <concepts>
#include <cstdint>
#include "../task.h"

export module drivers.drv8323;

// Compile-time configuration of a TI DRV8323S three phase gate driver with
// current sense amplifiers, the SPI variant. It is not part of the
// microcontroller: the access to it comes from an IO type of the
// configuration that uses it, see drv8323::IO. One drv8323::Init holds the
// five control registers by field; drv8323::config_async<init, IO>(sched)
// enables the driver, writes them and reads them back, as a coroutine: the
// wake up time and the frames are waited for on the scheduler, next to
// whatever else is starting.
//
// SPI: 16 bit frames, clock idle low, data sampled on the falling edge
// (SPIClockPhase::SECOND_EDGE), 10 MHz at most, chip select high for 400 ns
// between frames. The registers only respond while the enable pin is high.
export namespace drv8323 {

enum class Enable : bool {
    DISABLED,
    ENABLED
};

enum class PWMMode : uint8_t {          // driver control PWM_MODE
    PWM_6X,             // an input per switch: INHx and INLx
    PWM_3X,             // INHx is the phase's pwm, INLx its enable
    PWM_1X,
    INDEPENDENT,
};

enum class SourceCurrent : uint8_t {    // gate drive IDRIVEP_HS / IDRIVEP_LS: peak current into the gate
    MA_10, MA_30, MA_60, MA_80, MA_120, MA_140, MA_170, MA_190,
    MA_260, MA_330, MA_370, MA_440, MA_570, MA_680, MA_820, MA_1000,
};

enum class SinkCurrent : uint8_t {      // gate drive IDRIVEN_HS / IDRIVEN_LS: peak current out of the gate
    MA_20, MA_60, MA_120, MA_160, MA_240, MA_280, MA_340, MA_380,
    MA_520, MA_660, MA_740, MA_880, MA_1140, MA_1360, MA_1640, MA_2000,
};

enum class DriveTime : uint8_t {        // gate drive LS TDRIVE: how long the peak gate current is applied
    NS_500,
    NS_1000,
    NS_2000,
    NS_4000,
};

enum class DeadTime : uint8_t {         // OCP control DEAD_TIME, added by the driver
    NS_50,
    NS_100,
    NS_200,
    NS_400,
};

enum class OCPMode : uint8_t {          // OCP control OCP_MODE: what a VDS or sense overcurrent does
    LATCHED,            // outputs off until the fault is cleared
    RETRY,              // outputs off for the retry time
    REPORT,             // reported only
    DISABLED,
};

enum class OCPRetry : uint8_t {         // OCP control TRETRY
    MS_4,
    US_50,
};

enum class OCPDeglitch : uint8_t {      // OCP control OCP_DEG
    US_2,
    US_4,
    US_6,
    US_8,
};

enum class VDSLevel : uint8_t {         // OCP control VDS_LVL: the drain source voltage that counts as overcurrent
    V_0_06, V_0_13, V_0_2, V_0_26, V_0_31, V_0_45, V_0_53, V_0_6,
    V_0_68, V_0_75, V_0_94, V_1_13, V_1_3, V_1_5, V_1_7, V_1_88,
};

enum class CSAGain : uint8_t {          // CSA control CSA_GAIN, V/V
    X5,
    X10,
    X20,
    X40,
};

enum class SenseLevel : uint8_t {       // CSA control SEN_LVL: sense resistor overcurrent threshold
    V_0_25,
    V_0_5,
    V_0_75,
    V_1,
};

// Defaults are the register reset values
struct Init {
    // driver control, 0x02
    Enable disable_charge_pump_undervoltage = Enable::DISABLED;    // DIS_CPUV
    Enable disable_gate_drive_fault = Enable::DISABLED;            // DIS_GDF
    Enable report_overtemperature_warning = Enable::DISABLED;      // OTW_REP: on the fault pin
    PWMMode pwm_mode = PWMMode::PWM_6X;

    // gate drive HS, 0x03 and LS, 0x04
    SourceCurrent high_side_source_current = SourceCurrent::MA_1000;   // IDRIVEP_HS
    SinkCurrent high_side_sink_current = SinkCurrent::MA_2000;         // IDRIVEN_HS
    Enable cycle_by_cycle = Enable::ENABLED;                // CBC: an overcurrent in retry mode clears at the next pwm edge
    DriveTime drive_time = DriveTime::NS_4000;
    SourceCurrent low_side_source_current = SourceCurrent::MA_1000;    // IDRIVEP_LS
    SinkCurrent low_side_sink_current = SinkCurrent::MA_2000;          // IDRIVEN_LS

    // OCP control, 0x05
    OCPRetry ocp_retry = OCPRetry::MS_4;
    DeadTime dead_time = DeadTime::NS_100;
    OCPMode ocp_mode = OCPMode::RETRY;
    OCPDeglitch ocp_deglitch = OCPDeglitch::US_4;
    VDSLevel vds_level = VDSLevel::V_0_75;

    // CSA control, 0x06
    Enable sense_from_sh = Enable::DISABLED;                // CSA_FET: amplifier positive input on SHx instead of SPx
    Enable bidirectional = Enable::ENABLED;                 // VREF_DIV: output centered on VREF / 2
    Enable low_side_reference_sn = Enable::DISABLED;        // LS_REF: low side VDS measured to SNx instead of SPx
    CSAGain csa_gain = CSAGain::X20;
    Enable disable_sense_overcurrent = Enable::DISABLED;    // DIS_SEN
    SenseLevel sense_level = SenseLevel::V_1;
};

// Register addresses
inline constexpr uint8_t fault_status1_addr = 0, fault_status2_addr = 1, driver_control_addr = 2,
    gate_drive_hs_addr = 3, gate_drive_ls_addr = 4, ocp_control_addr = 5, csa_control_addr = 6;
inline constexpr int control_register_count = 5;
inline constexpr uint16_t data_mask = 0x7FF;                // 11 data bits per register
inline constexpr uint16_t gate_drive_unlock = 3u << 8;      // gate drive HS LOCK: 3 unlocks the registers, 6 locks them
inline constexpr uint16_t clear_fault = 1u << 0;            // driver control CLR_FLT, clears itself

// Value of control register n: 0 is the driver control register (0x02)
inline consteval uint16_t get_reg(const Init& c, int n) {
    constexpr auto bit = [](Enable enable, int pos) consteval { return static_cast<uint16_t>(enable == Enable::ENABLED ? 1u << pos : 0); };
    constexpr auto field = [](auto value, int pos) consteval { return static_cast<uint16_t>(static_cast<uint16_t>(value) << pos); };
    switch (n) {
        case 0: return bit(c.disable_charge_pump_undervoltage, 9) | bit(c.disable_gate_drive_fault, 8) |
            bit(c.report_overtemperature_warning, 7) | field(c.pwm_mode, 5);
        case 1: return gate_drive_unlock | field(c.high_side_source_current, 4) | field(c.high_side_sink_current, 0);
        case 2: return bit(c.cycle_by_cycle, 10) | field(c.drive_time, 8) |
            field(c.low_side_source_current, 4) | field(c.low_side_sink_current, 0);
        case 3: return field(c.ocp_retry, 10) | field(c.dead_time, 8) | field(c.ocp_mode, 6) |
            field(c.ocp_deglitch, 4) | field(c.vds_level, 0);
        default: return bit(c.sense_from_sh, 10) | bit(c.bidirectional, 9) | bit(c.low_side_reference_sn, 8) |
            field(c.csa_gain, 6) | bit(c.disable_sense_overcurrent, 5) | field(c.sense_level, 0);
    }
}
// the defaults above are the reset values of the datasheet
static_assert(get_reg(Init{}, 0) == 0x000 && get_reg(Init{}, 1) == 0x3FF && get_reg(Init{}, 2) == 0x7FF &&
    get_reg(Init{}, 3) == 0x159 && get_reg(Init{}, 4) == 0x283);

// Amplifier gain as a number, for the current scaling
inline constexpr float csa_gain(const Init& c) {
    return 5 << static_cast<int>(c.csa_gain);
}

// What a configuration supplies to reach its driver, as static functions:
//   transfer(word):              one 16 bit SPI frame with its chip select, waited for. Returns the word received.
//   transfer_async(sched, word): the same for a task: an awaitable of the word received, the wait is the scheduler's
//   enable(on):                  the enable pin
// The 400 ns of chip select high between frames are the IO's to keep.
template<typename T>
concept IO = requires(CycleScheduler& sched, uint16_t word, bool on) {
    { T::transfer(word) } -> std::convertible_to<uint16_t>;
    T::transfer_async(sched, word);
    T::enable(on);
};

// Frames: bit 15 read, bits 14..11 address, bits 10..0 data. The reply to a
// write is the register's previous value.
inline constexpr uint16_t read_frame(uint8_t address) {
    return 1u << 15 | address << 11;
}
inline constexpr uint16_t write_frame(uint8_t address, uint16_t value) {
    return address << 11 | (value & data_mask);
}
// One register, waiting for the frame: 16 bit times
template<IO T>
inline uint16_t read(uint8_t address) {
    return T::transfer(read_frame(address)) & data_mask;
}
template<IO T>
inline void write(uint8_t address, uint16_t value) {
    T::transfer(write_frame(address, value));
}

// Fault status 2 << 16 | fault status 1. Status 1 bit 10 is FAULT, the state
// of the fault pin; 0 is no fault.
template<IO T>
inline uint32_t status() {
    const uint32_t status1 = read<T>(fault_status1_addr);
    return read<T>(fault_status2_addr) << 16 | status1;
}

// Clears latched faults; the outputs follow the inputs again
template<Init init, IO T>
inline void clear_faults() {
    write<T>(driver_control_addr, get_reg(init, 0) | clear_fault);
}

// Into the low power state: all gates off, registers back at reset, no SPI.
// The way back is config_async, which enables the driver and writes the
// registers again: enabled alone it would run on their reset values.
template<IO T>
inline void disable() {
    T::enable(false);
}

// GlobalTask of the configuration: one driver. The frame is static and the
// call hangs if it does not fit.
inline constexpr int task_id = 110;
inline constexpr int task_size = 96;

// Enables the driver and writes the control registers. The result is 0 when
// every register reads back as written, else a bit per register that does not
// (bit 0: driver control, ... bit 4: CSA control): no driver, no supply, or
// a broken SPI connection. The driver is left enabled either way: disabling
// it on a failure is the caller's decision. The inputs should hold the
// gates off while this runs, and nothing else may use the IO.
template<Init init, IO T>
[[gnu::cold]] inline GlobalTask<uint32_t, task_id, task_size> config_async(CycleScheduler& sched) {
    constexpr uint32_t wake_up_us = 10'000;     // 1 ms in the datasheet, the obot_g474 waits 10
    static constexpr uint16_t regs[control_register_count] = {get_reg(init, 0), get_reg(init, 1),
        get_reg(init, 2), get_reg(init, 3), get_reg(init, 4)};
    T::enable(true);
    co_await sched.async_delay_us(wake_up_us);
    uint32_t errors = 0;
    for (int n = 0; n < control_register_count; n++) {
        co_await T::transfer_async(sched, write_frame(driver_control_addr + n, regs[n]));
        const uint16_t reply = co_await T::transfer_async(sched, read_frame(driver_control_addr + n));
        if ((reply & data_mask) != regs[n]) errors |= 1u << n;
    }
    co_return errors;
}
}
