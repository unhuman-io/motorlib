module;
#include <concepts>
#include <cstdint>
#include "../../task.h"

export module device.drv8323_init;

export {
// Compile-time configuration of a TI DRV8323S three phase gate driver with
// current sense amplifiers, the SPI variant. It is not part of the
// microcontroller: the access to it comes from an IO type of the
// configuration that uses it, see DRV8323IO. One DRV8323Init holds the five
// control registers by field; drv8323_config_async<init, IO>(sched) enables
// the driver, writes them and reads them back, as a coroutine: the wake up
// time and the frames are waited for on the scheduler, next to whatever else
// is starting.
//
// SPI: 16 bit frames, clock idle low, data sampled on the falling edge
// (SPIClockPhase::SECOND_EDGE), 10 MHz at most, chip select high for 400 ns
// between frames. The registers only respond while the enable pin is high.

enum class DRV8323Enable : bool {
    DISABLED,
    ENABLED
};

enum class DRV8323PWMMode : uint8_t {       // driver control PWM_MODE
    PWM_6X,             // an input per switch: INHx and INLx
    PWM_3X,             // INHx is the phase's pwm, INLx its enable
    PWM_1X,
    INDEPENDENT,
};

enum class DRV8323DriveTime : uint8_t {     // gate drive LS TDRIVE: how long the peak gate current is applied
    NS_500,
    NS_1000,
    NS_2000,
    NS_4000,
};

enum class DRV8323DeadTime : uint8_t {      // OCP control DEAD_TIME, added by the driver
    NS_50,
    NS_100,
    NS_200,
    NS_400,
};

enum class DRV8323OCPMode : uint8_t {       // OCP control OCP_MODE: what a VDS or sense overcurrent does
    LATCHED,            // outputs off until the fault is cleared
    RETRY,              // outputs off for the retry time
    REPORT,             // reported only
    DISABLED,
};

enum class DRV8323OCPRetry : uint8_t {      // OCP control TRETRY
    MS_4,
    US_50,
};

enum class DRV8323OCPDeglitch : uint8_t {   // OCP control OCP_DEG
    US_2,
    US_4,
    US_6,
    US_8,
};

enum class DRV8323CSAGain : uint8_t {       // CSA control CSA_GAIN, V/V
    X5,
    X10,
    X20,
    X40,
};

enum class DRV8323SenseLevel : uint8_t {    // CSA control SEN_LVL: sense resistor overcurrent threshold
    V_0_25,
    V_0_5,
    V_0_75,
    V_1,
};

// Defaults are the register reset values
struct DRV8323Init {
    // driver control, 0x02
    DRV8323Enable disable_charge_pump_undervoltage = DRV8323Enable::DISABLED;  // DIS_CPUV
    DRV8323Enable disable_gate_drive_fault = DRV8323Enable::DISABLED;          // DIS_GDF
    DRV8323Enable report_overtemperature_warning = DRV8323Enable::DISABLED;    // OTW_REP: on the fault pin
    DRV8323PWMMode pwm_mode = DRV8323PWMMode::PWM_6X;

    // gate drive HS, 0x03 and LS, 0x04. The currents are the datasheet's
    // IDRIVE codes: 0 is the weakest (10 mA source, 20 mA sink), 15 the
    // strongest (1 A source, 2 A sink).
    uint8_t high_side_source_current = 15;                  // IDRIVEP_HS
    uint8_t high_side_sink_current = 15;                    // IDRIVEN_HS
    DRV8323Enable cycle_by_cycle = DRV8323Enable::ENABLED;  // CBC: an overcurrent in retry mode clears at the next pwm edge
    DRV8323DriveTime drive_time = DRV8323DriveTime::NS_4000;
    uint8_t low_side_source_current = 15;                   // IDRIVEP_LS
    uint8_t low_side_sink_current = 15;                     // IDRIVEN_LS

    // OCP control, 0x05
    DRV8323OCPRetry ocp_retry = DRV8323OCPRetry::MS_4;
    DRV8323DeadTime dead_time = DRV8323DeadTime::NS_100;
    DRV8323OCPMode ocp_mode = DRV8323OCPMode::RETRY;
    DRV8323OCPDeglitch ocp_deglitch = DRV8323OCPDeglitch::US_4;
    // VDS_LVL code, the drain source voltage that counts as overcurrent:
    // 0..15 = 0.06, 0.13, 0.2, 0.26, 0.31, 0.45, 0.53, 0.6, 0.68, 0.75, 0.94, 1.13, 1.3, 1.5, 1.7, 1.88 V
    uint8_t vds_level = 9;

    // CSA control, 0x06
    DRV8323Enable sense_from_sh = DRV8323Enable::DISABLED;      // CSA_FET: amplifier positive input on SHx instead of SPx
    DRV8323Enable bidirectional = DRV8323Enable::ENABLED;       // VREF_DIV: output centered on VREF / 2
    DRV8323Enable low_side_reference_sn = DRV8323Enable::DISABLED;   // LS_REF: low side VDS measured to SNx instead of SPx
    DRV8323CSAGain csa_gain = DRV8323CSAGain::X20;
    DRV8323Enable disable_sense_overcurrent = DRV8323Enable::DISABLED;   // DIS_SEN
    DRV8323SenseLevel sense_level = DRV8323SenseLevel::V_1;
};

// Register addresses
inline constexpr uint8_t drv8323_fault_status1 = 0, drv8323_fault_status2 = 1, drv8323_driver_control = 2,
    drv8323_gate_drive_hs = 3, drv8323_gate_drive_ls = 4, drv8323_ocp_control = 5, drv8323_csa_control = 6;
inline constexpr int drv8323_control_register_count = 5;
inline constexpr uint16_t drv8323_data_mask = 0x7FF;        // 11 data bits per register
inline constexpr uint16_t drv8323_gate_drive_unlock = 3u << 8;   // gate drive HS LOCK: 3 unlocks the registers, 6 locks them
inline constexpr uint16_t drv8323_clear_fault = 1u << 0;    // driver control CLR_FLT, clears itself

void drv8323_field_out_of_range();   // not constexpr: calling it fails the constant evaluation

// Value of control register n: 0 is the driver control register (0x02)
inline consteval uint16_t get_drv8323_reg(const DRV8323Init& c, int n) {
    constexpr auto bit = [](DRV8323Enable enable, int pos) consteval { return static_cast<uint16_t>(enable == DRV8323Enable::ENABLED ? 1u << pos : 0); };
    if (c.high_side_source_current > 15 || c.high_side_sink_current > 15 || c.low_side_source_current > 15 ||
        c.low_side_sink_current > 15 || c.vds_level > 15) drv8323_field_out_of_range();
    switch (n) {
        case 0: return bit(c.disable_charge_pump_undervoltage, 9) | bit(c.disable_gate_drive_fault, 8) |
            bit(c.report_overtemperature_warning, 7) | static_cast<uint16_t>(c.pwm_mode) << 5;
        case 1: return drv8323_gate_drive_unlock | c.high_side_source_current << 4 | c.high_side_sink_current;
        case 2: return bit(c.cycle_by_cycle, 10) | static_cast<uint16_t>(c.drive_time) << 8 |
            c.low_side_source_current << 4 | c.low_side_sink_current;
        case 3: return static_cast<uint16_t>(c.ocp_retry) << 10 | static_cast<uint16_t>(c.dead_time) << 8 |
            static_cast<uint16_t>(c.ocp_mode) << 6 | static_cast<uint16_t>(c.ocp_deglitch) << 4 | c.vds_level;
        default: return bit(c.sense_from_sh, 10) | bit(c.bidirectional, 9) | bit(c.low_side_reference_sn, 8) |
            static_cast<uint16_t>(c.csa_gain) << 6 | bit(c.disable_sense_overcurrent, 5) | static_cast<uint16_t>(c.sense_level);
    }
}
// the defaults above are the reset values of the datasheet
static_assert(get_drv8323_reg(DRV8323Init{}, 0) == 0x000 && get_drv8323_reg(DRV8323Init{}, 1) == 0x3FF &&
    get_drv8323_reg(DRV8323Init{}, 2) == 0x7FF && get_drv8323_reg(DRV8323Init{}, 3) == 0x159 &&
    get_drv8323_reg(DRV8323Init{}, 4) == 0x283);

// Amplifier gain as a number, for the current scaling
inline constexpr float drv8323_csa_gain(const DRV8323Init& c) {
    return 5 << static_cast<int>(c.csa_gain);
}

// What a configuration supplies to reach its driver, as static functions. A
// frame is in three steps so that the wait can be spent elsewhere:
//   transfer_start(word): chip select low and one 16 bit SPI frame started
//   transfer_ready():     the frame is in
//   transfer_end():       returns the word received, chip select high
//   enable(on):           the enable pin
// The 400 ns of chip select high between frames are the IO's to keep.
template<typename IO>
concept DRV8323IO = requires(uint16_t word, bool on) {
    IO::transfer_start(word);
    { IO::transfer_ready() } -> std::convertible_to<bool>;
    { IO::transfer_end() } -> std::convertible_to<uint16_t>;
    IO::enable(on);
};

// One frame, waiting for it: 16 bit times
template<DRV8323IO IO>
inline uint16_t drv8323_transfer(uint16_t word) {
    IO::transfer_start(word);
    while (!IO::transfer_ready());
    return IO::transfer_end();
}

// Frames: bit 15 read, bits 14..11 address, bits 10..0 data. The reply to a
// write is the register's previous value.
inline constexpr uint16_t drv8323_read_frame(uint8_t address) {
    return 1u << 15 | address << 11;
}
inline constexpr uint16_t drv8323_write_frame(uint8_t address, uint16_t value) {
    return address << 11 | (value & drv8323_data_mask);
}
template<DRV8323IO IO>
inline uint16_t drv8323_read(uint8_t address) {
    return drv8323_transfer<IO>(drv8323_read_frame(address)) & drv8323_data_mask;
}
template<DRV8323IO IO>
inline void drv8323_write(uint8_t address, uint16_t value) {
    drv8323_transfer<IO>(drv8323_write_frame(address, value));
}

// Fault status 2 << 16 | fault status 1. Status 1 bit 10 is FAULT, the state
// of the fault pin; 0 is no fault.
template<DRV8323IO IO>
inline uint32_t drv8323_status() {
    const uint32_t status1 = drv8323_read<IO>(drv8323_fault_status1);
    return drv8323_read<IO>(drv8323_fault_status2) << 16 | status1;
}

// Clears latched faults; the outputs follow the inputs again
template<DRV8323Init init, DRV8323IO IO>
inline void drv8323_clear_faults() {
    drv8323_write<IO>(drv8323_driver_control, get_drv8323_reg(init, 0) | drv8323_clear_fault);
}

// Into the low power state: all gates off, registers back at reset, no SPI
template<DRV8323IO IO>
inline void drv8323_disable() {
    IO::enable(false);
}

// GlobalTask of the configuration: one driver. The frame is static and the
// call hangs if it does not fit.
inline constexpr int drv8323_task_id = 110;
inline constexpr int drv8323_task_size = 96;

// Enables the driver and writes the control registers. The result is 0 when
// every register reads back as written, else a bit per register that does not
// (bit 0: driver control, ... bit 4: CSA control): no driver, no supply, or
// a broken SPI connection. The driver is left enabled either way: disabling
// it on a failure is the caller's decision. The inputs should hold the
// gates off while this runs, and nothing else may use the IO.
template<DRV8323Init init, DRV8323IO IO>
[[gnu::cold]] inline GlobalTask<uint32_t, drv8323_task_id, drv8323_task_size> drv8323_config_async(CycleScheduler& sched) {
    constexpr uint32_t wake_up_us = 10'000;     // 1 ms in the datasheet, the obot_g474 waits 10
    static constexpr uint16_t regs[drv8323_control_register_count] = {get_drv8323_reg(init, 0), get_drv8323_reg(init, 1),
        get_drv8323_reg(init, 2), get_drv8323_reg(init, 3), get_drv8323_reg(init, 4)};
    IO::enable(true);
    co_await sched.async_delay_us(wake_up_us);
    uint32_t errors = 0;
    // per register a write frame, then a read frame
    for (int frame = 0; frame < 2 * drv8323_control_register_count; frame++) {
        const int n = frame / 2;
        const bool read = frame & 1;
        IO::transfer_start(read ? drv8323_read_frame(drv8323_driver_control + n) : drv8323_write_frame(drv8323_driver_control + n, regs[n]));
        while (!IO::transfer_ready()) co_await sched.yield();
        const uint16_t reply = IO::transfer_end() & drv8323_data_mask;
        if (read && reply != regs[n]) errors |= 1u << n;
    }
    co_return errors;
}
}
