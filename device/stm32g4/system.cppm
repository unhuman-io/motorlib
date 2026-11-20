module;
#include <cstdint>
#include "stm32g474xx.h"

export module device.system;

export {
// Resets, the watchdog and the hand over to the can bootloader
// (obot3/bootloader, or the earlier motorlib/bootloader/stm32g4) for an stm32g4
// application.
//
// That bootloader starts the independent watchdog before it jumps to the
// application: 0.5 s until the reset, and a watchdog reset stays in the
// bootloader. An application behind it has to call watchdog_kick() more often
// than that; one that stops, on purpose or not, ends up in the bootloader.
// Without the bootloader the watchdog does not run and the kick does nothing.

// Where that bootloader ends and the application's vector table starts
inline constexpr uint32_t application_address = 0x08002000;

// What the bootloader does first: the independent watchdog at its reset
// prescaler and reload, 0.5 s, and stopped while a debugger has the cpu halted
inline void watchdog_start() {
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP | DBGMCU_APB1FZR1_DBG_WWDG_STOP;
    IWDG->KR = 0xCCCC;
}

inline void watchdog_kick() {
    IWDG->KR = 0xAAAA;
}

[[noreturn]] inline void system_reset() {
    SCB->AIRCR = 0x5FAu << SCB_AIRCR_VECTKEY_Pos | SCB_AIRCR_SYSRESETREQ_Msk;
    while (true) __asm__ volatile("");
}

// The bootloader stays in control after a software reset if it finds this
// value in its flag, the first word of the ram. That word belongs to the
// application's data until the reset.
inline constexpr uint32_t bootloader_flag_address = 0x20000000;
inline constexpr uint32_t bootloader_flag_value = 0xB007;

[[noreturn]] inline void system_reset_to_bootloader() {
    *reinterpret_cast<volatile uint32_t*>(bootloader_flag_address) = bootloader_flag_value;
    system_reset();
}

// A load or store at an address that may have nothing behind it, for a
// debugger's memory access. Bus faults of data accesses are ignored for its
// duration, which needs the faults and with them every interrupt masked: about
// 30 cpu cycles. False if the access faulted.
template<typename Access>
inline bool memory_access(Access access) {
    constexpr uint32_t bus_faults = SCB_CFSR_BUSFAULTSR_Msk;
    __asm__ volatile("cpsid f");
    SCB->CFSR = bus_faults;                         // flags of the past
    SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;
    access();
    __asm__ volatile("dsb\nisb");                   // a fault of a buffered write arrives here
    const bool fault = (SCB->CFSR & bus_faults) != 0;
    SCB->CFSR = bus_faults;
    SCB->CCR &= ~SCB_CCR_BFHFNMIGN_Msk;
    __asm__ volatile("cpsie f");
    return !fault;
}

// Blocks, in the widest accesses that address and length are multiples of: a
// word that is read or written as one stays whole next to an interrupt, and
// peripheral registers get the access they expect. Both return the bytes done
// before a fault.
template<typename T>
inline uint32_t memory_read_as(uint32_t address, uint8_t* data, uint32_t length) {
    for (uint32_t i = 0; i < length; i += sizeof(T)) {
        T value{};
        if (!memory_access([&] { value = *reinterpret_cast<const volatile T*>(address + i); })) return i;
        for (uint32_t byte = 0; byte < sizeof(T); byte++) data[i + byte] = static_cast<uint8_t>(value >> (8 * byte));
    }
    return length;
}
inline uint32_t memory_read(uint32_t address, uint8_t* data, uint32_t length) {
    if (((address | length) & 3) == 0) return memory_read_as<uint32_t>(address, data, length);
    if (((address | length) & 1) == 0) return memory_read_as<uint16_t>(address, data, length);
    return memory_read_as<uint8_t>(address, data, length);
}

template<typename T>
inline uint32_t memory_write_as(uint32_t address, const uint8_t* data, uint32_t length) {
    for (uint32_t i = 0; i < length; i += sizeof(T)) {
        T value = 0;
        for (uint32_t byte = 0; byte < sizeof(T); byte++) value |= static_cast<T>(data[i + byte]) << (8 * byte);
        if (!memory_access([&] { *reinterpret_cast<volatile T*>(address + i) = value; })) return i;
    }
    return length;
}
inline uint32_t memory_write(uint32_t address, const uint8_t* data, uint32_t length) {
    if (((address | length) & 3) == 0) return memory_write_as<uint32_t>(address, data, length);
    if (((address | length) & 1) == 0) return memory_write_as<uint16_t>(address, data, length);
    return memory_write_as<uint8_t>(address, data, length);
}

// The 96 bit unique device id as the 6 bytes that are the serial number:
// printed in this order as 12 hex digits it is what the bootloader and the
// obot_g474 firmware report
inline void system_serial_bytes(uint8_t serial[6]) {
    const uint32_t* uid = reinterpret_cast<const uint32_t*>(UID_BASE);
    const uint32_t word0 = uid[0] + uid[2];
    const uint32_t word1 = uid[1];
    for (int i = 0; i < 4; i++) serial[i] = word0 >> (24 - 8 * i);
    for (int i = 0; i < 2; i++) serial[4 + i] = word1 >> (24 - 8 * i);
}
inline void system_serial_number(char serial[13]) {
    constexpr char hex[] = "0123456789abcdef";
    uint8_t bytes[6];
    system_serial_bytes(bytes);
    for (int i = 0; i < 6; i++) {
        serial[2 * i] = hex[bytes[i] >> 4];
        serial[2 * i + 1] = hex[bytes[i] & 0xF];
    }
    serial[12] = 0;
}
}
