module;
#include <cstdint>
#include "stm32g474xx.h"

export module device.system;

export {
// Resets, the watchdog and the hand over to the can bootloader
// (motorlib/bootloader/stm32g4) for an stm32g4 application.
//
// That bootloader starts the independent watchdog before it jumps to the
// application: 0.5 s until the reset, and a watchdog reset stays in the
// bootloader. An application behind it has to call watchdog_kick() more often
// than that; one that stops, on purpose or not, ends up in the bootloader.
// Without the bootloader the watchdog does not run and the kick does nothing.

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

// The 96 bit unique device id as the 12 hex digits the bootloader and the
// obot_g474 firmware report as the serial number
inline void system_serial_number(char serial[13]) {
    constexpr char hex[] = "0123456789abcdef";
    const uint32_t* uid = reinterpret_cast<const uint32_t*>(UID_BASE);
    const uint32_t word0 = uid[0] + uid[2];
    const uint32_t word1 = uid[1];
    for (int i = 0; i < 8; i++) serial[i] = hex[(word0 >> (28 - 4 * i)) & 0xF];
    for (int i = 0; i < 4; i++) serial[8 + i] = hex[(word1 >> (28 - 4 * i)) & 0xF];
    serial[12] = 0;
}
}
