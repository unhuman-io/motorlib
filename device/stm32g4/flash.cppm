module;
#include <cstdint>
#include "stm32g474xx.h"

export module device.flash;

export {
// The flash of an stm32g4 as a bootloader needs it: erase a page, program a
// double word. Both wait for the operation and return the error flags of the
// status register, 0 if there were none. The cpu stalls while it executes
// from the flash that is busy, so interrupts are late by up to an erase
// (22 ms typical).

inline constexpr uint32_t flash_base = FLASH_BASE;
inline constexpr uint32_t flash_error_mask = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
    FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR;

// From the size the device reports, in KB
inline uint32_t flash_end() {
    return flash_base + *reinterpret_cast<const uint16_t*>(FLASHSIZE_BASE) * 1024u;
}

// Category 3 devices have 2K pages in two banks or, with the DBANK option
// cleared, 4K pages in one
inline bool flash_dual_bank() { return (FLASH->OPTR & FLASH_OPTR_DBANK) != 0; }
inline uint32_t flash_page_size() { return flash_dual_bank() ? 2048 : 4096; }

inline void flash_unlock() {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
}
inline void flash_lock() { FLASH->CR = FLASH_CR_LOCK; }

// The page that holds address
inline uint32_t flash_erase_page(uint32_t address) {
    const uint32_t page = (address - flash_base) / flash_page_size();
    FLASH->SR = flash_error_mask | FLASH_SR_EOP;       // flags of earlier operations
    const uint32_t select = flash_dual_bank() ?
        (page & 0x7F) << FLASH_CR_PNB_Pos | (page & 0x80 ? FLASH_CR_BKER : 0) : page << FLASH_CR_PNB_Pos;
    FLASH->CR = FLASH_CR_PER | select;
    FLASH->CR = FLASH_CR_PER | select | FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    const uint32_t errors = FLASH->SR & flash_error_mask;
    FLASH->CR = 0;
    return errors;
}

// 8 bytes at an address that is a multiple of 8, in an erased page
inline uint32_t flash_program(uint32_t address, uint32_t low, uint32_t high) {
    FLASH->SR = flash_error_mask | FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_PG;
    *reinterpret_cast<volatile uint32_t*>(address) = low;
    *reinterpret_cast<volatile uint32_t*>(address + 4) = high;
    while (FLASH->SR & FLASH_SR_BSY);
    const uint32_t errors = FLASH->SR & flash_error_mask;
    FLASH->CR = 0;
    return errors;
}
}
