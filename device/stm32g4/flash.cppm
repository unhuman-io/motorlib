module;
#include <cstdint>
#include "stm32g474xx.h"

export module device.flash;

export {
// The flash of an stm32g4 as a bootloader needs it: erase a page, program a
// double word. Both wait for the operation and return the error flags of the
// status register, and flash_not_done if it did not end: 0 if all is well.
// The cpu stalls while it executes from the flash that is busy, so interrupts
// are late by up to an erase (22 ms typical).

inline constexpr uint32_t flash_base = FLASH_BASE;
inline constexpr uint32_t flash_error_mask = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
    FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR;
// No end of operation flag: the operation never started, as with a flash that
// is locked, where the writes to the control register are ignored and no
// error flag is raised either. Bit 0, where the status register has that flag.
inline constexpr uint32_t flash_not_done = FLASH_SR_EOP;
static_assert((flash_error_mask & flash_not_done) == 0);

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

// The result of the operation that just ended. The end of operation flag is
// only raised with its interrupt enable (EOPIE) set in the control register.
inline uint32_t flash_result() {
    const uint32_t status = FLASH->SR;
    return (status & flash_error_mask) | (status & FLASH_SR_EOP ? 0 : flash_not_done);
}

// The data and the instruction cache of the flash keep what they hold through
// an erase. Measured on an stm32g474: a read after the erase returned the old
// contents and a call ran the old code, with the new one programmed, for as
// long as the line stayed in the cache. Off, reset, on, as in ST's driver;
// off and on alone leaves the lines in. A program takes the lines it writes
// out of the data cache by itself, but not out of the instruction cache.
inline void flash_cache_reset() {
    const uint32_t acr = FLASH->ACR;
    const uint32_t off = acr & ~(FLASH_ACR_DCEN | FLASH_ACR_ICEN);
    FLASH->ACR = off;
    FLASH->ACR = off | FLASH_ACR_DCRST | FLASH_ACR_ICRST;
    FLASH->ACR = off;
    FLASH->ACR = acr;
}

// The page that holds address
inline uint32_t flash_erase_page(uint32_t address) {
    const uint32_t page = (address - flash_base) / flash_page_size();
    FLASH->SR = flash_error_mask | FLASH_SR_EOP;       // flags of earlier operations
    const uint32_t select = flash_dual_bank() ?
        (page & 0x7F) << FLASH_CR_PNB_Pos | (page & 0x80 ? FLASH_CR_BKER : 0) : page << FLASH_CR_PNB_Pos;
    FLASH->CR = FLASH_CR_EOPIE | FLASH_CR_PER | select;
    FLASH->CR = FLASH_CR_EOPIE | FLASH_CR_PER | select | FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    const uint32_t result = flash_result();
    FLASH->CR = 0;
    flash_cache_reset();
    return result;
}

// 8 bytes at an address that is a multiple of 8, in an erased page
inline uint32_t flash_program(uint32_t address, uint32_t low, uint32_t high) {
    FLASH->SR = flash_error_mask | FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_EOPIE | FLASH_CR_PG;
    *reinterpret_cast<volatile uint32_t*>(address) = low;
    *reinterpret_cast<volatile uint32_t*>(address + 4) = high;
    while (FLASH->SR & FLASH_SR_BSY);
    const uint32_t result = flash_result();
    FLASH->CR = 0;
    return result;
}
}
