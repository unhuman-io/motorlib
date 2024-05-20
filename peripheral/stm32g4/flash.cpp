#include "flash.h"

void Flash::unlock() {
    if (regs_.CR & FLASH_CR_LOCK) {
        regs_.KEYR = 0x45670123;
        regs_.KEYR = 0xCDEF89AB;
    }
}

void Flash::erase_page(uint32_t address) {
    uint32_t page = (address - 0x8000000) / page_size_;
    regs_.CR = FLASH_CR_PER | page << FLASH_CR_PNB_Pos | 0 << FLASH_CR_BKER_Pos;
    regs_.CR |= FLASH_CR_STRT;
    while (regs_.SR & FLASH_SR_BSY);
    regs_.CR &= ~FLASH_CR_PER;
}

void Flash::write_dword(uint32_t address, const uint32_t* data) {
    regs_.CR = FLASH_CR_PG;
    *reinterpret_cast<volatile uint32_t*>(address) = data[0];
    *reinterpret_cast<volatile uint32_t*>(address+4) = data[1];
    while (regs_.SR & FLASH_SR_BSY);
    regs_.CR &= ~FLASH_CR_PG;
}

void Flash::write_impl(uint32_t address, const void *data, uint32_t size) {
    __disable_irq();
    unlock();
    regs_.SR = regs_.SR; // clear previous errors
    uint32_t num_pages = (size+1) / page_size_ + 1;
    for (uint32_t i = 0; i < num_pages; i++) {
        IWDG->KR = 0xAAAA;
        erase_page(address + i * page_size_);
    }

    // round size up to nearest +8
    size = (size + 7) & ~7;
    const uint32_t *data32 = static_cast<const uint32_t*>(data);
    for (uint32_t i = 0; i < size; i += 8) {
        IWDG->KR = 0xAAAA;
        write_dword(address + i, &data32[i]);
    }
    __enable_irq();
}
