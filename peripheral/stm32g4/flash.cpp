#include "flash.h"
#include "../../logger.h"

void Flash::unlock() {
    if (regs_.CR & FLASH_CR_LOCK) {
        regs_.KEYR = 0x45670123;
        regs_.KEYR = 0xCDEF89AB;
    }
}

void Flash::unlock_opt() {
    if (regs_.CR & FLASH_CR_OPTLOCK) {
        regs_.OPTKEYR = 0x08192A3B;
        regs_.OPTKEYR = 0x4C5D6E7F;
    }
}

void Flash::set_sbank(bool sbank) {
    unlock_opt();
    if (sbank) {
        regs_.OPTR &= ~FLASH_OPTR_DBANK;
    } else {
        regs_.OPTR |= FLASH_OPTR_DBANK;
    }
    save_opt();
    // will cause reset (and crash due to different flash)
}

void Flash::lock_section(uint32_t address, uint32_t size) {
    unlock_opt();
    uint32_t start_page = address_to_page(address);
    uint32_t end_page = address_to_page(address + size - 1);
    logger.log_printf("Locking pages %d-%d\n", start_page, end_page);

    if (is_sbank() || start_page < 128) {
        regs_.WRP1AR = start_page << FLASH_WRP1AR_WRP1A_STRT_Pos | end_page << FLASH_WRP1AR_WRP1A_END_Pos;
    } else {
        regs_.WRP2AR = (start_page - 128) << FLASH_WRP2AR_WRP2A_STRT_Pos | (end_page - 128) << FLASH_WRP2AR_WRP2A_END_Pos;
    }
    save_opt();
}

void Flash::unlock_section(uint32_t address, uint32_t size) {
    unlock_opt();
    // just unlock all
    // below are default values
    regs_.WRP1AR = 0xFF << FLASH_WRP1AR_WRP1A_STRT_Pos | 0x00 << FLASH_WRP1AR_WRP1A_END_Pos;
    regs_.WRP2AR = 0xFF << FLASH_WRP2AR_WRP2A_STRT_Pos | 0x00 << FLASH_WRP2AR_WRP2A_END_Pos;
    save_opt();
}

bool Flash::is_section_locked(uint32_t address, uint32_t size) const {
    uint32_t start_page = address_to_page(address);
    uint32_t end_page = address_to_page(address + size - 1);
    if (is_sbank() || start_page < 128) {
        return regs_.WRP1AR & (start_page << FLASH_WRP1AR_WRP1A_STRT_Pos) && regs_.WRP1AR & (end_page << FLASH_WRP1AR_WRP1A_END_Pos);
    } else {
        return regs_.WRP2AR & ((start_page - 128) << FLASH_WRP2AR_WRP2A_STRT_Pos) && regs_.WRP2AR & ((end_page - 128) << FLASH_WRP2AR_WRP2A_END_Pos);
    }
}

void Flash::save_opt(bool load) {
    regs_.CR = FLASH_CR_OPTSTRT;
    while (regs_.SR & FLASH_SR_BSY);
    if (load) {
        regs_.CR = FLASH_CR_OBL_LAUNCH;
    }
    while (regs_.SR & FLASH_SR_BSY);
}

bool Flash::check_general_error() {
    bool error = regs_.SR & ~FLASH_SR_EOP;
    regs_.SR = 0xFFFFFFFF;
    return error;
}

bool Flash::erase_page(uint32_t address) {
    uint32_t page = address_to_page(address);
    if (is_sbank()) {
        regs_.CR = FLASH_CR_PER | page << FLASH_CR_PNB_Pos | 0 << FLASH_CR_BKER_Pos;
    } else {
        regs_.CR = FLASH_CR_PER | (page & 0x7F) << FLASH_CR_PNB_Pos | ((page & 0x80) >> 7) << FLASH_CR_BKER_Pos;
    }
    regs_.CR |= FLASH_CR_STRT;
    while (regs_.SR & FLASH_SR_BSY);
    regs_.CR &= ~FLASH_CR_PER;
    return check_general_error();
}

bool Flash::write_dword(uint32_t address, const uint32_t* data) {
    *reinterpret_cast<volatile uint32_t*>(address) = data[0];
    *reinterpret_cast<volatile uint32_t*>(address+4) = data[1];
    while (regs_.SR & FLASH_SR_BSY);
    return check_general_error();
}

bool Flash::write_impl(uint32_t address, const void *data, uint32_t size) {
    __disable_irq();
    unlock();
    regs_.SR = regs_.SR; // clear previous errors
    uint32_t num_pages = (size+1) / page_size_ + 1;
    for (uint32_t i = 0; i < num_pages; i++) {
        IWDG->KR = 0xAAAA;
        if (!erase_page(address + i * page_size_)) {
            __enable_irq();
            return false;
        }
    }

    // round size up to nearest +8
    size = (size + 7) & ~7;
    uint32_t size32 = size/4;
    regs_.CR = FLASH_CR_PG;
    const uint32_t *data32 = static_cast<const uint32_t*>(data);
    for (uint32_t i = 0; i < size32; i += 2) {
        IWDG->KR = 0xAAAA;
        if (!write_dword(address + i*4, &data32[i])) {
            __enable_irq();
            return false;
        }
    }
    regs_.CR &= ~FLASH_CR_PG;
    return true;
}

uint32_t Flash::address_to_page(uint32_t address) const {
    return (address - 0x8000000) / page_size_;
}
