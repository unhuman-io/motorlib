#pragma once
#include "../flash.h"
#include <cstring>

#include "st_device.h"
#define BIT_BAND_SRAM(addr, bit) ((uint8_t *) 0x22000000)[(uint32_t) (addr-0x20000000) * 32 + 4 * bit]

class Flash : public FlashBase<Flash> {
 public:
    Flash(FLASH_TypeDef &regs) : regs_(regs) {
      if (!is_sbank()) {
        page_size_ = 2048;
      }
    }

    void unlock();
    void erase_page(uint32_t address);
    void write_dword(uint32_t address, const uint32_t* data);
    void write_impl(uint32_t address, const void *data, uint32_t size, EraseType erase);
    bool is_sbank() const { return (*((uint8_t *) 0x1fff7802) & 0x40) == 0; }
    void set_erased(uint32_t page) {
        //erased_[page/8] |= 1 << (page % 8);
        BIT_BAND_SRAM(erased_, page) = 1;
    }
    bool is_erased(uint32_t page) const {
        //return erased_[page/8] & (1 << (page % 8));
        return BIT_BAND_SRAM(erased_, page);
    }
    void clear_erased_status_impl() {
        std::memset(erased_, 0, sizeof(erased_));
    }

 private:
    FLASH_TypeDef &regs_;
    uint32_t page_size_ = 4096;
    uint8_t erased_[256/8] = {};
};
