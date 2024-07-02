#pragma once
#include "../flash.h"

#include "st_device.h"

class Flash : public FlashBase<Flash> {
 public:
    Flash(FLASH_TypeDef &regs) : regs_(regs) {
      if (!is_sbank()) {
        page_size_ = 2048;
      }
    }

    void unlock();
    void unlock_opt();
    void save_opt(bool load=true);
    void erase_page(uint32_t address);
    void write_dword(uint32_t address, const uint32_t* data);
    void write_impl(uint32_t address, const void *data, uint32_t size);
    bool is_sbank() const { return (regs_.OPTR & FLASH_OPTR_DBANK) == 0; } // Also *((uint8_t *) 0x1fff7802) & 0x40) == 0;
    void lock_section(uint32_t address, uint32_t size);
    void unlock_section(uint32_t address, uint32_t size);
    bool is_section_locked(uint32_t address, uint32_t size) const;
    uint32_t address_to_page(uint32_t address) const;
    void set_sbank(bool sbank=true);

 private:
    FLASH_TypeDef &regs_;
    uint32_t page_size_ = 4096;
};
