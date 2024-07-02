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
    void erase_page(uint32_t address);
    void write_dword(uint32_t address, const uint32_t* data);
    void write_impl(uint32_t address, const void *data, uint32_t size);
    bool is_sbank() const { return (*((uint8_t *) 0x1fff7802) & 0x40) == 0; }

 private:
    FLASH_TypeDef &regs_;
    uint32_t page_size_ = 4096;
};
