#pragma once
#include <cstdint>

template <class T>
class FlashBase {
 public:
    enum EraseType {NO_ERASE=0, ALWAYS_ERASE=1, ERASE_ONCE=2};
    // ALWAYS_ERASE will erase the pages necessary for address and size.
    // Assumes that address is at the start of a page
    // data must be located in ram. ERASE_ONCE will erase the page if 
    // is not erased. The page erase statuses can be cleared.
    void write(uint32_t address, const void *data, uint32_t size, EraseType erase = ALWAYS_ERASE) {
        static_cast<T*>(this)->write_impl(address, data, size, erase);
    }
    void clear_erased_status() {
        static_cast<T*>(this)->clear_erased_status_impl();
    }
};