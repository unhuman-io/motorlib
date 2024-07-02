#pragma once
#include <cstdint>

template <class T>
class FlashBase {
 public:
    // This will erase the pages necessary for address and size.
    // Assumes that address is at the start of a page
    // data must be located in ram
    void write(uint32_t address, const void *data, uint32_t size) {
        static_cast<T*>(this)->write_impl(address, data, size);
    }
};