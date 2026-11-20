module;
#include <cstdint>
#include <cstring>
#include "stm32g474xx.h"

export module device.crc_init;

export {
// Compile-time configuration of the CRC unit. Its clock (RCC crc enable) must
// be running.

enum class CRCPolynomialSize : uint8_t {    // CR POLYSIZE
    BITS_32,
    BITS_16,
    BITS_8,
    BITS_7,
};

enum class CRCInputReverse : uint8_t {      // CR REV_IN: the bit order of what is written
    NONE,
    BYTE,
    HALF_WORD,
    WORD,
};

enum class CRCOutputReverse : uint8_t {     // CR REV_OUT
    NONE,
    BIT,
};

// Defaults are the register reset values: the crc of ethernet and mpeg-2,
// without the reflections
struct CRCInit {
    uint32_t polynomial = 0x04C11DB7;                           // POL
    CRCPolynomialSize polynomial_size = CRCPolynomialSize::BITS_32;
    uint32_t initial_value = 0xFFFFFFFF;                        // INIT
    CRCInputReverse input_reverse = CRCInputReverse::NONE;
    CRCOutputReverse output_reverse = CRCOutputReverse::NONE;
    uint32_t output_xor = 0;                                    // not in the unit: crc_read() applies it
};

// The crc32 of zlib, python and most everything else
inline constexpr CRCInit crc32_init = {
    .input_reverse = CRCInputReverse::BYTE,
    .output_reverse = CRCOutputReverse::BIT,
    .output_xor = 0xFFFFFFFF,
};

inline consteval uint32_t get_crc_cr(const CRCInit& c) {
    return static_cast<uint32_t>(c.polynomial_size) << CRC_CR_POLYSIZE_Pos |
        static_cast<uint32_t>(c.input_reverse) << CRC_CR_REV_IN_Pos |
        static_cast<uint32_t>(c.output_reverse) << CRC_CR_REV_OUT_Pos;
}

// Only what differs from reset is written
template<const CRCInit c>
inline void init_crc() {
    if constexpr (c.polynomial != CRCInit{}.polynomial) CRC->POL = c.polynomial;
    if constexpr (c.initial_value != CRCInit{}.initial_value) CRC->INIT = c.initial_value;
    CRC->CR = get_crc_cr(c) | CRC_CR_RESET;
}

template<CRCInit crc_init>
inline void device_crc_config() {
    init_crc<crc_init>();
}

// A crc in steps: crc_reset<init>() starts from the initial value, crc_write()
// adds bytes in the order of the memory, crc_read<init>() is the result so far
template<const CRCInit c>
inline void crc_reset() {
    CRC->CR = get_crc_cr(c) | CRC_CR_RESET;
}

inline void crc_write(const void* data, uint32_t length) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (; length >= 4; length -= 4, p += 4) {
        uint32_t word;
        std::memcpy(&word, p, 4);
        CRC->DR = __builtin_bswap32(word);      // the unit takes the high byte of a word first
    }
    for (; length > 0; length--, p++) {
        *reinterpret_cast<volatile uint8_t*>(&CRC->DR) = *p;
    }
}

template<const CRCInit c>
inline uint32_t crc_read() {
    return CRC->DR ^ c.output_xor;
}

// The crc of a block: 4 cpu cycles per word in the unit
template<const CRCInit c>
inline uint32_t crc_compute(const void* data, uint32_t length) {
    crc_reset<c>();
    crc_write(data, length);
    return crc_read<c>();
}
}
