#pragma once
//module;

#include <charconv>
#include <cstdint>
#include <cstring>
#include <utility>
#include <ranges>
#include <algorithm>
#include <concepts>
#include <limits>
#include <type_traits>
// #include <functional>
//export module text_fun;



// consteval uint32_t pow10(int n) {
//     return std::ranges::fold_left(std::views::repeat(10, n), 1, std::multiplies<>{});
// }

// export template<int precision = 3>
// char* format_float(char* buffer, char* buffer_end, float val) {
//     static_assert(precision >= 0 && precision <= 6, "Precision must be between 0 and 6");
//     int32_t whole = static_cast<int32_t>(val);

//     const float frac_float = std::abs(val - static_cast<float>(whole));
    
//     int32_t fraction = static_cast<int32_t>(frac_float * pow10(precision) + 0.5f);

//     if constexpr (precision > 0) {
//         if (fraction >= pow10(precision)) {
//             fraction = 0;
//             whole += (val >= 0) ? 1 : -1;
//         }
//     }

//     auto res = std::to_chars(buffer, buffer_end, whole);
//     if (res.ec != std::errc()) return buffer; // Handle buffer overflow
//     buffer = res.ptr;

//     if constexpr (precision > 0) {
//         *buffer++ = '.';

//         if constexpr (precision >= 6) if (fraction < 100000) *buffer++ = '0';
//         if constexpr (precision >= 5) if (fraction < 10000)  *buffer++ = '0';
//         if constexpr (precision >= 4) if (fraction < 1000)   *buffer++ = '0';
//         if constexpr (precision >= 3) if (fraction < 100)    *buffer++ = '0';
//         if constexpr (precision >= 2) if (fraction < 10)     *buffer++ = '0';

//         res = std::to_chars(buffer, buffer_end, fraction);
//     }
    
//     return res.ptr;
// }


template <size_t MaxCapacity>
class StackString {
public:
    char buffer[MaxCapacity + 1]; // +1 for null terminator
    uint8_t current_len = 0;

    // --- Format Strings ---
    template <size_t N>
    inline __attribute__((always_inline))
    void push(const char (&str)[N]) {
        std::memcpy(buffer + current_len, str, N - 1);
        current_len += N - 1;
    }

    // --- Format Floats (from our previous fixed-point logic) ---
    inline __attribute__((always_inline))
    void push(float val) {
        if (val < 0) { buffer[current_len++] = '-'; val = -val; }
        val += 0.0005f;
        int32_t whole = static_cast<int32_t>(val);
        int32_t fraction = static_cast<int32_t>((val - whole) * 1000.0f);

        auto res = std::to_chars(buffer + current_len, buffer + MaxCapacity, whole);
        current_len = res.ptr - buffer;
        buffer[current_len++] = '.';

        if (fraction < 10) { buffer[current_len++] = '0'; buffer[current_len++] = '0'; } 
        else if (fraction < 100) { buffer[current_len++] = '0'; }

        res = std::to_chars(buffer + current_len, buffer + MaxCapacity, fraction);
        current_len = res.ptr - buffer;
    }

    // --- Format Ints ---
    template <std::integral T>
    inline __attribute__((always_inline))
    void push(T val) {
        auto res = std::to_chars(buffer + current_len, buffer + MaxCapacity, val);
        current_len = res.ptr - buffer;
    }

    // Hyper-optimized unsigned 32-bit integer push
    inline __attribute__((always_inline))
    void push(uint32_t val) {
        if (val == 0) {
            buffer[current_len++] = '0';
            return;
        }

        // Max uint32_t is 4294967295 (10 digits)
        char temp[10];
        char* p = temp + 10;
        
        // Write digits backwards (fastest mathematical method)
        while (val > 0) {
            // Cortex-M4 will compile this into a fast UDIV and MLS instruction pair
            uint32_t quotient = val / 10; 
            uint32_t remainder = val - (quotient * 10); 
            
            *--p = '0' + remainder;
            val = quotient;
        }
        
        // Calculate length and copy forward
        uint32_t len = (temp + 10) - p;
        
        // __builtin_memcpy tells GCC to use native LDM/STM instructions 
        // instead of calling the C-library memcpy.
        __builtin_memcpy(buffer + current_len, p, len);
        current_len += len;
    }

    // --- Allow composing StackStrings together! ---
    template <size_t OtherCap>
    inline __attribute__((always_inline))
    void push(const StackString<OtherCap>& other) {
        std::memcpy(buffer + current_len, other.buffer, other.current_len);
        current_len += other.current_len;
    }

    inline __attribute__((always_inline))
    operator std::string_view() const { 
        return std::string_view(buffer, current_len); 
    }
};

// Base size calculators
template <size_t N> constexpr size_t get_max_size(const char (&)[N]) { return N - 1; }
constexpr size_t get_max_size(float) { return 15; }
// Calculate exact max characters needed for ANY integer type at compile time
template <typename T, std::enable_if_t<std::is_integral_v<std::remove_reference_t<T>>, int> = 0>
constexpr size_t get_max_size(T&&) { 
    // digits10 gives the max decimal digits. 
    // +1 for a potential minus sign, +1 for rounding safety.
    return std::numeric_limits<std::remove_reference_t<T>>::digits10 + 2; 
}

// Allow calculating size of nested StackStrings
template <size_t N> constexpr size_t get_max_size(const StackString<N>&) { return N; }

// The Generator
template <typename... Args>
inline __attribute__((always_inline)) 
auto make_string(Args&&... args) {
    constexpr size_t TotalSize = (get_max_size(args) + ...);
    
    StackString<TotalSize> str;
    (str.push(std::forward<Args>(args)), ...);
    
    str.buffer[str.current_len] = '\0'; // Null terminate safely at the end
    return str;
}

// export template <size_t N>
// constexpr size_t get_max_size(const char (&)[N]) { return N - 1; }

// export template <size_t size>
// class LogBuilder {
// public:
//     ~LogBuilder() {
//         buffer[current_len++] = '\0';
//         //write_log(buffer, current_len);
//     }

//     template <size_t N>
//     void push(const char (&str)[N]) {
//         std::memcpy(buffer + current_len, str, N-1);
//         current_len += N - 1;
//     }

// private:
//     char buffer[size + 1];
//     uint8_t current_len = 0;
// };

// export template<typename... Args>
// void log(Args&&... args) {
//     constexpr size_t TotalSize = (get_max_size(args) + ...);
//     LogBuilder<TotalSize> builder;
//     (builder.push(std::forward<Args>(args)), ...);
// };
