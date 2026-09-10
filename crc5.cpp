#include "crc5.h"

namespace {
    // 3-Table Parallel Generator
    template <size_t NumBits, int Shift, uint8_t Seed>
    consteval auto generate_parallel_table() {
        std::array<uint8_t, (1ULL << NumBits)> table{};
        for (uint32_t val = 0; val < (1ULL << NumBits); ++val) {
            uint8_t crc = Seed;
            for (int i = NumBits - 1; i >= 0; --i) {
                uint8_t msb = (crc >> 4) & 1;
                crc = (crc << 1) & 0x1F;
                if (msb ^ ((val >> i) & 1)) crc ^= 0x05;
            }
            for (int i = 0; i < Shift; ++i) {
                uint8_t msb = (crc >> 4) & 1;
                crc = (crc << 1) & 0x1F;
                if (msb) crc ^= 0x05;
            }
            table[val] = crc;
        }
        return table;
    }

    // Sequential Generator
    template <int NumBits>
    consteval auto generate_sequential_table() {
        std::array<uint8_t, (1ULL << NumBits)> table{};
        for (uint32_t i = 0; i < (1ULL << NumBits); ++i) {
            uint32_t rem = i;
            for (int bit = 0; bit < NumBits; ++bit) {
                if (rem & (1 << (NumBits - 1))) {
                    rem = (rem << 1) ^ (0x05 << (NumBits - 5));
                } else {
                    rem <<= 1;
                }
            }
            table[i] = (rem >> (NumBits - 5)) & 0x1F;
        }
        return table;
    }
}

// Generate all three into the object file, separated by unique section names.
[[gnu::section(".data.crc_parallel")]] 
const Crc5Engine::ParallelTables Crc5Engine::parallel_tables = {
    generate_parallel_table<8, 18, 0x1F>(),
    generate_parallel_table<9, 9,  0x00>(),
    generate_parallel_table<9, 0,  0x00>()
};

[[gnu::section(".data.crc_seq9")]] 
const std::array<uint8_t, 512> Crc5Engine::seq9_table = generate_sequential_table<9>();

[[gnu::section(".data.crc_seq8")]] 
const std::array<uint8_t, 256> Crc5Engine::seq8_table = generate_sequential_table<8>();