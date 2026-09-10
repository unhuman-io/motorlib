#pragma once
#include <array>
#include <cstdint>
#include <cstddef>

enum class Crc5Strategy {
    Parallel, // 1280 Bytes, ~16 cycles, 0 stalls
    Seq9,     // 512 Bytes,  ~19 cycles, 2 stalls
    Seq8      // 256 Bytes,  ~21 cycles, 3 stalls
};

class Crc5Engine {
private:
    struct ParallelTables {
        std::array<uint8_t, 256> T2;
        std::array<uint8_t, 512> T1;
        std::array<uint8_t, 512> T0;
    };

    // Notice the unique section suffixes. This is MANDATORY for --gc-sections 
    // to successfully isolate and strip the unused arrays from the final binary.
    [[gnu::section(".data.crc_parallel")]] static const ParallelTables parallel_tables;
    [[gnu::section(".data.crc_seq9")]]     static const std::array<uint8_t, 512> seq9_table;
    [[gnu::section(".data.crc_seq8")]]     static const std::array<uint8_t, 256> seq8_table;

public:
    template <Crc5Strategy Strategy = Crc5Strategy::Seq9>
    [[nodiscard]] 
    static inline uint8_t calculate(uint32_t msg) {
        
        if constexpr (Strategy == Crc5Strategy::Parallel) {
            uint32_t m2 = (msg >> 18) & 0xFF;
            uint32_t m1 = (msg >> 9)  & 0x1FF;
            uint32_t m0 = msg         & 0x1FF;
            return parallel_tables.T2[m2] ^ parallel_tables.T1[m1] ^ parallel_tables.T0[m0];
        } 
        else if constexpr (Strategy == Crc5Strategy::Seq9) {
            uint8_t crc = 0x1D; 
            crc = seq9_table[ (crc << 4) ^ ((msg >> 18) & 0x1FF) ];
            crc = seq9_table[ (crc << 4) ^ ((msg >> 9)  & 0x1FF) ];
            crc = seq9_table[ (crc << 4) ^ (msg         & 0x1FF) ];
            return crc;
        } 
        else if constexpr (Strategy == Crc5Strategy::Seq8) {
            uint8_t crc = 0x1A; 
            crc = seq8_table[ (crc << 3) ^ ((msg >> 24) & 0xFF) ];
            crc = seq8_table[ (crc << 3) ^ ((msg >> 16) & 0xFF) ];
            crc = seq8_table[ (crc << 3) ^ ((msg >> 8)  & 0xFF) ];
            crc = seq8_table[ (crc << 3) ^ (msg         & 0xFF) ];
            return crc;
        }
    }
};
