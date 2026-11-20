#include <iostream>
#include "../crc5.h"

int main() {
    // The user's requested string (truncated safely to 26 bits)
    uint32_t test_msg = 0b000111000000000000000000000 & 0x3FFFFFF;
    
    std::cout << "--- 26-bit CRC-5 Test ---" << std::endl;
    std::cout << "Message (Hex): 0x" << std::hex << std::uppercase << test_msg << std::dec << std::endl;
    
    // 1. Calculate Reference
    uint8_t expected = 27;
    std::cout << "Reference (Bit-by-Bit): 0x" << std::hex << (int)expected << std::endl;
    
    // 2. Calculate using Fast Tables
    uint8_t res_parallel = Crc5Engine::calculate<Crc5Strategy::Parallel>(test_msg);
    uint8_t res_seq9     = Crc5Engine::calculate<Crc5Strategy::Seq9>(test_msg);
    uint8_t res_seq8     = Crc5Engine::calculate<Crc5Strategy::Seq8>(test_msg);
    
    std::cout << "Parallel (1280 bytes):  0x" << std::hex << (int)res_parallel << std::endl;
    std::cout << "Seq9     ( 512 bytes):  0x" << std::hex << (int)res_seq9     << std::endl;
    std::cout << "Seq8     ( 256 bytes):  0x" << std::hex << (int)res_seq8     << std::endl;
    
    std::cout << "\nResult: ";
    if (expected == res_parallel && expected == res_seq9 && expected == res_seq8) {
        std::cout << "PASS! All engines match hardware." << std::endl;
        return 0;
    } else {
        std::cout << "FAIL! Mismatch detected." << std::endl;
        return 1;
    }
}
