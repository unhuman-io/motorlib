#include "../../biss_crc.h"
#include <iostream>
#include <cstring>


int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <value_hex>" << std::endl;
        return 1;
    }

    uint32_t value = std::stoul(argv[1], 0, 16);

    uint8_t crc = CRC_BiSS_43_24bit(value);

    std::cout << "crc6 for value: " << std::hex << value
        << " is: " << (int) crc << " inverted: " << (~crc & 0x3f) << std::endl;

   std::cout << "crc4 of last 11 bits is: " << (int) biss_crc4(value) << std::endl;
}