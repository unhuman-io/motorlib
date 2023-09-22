#include "util.h"

volatile uint32_t uptime = 0;

void ms_delay(uint16_t ms) {
    uint32_t t_start = get_clock();
    while((get_clock() - t_start) < ms*CPU_FREQUENCY_HZ/1000);
}

void us_delay(uint16_t us) {
    uint32_t t_start = get_clock();
    while((get_clock() - t_start) < us*CPU_FREQUENCY_HZ/1000000);
}

void ns_delay(uint16_t ns) {
    uint32_t t_start = get_clock();
    while((get_clock() - t_start) < ns/(uint16_t) (1e9/CPU_FREQUENCY_HZ));
}

std::vector<char> hex_to_bytes(const std::string& hex) {
  std::vector<char> bytes;

  for (unsigned int i = 0; i < hex.length(); i += 2) {
    std::string byteString = hex.substr(i, 2);
    char byte = (char) strtol(byteString.c_str(), NULL, 16);
    bytes.push_back(byte);
  }
  return bytes;
}

std::string byte_to_hex(const uint8_t byte) {
    const char hexval[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    char c[3] = { hexval[(byte >> 4) & 0xF], hexval[byte & 0xF] };
    std::string s = c;
    return s;
}

std::string u16_to_hex(const uint16_t w) {
    return byte_to_hex((uint8_t) (w>>8)) + byte_to_hex((uint8_t) (w & 0xFF));
}

std::string u32_to_hex(const uint32_t w) {
    return byte_to_hex((uint8_t) (w>>24)) + byte_to_hex((uint8_t) ((w>>16) & 0xFF)) +
            byte_to_hex((uint8_t) ((w>>8) & 0xFF)) + byte_to_hex((uint8_t) (w & 0xFF));
}

std::string bytes_to_hex(const std::vector<char>& bytes) { 
    std::string s;   
    for (uint8_t b : bytes) {
        s += byte_to_hex(b);
    }
    return s;
}

std::string bytes_to_hex(const std::vector<uint8_t>& bytes) { 
    std::string s;   
    for (uint8_t b : bytes) {
        s += byte_to_hex(b);
    }
    return s;
}

std::string bytes_to_hex(const uint8_t bytes[], const uint8_t length) { 
    std::string s;   
    for (int i=0; i<length; i++) {
        s += byte_to_hex(bytes[i]);
    }
    return s;
}