#pragma once
#include <cstdint>
#include "../st_device.h"
#include "../../gpio.h"

inline std::vector<char> HexToBytes(const std::string& hex) {
  std::vector<char> bytes;

  for (unsigned int i = 0; i < hex.length(); i += 2) {
    std::string byteString = hex.substr(i, 2);
    char byte = (char) strtol(byteString.c_str(), NULL, 16);
    bytes.push_back(byte);
  }
  return bytes;
}

inline std::string BytesToHex(const std::vector<char>& bytes) {
    char finalhash[bytes.size()*2+1] = {};
    char hexval[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    for(unsigned int j = 0; j < bytes.size(); j++){
        finalhash[j*2] = hexval[((bytes[j] >> 4) & 0xF)];
        finalhash[(j*2) + 1] = hexval[(bytes[j]) & 0x0F];
    }
    return finalhash;
}

class SPIDebug {
 public:
    SPIDebug(SPI_TypeDef &regs, GPIO &gpio_cs, DMA_Channel_TypeDef &tx_dma, DMA_Channel_TypeDef &rx_dma) : 
        regs_(regs), gpio_cs_(gpio_cs),
        tx_dma_(tx_dma), rx_dma_(rx_dma) {}

    void init() {
        regs_.CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
        
        tx_dma_.CPAR = (uint32_t) &regs_.DR;
        
        rx_dma_.CPAR = (uint32_t) &regs_.DR;
    }

    std::string read(std::string hex_str) {
        
        std::vector<char> data_out = HexToBytes(hex_str);
        std::vector<char> data_in(data_out.size(), 0);
        gpio_cs_.clear();
        ns_delay(start_cs_delay_ns_);
        tx_dma_.CCR = 0;
        rx_dma_.CCR = 0;
        tx_dma_.CNDTR = hex_str.size()/2;
        rx_dma_.CNDTR = hex_str.size()/2;
        tx_dma_.CMAR = (uint32_t) data_out.data();
        rx_dma_.CMAR = (uint32_t) data_in.data();        
        rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory

        while(rx_dma_.CNDTR);
        ns_delay(end_cs_delay_ns_);
        // set CS high
        gpio_cs_.set();

        return "in " + std::to_string(data_in.size()) + " " + BytesToHex(data_in);
    }
 //private:
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
    uint16_t start_cs_delay_ns_ = 100;
    uint16_t end_cs_delay_ns_ = 100;

    friend class System;
};