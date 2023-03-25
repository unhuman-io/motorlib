#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DEBUG_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DEBUG_H_

#include <cstdint>
#include <vector>
#include "../../util.h"
#include "spi_dma.h"

class SPIDebug {
 public:
    SPIDebug(SPIDMA &spi_dma, volatile int* register_operation = nullptr) : 
        spi_dma_(spi_dma) {
            if (register_operation != nullptr) {
                register_operation_ = register_operation;
            }
        }

    std::string read() {
        (*register_operation_)++;
        std::vector<char> data_out = hex_to_bytes(hex_str_);
        std::vector<char> data_in(data_out.size(), 0);
        spi_dma_.readwrite((uint8_t *) data_out.data(), (uint8_t *) data_in.data(), hex_str_.size()/2);
        (*register_operation_)--;
        return "in " + std::to_string(data_in.size()) + " " + bytes_to_hex(data_in);
    }

    void write(std::string hex_str) {
        hex_str_ = hex_str;
    }
    volatile int * register_operation_ = &register_operation_local_;
  private:
    SPIDMA &spi_dma_;
    std::string hex_str_ = "";
    volatile int register_operation_local_;

    friend void config_init();
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_SPI_DEBUG_H_
