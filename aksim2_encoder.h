#pragma once

#include "encoder.h"

class Aksim2Encoder : public EncoderBase {
 public:
    // BISS, 5 MHz max
    Aksim2Encoder(SPIDMA &spi_dma) : EncoderBase(), spi_dma_(spi_dma) {}
    void trigger() {
        spi_dma_.start_readwrite(data_out_, data_in_, length_);
    }
    int32_t read() {
        spi_dma_.finish_readwrite();
    }
    int32_t get_value() const {}
    bool index_received() { return true; }
 private:
    SPIDMA &spi_dma_;
    static const uint8_t length_ = 10;
    uint8_t data_out_[length_] = {};
    uint8_t data_in_[length_] = {}; 
};
