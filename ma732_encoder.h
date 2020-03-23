#include "peripheral/spi_encoder.h"

// Note MA732 encoder expects cpol 1, cpha 1, max 25 mbit
// 80 ns cs start to sclk, 25 ns sclk end to cs end
class MA732Encoder final : public SPIEncoder {
 public:
    union MA732reg {
        struct {
            uint16_t value:8;
            uint16_t address:5;
            uint16_t command:3;
        } bits;
        uint16_t word;
    };
    MA732Encoder(SPI_TypeDef &regs, GPIO &gpio_cs) : SPIEncoder(regs, gpio_cs) {}
    virtual int32_t read()  __attribute__((section (".ccmram"))) {
        SPIEncoder::read();
        count_ += (int16_t) (data_ - last_data_); // rollover summing
        last_data_ = data_;
        return count_;
    }
    virtual int32_t get_value()  const __attribute__((section (".ccmram"))) { return count_; }
    void init() {
        // filter frequency 1500 Hz
        MA732reg filter;
        filter.bits.command = 0b100;
        filter.bits.address = 0xE;
        filter.bits.value = 85;
        send_and_read(filter.word);
        uint8_t value = send_and_read(0) >> 8;
        if (value != filter.bits.value) {
            while(1); // an error
        }
    }
 private:
    uint16_t last_data_ = 0;
    int32_t count_ = 0;
};
