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
    bool init() {
        // filter frequency 1500 Hz
        MA732reg filter;
        uint8_t desired_filter = 85;
        filter.bits.command = 0b010; // read register
        filter.bits.address = 0xE;
        filter.bits.value = 0;
        send_and_read(filter.word);
        ns_delay(750); // read register delay
        uint8_t value = send_and_read(0) >> 8;
        if (desired_filter != value) {
            // uncomment below to write the register
            // filter.bits.command = 0b100; // write register
            // filter.bits.value = desired_filter;
            send_and_read(filter.word);
            ms_delay(20); // 20 ms delay for idle time to register readout 
            value = send_and_read(0) >> 8;
            if (value != desired_filter) {
                return false;
            }
        }
        return true;
    }
 private:
    uint16_t last_data_ = 0;
    int32_t count_ = 0;
};
