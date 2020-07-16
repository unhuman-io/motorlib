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
    MA732Encoder(SPI_TypeDef &regs, GPIO &gpio_cs, uint8_t filter = 119) : SPIEncoder(regs, gpio_cs), filter_(filter) {}
    virtual int32_t read()  __attribute__((section (".ccmram"))) {
        SPIEncoder::read();
        count_ += (int16_t) (data_ - last_data_); // rollover summing
        last_data_ = data_;
        return count_;
    }

    uint8_t read_register(uint8_t address) const {
        MA732reg reg = {};
        reg.bits.address = address;
        reg.bits.command = 0b010; // read register
        const_cast<MA732Encoder*>(this)->send_and_read(reg.word);
        ns_delay(750); // read register delay
        return const_cast<MA732Encoder*>(this)->send_and_read(0) >> 8;
    }

    bool set_register(uint8_t address, uint8_t value) {
        if (read_register(address) != value) {
            MA732reg reg = {};
            reg.bits.address = address;
            reg.bits.command = 0b100; // write register
            reg.bits.value = value;
            send_and_read(reg.word);
            ms_delay(20); 
            return read_register(address) == value;
        }
        return true;
    }

    void set_k(uint32_t value) {
        set_register(0x2, value);
    }

    uint32_t get_k() const {
        return read_register(0x2);
    }

    virtual int32_t get_value()  const __attribute__((section (".ccmram"))) { return count_; }

    bool init() {
        // filter frequency
        bool success = set_register(0xE, filter_);
        return success;
    }

 private:
    uint8_t filter_;
    uint16_t last_data_ = 0;
    int32_t count_ = 0;
};
