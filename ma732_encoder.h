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

    virtual void trigger()  __attribute__((section (".ccmram"))) {
        if (!register_operation_) {
            SPIEncoder::trigger();
        }
    }   
    virtual int32_t read()  __attribute__((section (".ccmram"))) {
        if (!register_operation_) {
            SPIEncoder::read();
            count_ += (int16_t) (data_ - last_data_); // rollover summing
            last_data_ = data_;
        }
        return count_;
    }

    uint8_t read_register(uint8_t address) {
        register_operation_++;
        MA732reg reg = {};
        reg.bits.address = address;
        reg.bits.command = 0b010; // read register
        send_and_read(reg.word);
        ns_delay(750); // read register delay
        uint8_t retval = send_and_read(0) >> 8;
        register_operation_--;
        return retval;
    }

    bool set_register(uint8_t address, uint8_t value) {
        register_operation_++;
        bool retval = true;
        if (read_register(address) != value) {
            MA732reg reg = {};
            reg.bits.address = address;
            reg.bits.command = 0b100; // write register
            reg.bits.value = value;
            send_and_read(reg.word);
            ms_delay(20); 
            retval = read_register(address) == value;
        }
        register_operation_--;
        return retval;
    }

    void set_bct(uint32_t value) {
        set_register(0x2, value);
    }

    uint32_t get_bct() {
        return read_register(0x2);
    }

    void set_et(uint32_t value) {
        set_register(0x3, value);
    }

    uint32_t get_et() {
        return read_register(0x3);
    }

    void set_mgt(uint32_t value) {
        set_register(0x6, value);
    }

    uint32_t get_magnetic_field_strength() {
        uint8_t original_mgt = read_register(0x6);
        uint8_t mght = 0, mglt = 7;
        for (uint8_t i=0; i<8; i++) {
            set_register(0x6, i<<2 | i<<5); // increment magnetic field thresholds
            uint8_t test_mgt = read_register(0x1B);    
            // find last value that mght is 1 and the last that mglt is 0
            // I think they should be equal
            if (test_mgt & 0x80) {
                mght = i;
            }
            if (!(test_mgt & 0x40)) {
                mglt = i;
            }
        }
        set_register(0x6, original_mgt);
        return (mght << 2 | mglt << 5);
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
    volatile int register_operation_ = 0;
    uint32_t tmp;
};
