#ifndef UNHUMAN_MOTORLIB_MA732_ENCODER_H_
#define UNHUMAN_MOTORLIB_MA732_ENCODER_H_

#include "peripheral/spi_encoder.h"
#include "util.h"
#include "logger.h"
#include "peripheral/spi_dma.h"

// Note MA732 encoder expects cpol 1, cpha 1, max 25 mbit
// 80 ns cs start to sclk, 25 ns sclk end to cs end
class MA732Encoder : public SPIEncoder {
 public:
    union MA732reg {
        struct {
            uint16_t value:8;
            uint16_t address:5;
            uint16_t command:3;
        } bits;
        uint16_t word;
    };
    MA732Encoder(SPI_TypeDef &regs, GPIO &gpio_cs, SPIPause &spi_pause, uint8_t filter = 119) : SPIEncoder(regs, gpio_cs), 
        filter_(filter), regs_(regs), spi_pause_(spi_pause) {
        reinit();
    }

    void reinit() {
#ifdef STM32F446xx
        regs_.CR1 = SPI_CR1_MSTR | (3 << SPI_CR1_BR_Pos) | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE | SPI_CR1_DFF;    // baud = clock/16, 16 bit
#else    
        regs_.CR2 = (15 << SPI_CR2_DS_Pos);   // 16 bit
        regs_.CR1 = SPI_CR1_MSTR | (3 << SPI_CR1_BR_Pos) | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE;    // baud = clock/16
#endif
    }

    // interrupt context
    void trigger()  __attribute__((section (".ccmram"))) {
        if (!spi_pause_.is_paused()) {
            SPIEncoder::trigger();
        }
    }

    // interrupt context   
    int32_t read()  __attribute__((section (".ccmram"))) {
        if (!spi_pause_.is_paused()) {
            SPIEncoder::read();
            count_ += (int16_t) (data_ - last_data_); // rollover summing
            last_data_ = data_;
        }
        return count_;
    }

    // non interrupt context
    virtual uint8_t read_register(uint8_t address) {
        spi_pause_.pause();
        reinit(); // only really necessary if there are multiple users of the spi
        MA732reg reg = {};
        reg.bits.address = address;
        reg.bits.command = 0b010; // read register
        send_and_read(reg.word);
        ns_delay(750); // read register delay
        uint8_t retval = send_and_read(0) >> 8;
        spi_pause_.unpause();
        return retval;
    }

    // non interrupt context
    virtual bool set_register(uint8_t address, uint8_t value) {
        reinit();
        spi_pause_.pause();
        bool retval = true;
        if (read_register(address) != value) {
            MA732reg reg = {};
            reg.bits.address = address;
            reg.bits.command = 0b100; // write register
            reg.bits.value = value;
            send_and_read(reg.word);
            ms_delay(20); 
            uint8_t read_value = read_register(address);
            retval = read_value == value;
            if (!retval) {
                logger.log_printf("ma732 set reg %x: %x, read %x", address, value, read_value);
            }
        }
        spi_pause_.unpause();
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

    uint32_t get_filt() {
        return read_register(0xE);
    }

    virtual void set_filt(uint32_t value) {
        set_register(0xE, value);
    }

    void set_mgt(uint32_t value) {
        set_register(0x6, value);
    }

    // The MA732 encoder doesn't give magnetic field strength directly but allows 
    // you to set high and low thresholds in the 0x6 MGT register, then you can read
    // the 0x1B status register to determine if the field is within those thresholds 
    // or not. The full range of the MGT register is 20 to 126 mT in 8 steps. Recommended
    // min mT is 40 which is step 2 in MGT. I combine the two readings like this
    // (mght << 0 | (uint16_t) mglt << 8), so the minimum recommended value is about
    // 0x202 or 514.
    uint32_t get_magnetic_field_strength() {
        uint8_t original_mgt = read_register(0x6);
        uint8_t mght = 0, mglt = 0;
        for (uint8_t i=0; i<8; i++) {
            if (!set_register(0x6, i<<2 | i<<5)) { // increment magnetic field thresholds
                set_register(0x6, original_mgt);
                return 0xFFFF; // a fail test value
            }
            uint8_t test_mgt = read_register(0x1B);    
            // find last value that mght is 1 and the last that mglt is 0
            // I think they should be equal
            if (test_mgt & 0x80) {
                mght = i+1;
            }
            if (!(test_mgt & 0x40)) {
                mglt = i+1;
            }
        }
        set_register(0x6, original_mgt);
        return (mght << 0 | (uint16_t) mglt << 8);
    }

    int32_t get_value()  const __attribute__((section (".ccmram"))) { return count_; }

    bool init() {
        // filter frequency
        bool success = set_register(0xE, filter_);
        return success;
    }

 protected:
    uint8_t filter_;
    SPI_TypeDef &regs_;
    uint16_t last_data_ = 0;
    int32_t count_ = 0;
    SPIPause &spi_pause_;
    uint32_t tmp;
};

#endif  // UNHUMAN_MOTORLIB_MA732_ENCODER_H_
