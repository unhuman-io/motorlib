#ifndef UNHUMAN_MOTORLIB_MA7XX_ENCODER_H_
#define UNHUMAN_MOTORLIB_MA7XX_ENCODER_H_

#include "../../peripheral/spi_encoder.h"
#include "../../util.h"
#include "../../logger.h"
#include "../../peripheral/spi_dma.h"

#define MA7XX_SET_DEBUG_VARIABLES(prefix, api, ma7xx) \
    api.add_api_variable(prefix "err", new APIUint32(&ma7xx.error_count_));\
    api.add_api_variable(prefix "filt", new APICallbackUint8([]{ return ma7xx.get_filt(); }, \
        [](uint8_t u){ ma7xx.set_filt(u); }));\
    api.add_api_variable(prefix "bct", new APICallbackUint8([]{ return ma7xx.get_bct(); }, \
        [](uint8_t u){ ma7xx.set_bct(u); }));\
    api.add_api_variable(prefix "et", new APICallbackUint8([]{ return ma7xx.get_et(); }, \
        [](uint8_t u){ ma7xx.set_et(u); }));\
    api.add_api_variable(prefix "mgt", new APICallbackHex<uint16_t>([]{ return ma7xx.get_magnetic_field_strength(); }, \
        [](uint8_t u){ ma7xx.set_mgt(u); }));\
    api.add_api_variable(prefix "raw", new const APIUint16(&ma7xx.data_));\

// Note MA7XX encoder expects cpol 1, cpha 1, max 25 mbit
// 80 ns cs start to sclk, 25 ns sclk end to cs end
template <class T>
class MA7XXEncoderBase : public SPIEncoder {
 public:
    union MA7XXreg {
        struct {
            uint16_t value:8;
            uint16_t address:5;
            uint16_t command:3;
        } bits;
        uint16_t word;
    };
    MA7XXEncoderBase(SPI_TypeDef &regs, GPIO &gpio_cs, SPIPause &spi_pause, uint8_t filter = 119) : SPIEncoder(regs, gpio_cs), 
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
    void trigger() {
        if (!spi_pause_.is_paused()) {
            SPIEncoder::trigger();
        }
    }

    // interrupt context   
    int32_t read() {
        if (!spi_pause_.is_paused()) {
            SPIEncoder::read();
            if (data_ == last_data_) {
                stall_count_++;
                if (stall_count_ > stall_count_max_) {
                    error_count_++;
                    stall_count_ = 0;
                }
            } else {
                stall_count_ = 0;
            }

            count_ += (int16_t) (data_ - last_data_); // rollover summing
            last_data_ = data_;
        }
        return count_;
    }

    // non interrupt context
    uint8_t read_register(uint8_t address) {
        spi_pause_.pause();
        reinit(); // only really necessary if there are multiple users of the spi
        uint8_t retval = static_cast<T*>(this)->read_register_impl(address);
        spi_pause_.unpause();
        return retval;
    }

    uint8_t read_register_impl(uint8_t address) {
        MA7XXreg reg = {};
        reg.bits.address = address;
        reg.bits.command = 0b010; // read register
        send_and_read(reg.word);
        ns_delay(750); // read register delay
        return send_and_read(0) >> 8;
    }


    // non interrupt context
    bool set_register(uint8_t address, uint8_t value) {
        reinit();
        spi_pause_.pause();
        bool retval = true;
        if (read_register(address) != value) {
            MA7XXreg reg = {};
            reg.bits.address = address;
            reg.bits.command = 0b100; // write register
            reg.bits.value = value;
            send_and_read(reg.word);
            ms_delay(20); 
            uint8_t read_value = read_register(address);
            retval = read_value == value;
            if (!retval) {
                logger.log_printf("ma7xx set reg %x: %x, read %x", address, value, read_value);
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
        return static_cast<T*>(this)->get_filt_impl();
    }

    uint32_t get_filt_impl() {
        return read_register(0xE);
    }

    void set_filt(uint32_t value) {
        return static_cast<T*>(this)->set_filt_impl(value);
    }

    void set_filt_impl(uint32_t value) {
        set_register(0xE, value);
    }

    void set_mgt(uint32_t value) {
        set_register(0x6, value);
    }

    uint32_t get_magnetic_field_strength() {
        spi_pause_.pause();
        reinit();
        uint32_t retval = static_cast<T*>(this)->get_magnetic_field_strength_impl();
        spi_pause_.unpause();
        return retval;
    }

    // The MA7XX encoder doesn't give magnetic field strength directly but allows 
    // you to set high and low thresholds in the 0x6 MGT register, then you can read
    // the 0x1B status register to determine if the field is within those thresholds 
    // or not. The full range of the MGT register is 20 to 126 mT in 8 steps. Recommended
    // min mT is 40 which is step 2 in MGT. I combine the two readings like this
    // (mght << 0 | (uint16_t) mglt << 8), so the minimum recommended value is about
    // 0x202 or 514.
    uint32_t get_magnetic_field_strength_impl() {
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

    int32_t get_value()  const { return count_; }

    bool check_magnetic_field_strength() {
        bool success = true;
        uint32_t field_strength = get_magnetic_field_strength();
        if (field_strength < 0x202) {
            logger.log_printf("ma7XX magnetic field strength too low: %x", field_strength);
            success = false;
        } else if (field_strength > 0x606) {
            logger.log_printf("ma7XX magnetic field strength too high: %x", field_strength);
            success = false;
        } else {
            logger.log_printf("ma7XX magnetic field strength ok: %x", field_strength);}
        return success;
    }

    bool init() {
        set_filt(filter_);
        bool success = get_filt() == filter_;
        success &= check_magnetic_field_strength();
        return success;
    }

    void clear_faults() {
        error_count_ = 0;
    }

    uint8_t filter_;
    SPI_TypeDef &regs_;
    uint16_t last_data_ = 0;
    int32_t count_ = 0;
    SPIPause &spi_pause_;
    uint32_t stall_count_ = 0;
    uint32_t error_count_ = 0;
    uint32_t stall_count_max_ = 50;
};

class MA732Encoder : public MA7XXEncoderBase<MA732Encoder> {
 public:
    MA732Encoder(SPI_TypeDef &regs, GPIO &gpio_cs, SPIPause &spi_pause, uint8_t filter = 119)
        : MA7XXEncoderBase(regs, gpio_cs, spi_pause, filter) {}
};

class MA730Encoder : public MA732Encoder {
 public:
    MA730Encoder(SPI_TypeDef& s, GPIO& g, SPIPause &sp) : MA732Encoder(s, g, sp) {
        // 23 Hz filter has lots of repeated values
        stall_count_max_ = 1000;
    }
    // don't set filter, it is fixed at 23 Hz
    bool init() { return check_magnetic_field_strength(); }
};


#endif  // UNHUMAN_MOTORLIB_MA7XX_ENCODER_H_
