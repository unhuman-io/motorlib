module;

#include "../../logger.h"
#include "../../gpio.h"
#include "../../peripheral/spi_dma.h"

export module ma782_encoder;

import ma7xx_encoder;

export using ::MA7XXEncoderBase;

// Note MA782 is similar to MA732 with different registers and such encoder expects cpol 1, cpha 1, max 25 mbit or cpol 0 cpha 0, modes 0 or 3
// 100 ns cs start to sclk, 20 ns sclk end to cs end
export template<typename SPI>
class MA782Encoder : public MA7XXEncoderBase<MA782Encoder<SPI>, SPI> {
 public:
    enum MA782FW {_1, _2, _4, _8, _16, _32, _64, _128, _256, _512, _1024, _2048, _4096}; // us
    MA782Encoder(SPI &spi, GPIO &gpio_cs, SPIPause &spi_pause, uint8_t filter = _512) : 
        MA7XXEncoderBase<MA782Encoder<SPI>, SPI>(spi, gpio_cs, spi_pause, filter) {}
    
    void set_filt_impl(uint32_t value) {
        this->set_register(0xE, value << 4);
    }

    uint32_t get_filt_impl() {
        return this->read_register(0xE) >> 4;
    }

    // see ma7xx_encoder get_magnetic_field strength
    // difference is set_register(0x6, original_mgt | 1);
    uint32_t get_magnetic_field_strength_impl() {
        uint8_t original_mgt = this->read_register(0x6);
        uint8_t mght = 0, mglt = 0;
        for (uint8_t i=0; i<8; i++) {
            logger.log("mgt");
            if (!this->set_register(0x6, i<<2 | i<<5 | 1)) { // increment magnetic field thresholds
                this->set_register(0x6, original_mgt | 1);
                return 0xFFFF; // a fail test value
            }
            uint8_t test_mgt = this->read_register(0x1B);
            char s[50];
            std::sprintf(s, "test_mgt: 0x%2x", test_mgt);
            logger.log(s);
            // find last value that mght is 1 and the last that mglt is 0
            // I think they should be equal
            if (test_mgt & 0x80) {
                mght = i+1;
            }
            if (!(test_mgt & 0x40)) {
                mglt = i+1;
            }
        }
        this->set_register(0x6, original_mgt | 1);
        logger.log_printf("mgt: 0x%04x", mght << 0 | (uint16_t) mglt << 8);
        return (mght << 0 | (uint16_t) mglt << 8);
    }

        // difference to ma7xx is send_and_read(0) & 0xFF
    uint8_t read_register_impl(uint8_t address) {
        typename MA7XXEncoderBase<MA782Encoder<SPI>, SPI>::MA7XXreg reg = {};
        reg.bits.address = address;
        reg.bits.command = 0b010; // read register
        this->send_and_read(reg.word);
        ns_delay(750); // read register delay
        return this->send_and_read(0) & 0xFF;
    }

};
