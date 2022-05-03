#pragma once
#include "ma732_encoder.h"

// Note MA782 is similar to MA732 with different registers and such encoder expects cpol 1, cpha 1, max 25 mbit or cpol 0 cpha 0, modes 0 or 3
// 100 ns cs start to sclk, 20 ns sclk end to cs end
class MA782Encoder final : public MA732Encoder {
 public:
    enum MA782FW {_1, _2, _4, _8, _16, _32, _64, _128, _256, _512, _1024, _2048, _4096}; // us
    MA782Encoder(SPI_TypeDef &regs, GPIO &gpio_cs, uint8_t filter = _512, volatile int *register_operation = nullptr) : 
        MA732Encoder(regs, gpio_cs, filter, register_operation) {}
    
    virtual void set_filt(uint32_t value) override {
        set_register(0xE, value << 4);
    }

    // see ma732_encoder get_magnetic_field strength
    uint32_t get_magnetic_field_strength() {
        uint8_t original_mgt = read_register(0x6);
        uint8_t mght = 0, mglt = 0;
        for (uint8_t i=0; i<8; i++) {
            if (!set_register(0x6, i<<2 | i<<5 | 1)) { // increment magnetic field thresholds
                set_register(0x6, original_mgt | 1);
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
        set_register(0x6, original_mgt | 1);
        return (mght << 0 | (uint16_t) mglt << 8);
    }

};
