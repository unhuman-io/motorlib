#pragma once
#include "torque_sensor.h"
#include "logger.h"

class MAX11270 : public TorqueSensorBase {
 public:
    union register_address {
        struct {
            uint8_t rw:1;
            uint8_t addr:5;
            uint8_t bits2:2;
        };
        uint8_t word;
    };
    union conversion {
        struct {
            uint8_t rate:4;
            uint8_t impd:1;
            uint8_t cal:1;
            uint8_t mode:1;
            uint8_t start:1;
        };
        uint8_t word;
    };

    MAX11270(SPIDMA &spi_dma) :
        TorqueSensorBase(), spi_dma_(spi_dma) {
        
        register_address stat_read = {.rw = 1, .addr = 0, .bits2 = 3};
        uint8_t data_out[3] = {stat_read.word};
        uint8_t data_in[3];
        spi_dma_.readwrite(data_out, data_in, 3);
        logger.log_printf("max11270 stat: %02x %02x", data_in[1], data_in[2]);
        register_address dr = {.rw = 1, .addr = 6, .bits2 = 3};
        data_out_[0] = dr.word;
    }
    bool init() { 
        register_address stat_read = {.rw = 1, .addr = 0, .bits2 = 3};
        uint8_t data_out[3] = {stat_read.word};
        uint8_t data_in[3];
        spi_dma_.readwrite(data_out, data_in, 3);
        logger.log_printf("max11270 stat: %02x %02x", data_in[1], data_in[2]);
        return true; }
    void trigger() {
        if (++count_ <= decimation_) {
            return;
        }
        count_ = 0;
        spi_dma_.start_readwrite_isr(data_out_, data_in_, length_);
    }
    float read() {
        if (count_ == 0) {
            spi_dma_.finish_readwrite_isr();
            raw_value_ = data_in_[1] << 16 | data_in_[2] << 8 | data_in_[3];
            if (isol) {
                raw_value_ = (data_in_[2] << 24 | data_in_[3] << 16 | data_in_[4] << 8) >> 8;
            }
            int32_t s32 = raw_value_ << 8;
            signed_value_ = s32 >> 8;
            torque_ = signed_value_ * (2.5 / pow(2,23)) / .02085;
            uint8_t data_in;
            conversion conv = {.rate=0b1111, .start=1};
            spi_dma_.readwrite(&conv.word, &data_in, 1);
        }
        return torque_;
    }
    bool isol = true;
    uint8_t count_ = 0;
    uint8_t decimation_ = 1;
    int32_t signed_value_ = 0;
    uint32_t raw_value_ = 0;
 private:
    static const uint8_t length_ = 5;
    uint8_t data_out_[length_] = {};
    uint8_t data_in_[length_] = {};
    SPIDMA &spi_dma_;
};
