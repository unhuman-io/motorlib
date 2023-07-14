#pragma once
#include "torque_sensor.h"
#include "logger.h"

class MAX11254 : public TorqueSensorBase {
 public:
    union register_address {
        struct {
            uint8_t rw:1;
            uint8_t addr:5;
            uint8_t bits2:2;
        };
        uint8_t word;
    };
    union command {
        struct {
            uint8_t rate:4;
            uint8_t mode:2;
            uint8_t b6:1;
            uint8_t b7:1;
        };
        uint8_t word;
    };

    MAX11254(SPIDMA &spi_dma) :
        TorqueSensorBase(), spi_dma_(spi_dma) {
        
        //init();
        register_address dr = {.rw = 1, .addr = 14, .bits2 = 3};
        data_out_[0] = dr.word;
    }
    bool init() { 
        register_address stat_read = {.rw = 1, .addr = 0, .bits2 = 3};
        uint8_t data_out[5] = {stat_read.word};
        uint8_t data_in[5];
        spi_dma_.readwrite(data_out, data_in, 5);
        logger.log_printf("max11274 stat: %02x %02x %02x", data_in[2], data_in[3], data_in[4]);
        bool ret_val = true;
        //ret_val &= write_reg(8, 0x40); // in 1
        ret_val &= write_reg(1, 0x3);   // continuous conversion
        ret_val &= write_reg(2, 0x2F);
        ret_val &= write_reg(9, 0x1); //  GPO0 on
        //ret_val &= write_reg24(7, 0x6);


        command conv = {.rate=0b1111, .mode=3, .b7=1};
        spi_dma_.readwrite(&conv.word, data_in, 1);
        return ret_val;
    }

    bool write_reg(uint8_t address, uint8_t value) {
        register_address reg = {.rw = 0, .addr = address, .bits2 = 3};
        uint8_t data_out[3] = {reg.word, value};
        uint8_t data_in[3];
        spi_dma_.readwrite(data_out, data_in, 2);
        reg.rw = 1;
        data_out[0] = reg.word;
        spi_dma_.readwrite(data_out, data_in, 3);
        uint8_t read_value = data_in[2];
        if (read_value != value) {
            logger.log_printf("max11274 register error %d: wrote %02x, read %02x", address, value, read_value);
            return false;
        }
        return true;
    }

    bool write_reg24(uint8_t address, uint32_t value) {
        register_address reg = {.rw = 1, .addr = address, .bits2 = 3};
        uint8_t data_out[5] = {reg.word, (uint8_t) (value>>16 & 0xff), (uint8_t) (value>>8 & 0xff), (uint8_t) (value & 0xff)};
        uint8_t data_in[5];
        spi_dma_.readwrite(data_out, data_in, 4);
        reg.rw = 0;
        data_out[0] = reg.word;
        spi_dma_.readwrite(data_out, data_in, 5);
        uint32_t read_value = data_in[2] << 16 | data_in[3] << 8 | data_in[4];
        if (read_value != value) {
            logger.log_printf("max11274 register error %d: wrote %06x, read %06x", address, value, read_value);
            return false;
        }
        return true;
    }

    void trigger() {
        if (++count_ <= decimation_) {
            return;
        }
        count_ = 0;
        spi_dma_.start_readwrite(data_out_, data_in_, length_);
    }

    float read() {
        if (count_ == 0) {
            spi_dma_.finish_readwrite();
            raw_value_ = data_in_[1] << 16 | data_in_[2] << 8 | data_in_[3];
            if (isol) {
                raw_value_ = (data_in_[2] << 24 | data_in_[3] << 16 | data_in_[4] << 8) >> 8;
            }
            int32_t s32 = raw_value_ << 8;
            signed_value_ = s32 >> 8;
            torque_ = signed_value_ * gain_ + bias_;

        }
        uint8_t data_in;
        command conv = {.rate=0b1111, .mode=3, .b7=1};
        spi_dma_.readwrite(&conv.word, &data_in, 1);
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
