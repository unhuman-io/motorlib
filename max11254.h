#pragma once
#include "torque_sensor.h"
#include "logger.h"

template<bool isolation=true>
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
    union cr1_register {
        struct {
            uint8_t contsc:1;
            uint8_t scycle:1;
            uint8_t format:1;
            uint8_t ub:1;
            uint8_t pd:2;
            uint8_t cal:2;
        };
        uint8_t word;
    };

    union cr2_register {
        struct {
            uint8_t pga:3;
            uint8_t pgaen:1;
            uint8_t lpmode:1;
            uint8_t ldoen:1;
            uint8_t cssen:1;
            uint8_t extclk:1;
        };
        uint8_t word;
    };
    union chmap_register {
        struct {
            uint8_t gpoen:1;
            uint8_t en:1;
            uint8_t ord:3;
            uint8_t gpio:2;
        };
        uint8_t word;
    };
    union chmap24_register {
        struct {
            chmap_register ch0, ch1, ch2;
        };
        uint32_t word;
    };
    union seq_register {
        struct {
            uint8_t rdyben:1;
            uint8_t mdren:1;
            uint8_t gpodren:1;
            uint8_t mode:2;
            uint8_t mux:3;
        };
        uint8_t word;
    };

    MAX11254(SPIDMA &spi_dma, uint8_t decimation=1) :
        TorqueSensorBase(), spi_dma_(spi_dma), decimation_(decimation) {
        
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
        // reset sequence
        ret_val &= write_reg(1, 0x30); // reset
        command conv = {.rate=0b1111, .mode=1, .b7=1};
        spi_dma_.readwrite(&conv.word, data_in, 1);
        ms_delay(28);
        spi_dma_.readwrite(data_out, data_in, 5);
        logger.log_printf("max11274 stat: %02x %02x %02x", data_in[2], data_in[3], data_in[4]);

        cr1_register cr1 = {.scycle=1, .format=1};
        ret_val &= write_reg(1, cr1.word);   // single conversion
        // pga128
        cr2_register cr2 = {.pga=7, .pgaen=1, .ldoen=1};
        // pga off option
        //cr2_register cr2 = {.ldoen=1};
        ret_val &= write_reg(2, cr2.word);
        ret_val &= write_reg(9, 0x1); //  GPO0 on
        // sample channel 1
        //seq_register seq = {.mux=1};
        //ret_val &= write_reg(8, seq.word);

        spi_dma_.readwrite(data_out, data_in, 5);
        logger.log_printf("max11274 stat: %02x %02x %02x", data_in[2], data_in[3], data_in[4]);

        command conv2 = {.rate=0b1100, .mode=3, .b7=1}; // 8khz rate
        spi_dma_.readwrite(&conv2.word, data_in, 1);

        spi_dma_.readwrite(data_out, data_in, 5);
        logger.log_printf("max11274 stat: %02x %02x %02x", data_in[2], data_in[3], data_in[4]);
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
            if (isolation) {
                raw_value_ = data_in_[2] << 16 | data_in_[3] << 8 | data_in_[4];
            }
            signed_value_ = raw_value_ - 0x7FFFFF;
            torque_ = signed_value_ * gain_ + bias_;

            // stale values are currently the only fault mechanism
            if (last_new_signed_value_ != signed_value_) {
                last_new_signed_value_ = signed_value_;
                stale_value_count_ = 0;
            } else {
                stale_value_count_++;
                if (stale_value_count_ > 3) {
                    timeout_error_++;
                    stale_value_count_ = 0;
                }
            }
            if (raw_value_ == 0 || raw_value_ == 0xFFFFFF) {
                read_error_++;
            }

        }
        uint8_t data_in;
        command conv = {.rate=0b1111, .mode=3, .b7=1};
        spi_dma_.readwrite(&conv.word, &data_in, 1);
        return torque_;
    }

    void clear_faults() {
        timeout_error_ = 0;
        stale_value_count_ = 0;
        read_error_ = 0;
    }
    uint8_t count_ = 0;

    int32_t signed_value_ = 0;
    int32_t last_new_signed_value_ = 0;
    uint8_t stale_value_count_ = 0;
    uint32_t timeout_error_ = 0;
    uint32_t read_error_ = 0;
    uint32_t raw_value_ = 0;
    SPIDMA &spi_dma_;
    static const uint8_t length_ = 5;
    uint8_t data_out_[length_] = {};
    uint8_t data_in_[length_] = {};
    uint8_t decimation_ = 1;
};
