#ifndef UNHUMAN_MOTORLIB_SENSORS_ENCODERS_AEAT9922_H_
#define UNHUMAN_MOTORLIB_SENSORS_ENCODERS_AEAT9922_H_

#include "../../encoder.h"
#include "../../peripheral/stm32g4/spi_dma.h"
#include "../../logger.h"

// An "18 bit" on or off axis single pole magnetic encoder from Broadcom
template<uint8_t isol_ = 1>
class AEAT9922 : public EncoderBase {
 public:
    AEAT9922(SPIDMA &spidma) : EncoderBase(), spidma_(spidma) {}
    bool init() {
        bool success = true;

        // status check
        uint8_t data_out[2] = {0x40, 0x21};
        uint8_t data_in[2];
        spidma_.readwrite(data_out, data_in, 2, true);
        if (isol_) {
            us_delay(2);
        }
        spidma_.readwrite(read_out_, read_in_, 3+isol_);
        if (read_in_[1+isol_] != 0x80) {
            success = false;
            logger.log_printf("aeat9922 status error reg 0x21, expected 0x80, read 0x%02X",
                read_in_[1+isol_]);
        }

        // Set into 24 bit mode
        // uint8_t data_out[2] = {0x80, 0x0B};
        // uint8_t data_in[2];
        // spidma_.readwrite(data_out, data_in, 2, true);
        // data_out[1] = 0x20;
        // spidma_.readwrite(data_out, data_in, 2, true);

        
        // success &= set_register(0x8, 0); // set hysteresis to 0
        return success;
    }

    void trigger() {
        spidma_.start_readwrite(read_out_, read_in_, 3+isol_);
    }

    int32_t read() {
        spidma_.finish_readwrite();
        
        // 24 bit
        // raw_value_ = read_in_[0+isol_] << 12 | read_in_[1+isol_] << 4 | read_in_[2+isol_] >> 4; // 20 bits
        // uint8_t crc_read = read_in_[2+isol_] << 4 | read_in_[3+isol_] >> 4;
        // uint8_t crc_calc = crc((uint8_t *) &raw_value_);
        // if (crc_calc != crc_read) {
        //     crc_error_count_++;
        // } // else if
        // if (raw_value_ & 0x40000) {
        //     error_count_++;
        // } else {
        //     raw_encoder_ = (raw_value_ & 0x3FFFF) << 14;
        //     int32_t diff = raw_encoder_ - last_raw_encoder_;
        //     last_raw_encoder_ = raw_encoder_;
        //     accumulated_value_ += (diff >> 14);
        // }

        // if (raw_value_ & 0x80000) {
        //     warning_count_++;
        // }
        
        // 16 bit
        raw_value_ = read_in_[0+isol_] << 12 | read_in_[1+isol_] << 4 | read_in_[2+isol_] >> 4;
        if (raw_value_ & 0x40000) {
            error_count_++;
        } else {
            raw_encoder_ = (raw_value_ & 0x3FFFF) << 14;
            int32_t diff = raw_encoder_ - last_raw_encoder_;
            last_raw_encoder_ = raw_encoder_;
            accumulated_value_ += (diff >> 14);
        }



        return get_value();
    }

    int32_t get_value() const { 
        return accumulated_value_;
    }

    bool index_received() const { return true; }

    bool set_register(uint8_t address, uint8_t value) {
        uint8_t data_out[3+isol_] = {0, address};
        data_out[2] = crc(data_out);

        uint8_t data_in[3+isol_];
        // write address
        spidma_.readwrite(data_out, data_in, 3, true);
        // write data
        data_out[1] = value;
        data_out[2] = 0;
        data_out[2] = crc(data_out);
        spidma_.readwrite(data_out, data_in, 3, true);
        // read
        data_out[0] = 0x40;
        spidma_.readwrite(data_out, data_in, 3+isol_, true);
        if (data_in[1+isol_] != data_out[1]) {
            return false;
        }
        return true;
    }
    uint8_t crc(uint8_t data[3]) {
        return 0;
    }
    void clear_faults() {
        error_count_ = 0;
        warning_count_ = 0;
        crc_error_count_ = 0;
    }
    uint32_t crc_error_count_ = 0;
    uint32_t error_count_ = 0;
    uint32_t warning_count_ = 0;   
    uint32_t raw_value_;
    uint32_t raw_encoder_;

    SPIDMA &spidma_;
    uint8_t read_out_[3+isol_] = {0x40, 0x3F};
    uint8_t read_in_[3+isol_];
    uint32_t last_raw_encoder_ = 0;
    int32_t accumulated_value_ = 0;
};

#endif // UNHUMAN_MOTORLIB_SENSORS_ENCODERS_AEAT9922_H_
