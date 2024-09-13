#ifndef UNHUMAN_MOTORLIB_RESOLUTE_ENCODER_H_
#define UNHUMAN_MOTORLIB_RESOLUTE_ENCODER_H_

#include "encoder.h"
#include <cmath>

#define RESOLUTE_SET_DEBUG_VARIABLES(prefix, api, resolute) \
    api.add_api_variable(prefix "err", new APIUint32(&resolute.diag_err_count_));\
    api.add_api_variable(prefix "warn", new APIUint32(&resolute.diag_warn_count_));\
    api.add_api_variable(prefix "crc_cnt", new APIUint32(&resolute.crc_err_count_));\
    api.add_api_variable(prefix "raw", new APIUint32(&resolute.raw_value_));\
    api.add_api_variable(prefix "rawh", new const APICallback([](){ return u32_to_hex(resolute.raw_value_); }));\
    api.add_api_variable(prefix "len", new APIUint8(&resolute.length_));\
    api.add_api_variable(prefix "ind", new const APIUint8(&resolute.byte_ind));\
    api.add_api_variable(prefix "crc_calc", new const APIUint8(&resolute.crc_calc_));\
    api.add_api_variable(prefix "zeros", new const APIUint32(&resolute.leading_zeros));\
    api.add_api_variable(prefix "diag", new const APIUint8(&resolute.diag_raw_.word));\

static uint8_t CRC_BiSS_43_36bit(uint64_t w_InputData);

class ResoluteEncoder : public EncoderBase {
 public:
    union Diag {
        struct {
            uint8_t crc6:6;
            uint8_t warn:1;
            uint8_t err:1;
        };
        uint8_t word;
    };
    union DiagProcessed {   // 0b111 is good
        struct {
            uint8_t crc6:1;
            uint8_t warn:1;
            uint8_t err:1;
            uint8_t resv:5;
        };
        uint8_t word;
    };
    // BISS, 5 MHz max
    ResoluteEncoder(SPIDMA &spi_dma) : EncoderBase(), spi_dma_(spi_dma) {}
    void trigger() {
        //spi_dma_.reinit();
        spi_dma_.start_readwrite_isr(data_out_, data_in_, length_);
    }
    bool init() { return true; }
    int32_t read() {
        GPIOC->BSRR = GPIO_BSRR_BS0;
        spi_dma_.finish_readwrite_isr();
        GPIOC->BSRR = GPIO_BSRR_BR0;

        uint64_t super_raw;
        // variable ack length. Find start bit. Skip a few bytes to save time
        
        for (int i=4; i<length_; i++) {
            if (data_in_[i]) {
                super_raw = (((uint64_t) data_in_[i]) & 0xFF) << 48 | (((uint64_t) data_in_[i+1]) & 0xFF) << 40 | (((uint64_t) data_in_[i+2]) & 0xFF) << 32  
                    | (uint64_t) data_in_[i+3] << 24 | (uint64_t) data_in_[i+4] << 16 | (uint64_t) data_in_[i+5] << 8 | (uint64_t) data_in_[i+6];
                leading_zeros = clz(data_in_[i]);
                byte_ind = i;
                break;
            }
        }
        // super_raw is 64 bits with >=8 bits of leading zeros
        // e.g. 00 02 FF FF FF FF 3F 00
        //          s| position  |e|crc
        // above clz returns 30. right shift to data is 16 = 48 - 2 - 30
        raw_value_ = super_raw >> (48 - 2 - leading_zeros);
        Diag diag;
        diag.word = (super_raw >> (40 - 2 - leading_zeros)) & 0xFF;
        diag_.err = diag.err;
        diag_.warn = diag.warn;
        uint64_t crc_val_raw = (super_raw >> (46 - 2 -leading_zeros)) & 0x3FFFFFFFF;
        crc_calc_ = ~CRC_BiSS_43_36bit(crc_val_raw) & 0x3F; // crc of data plus 2 status bits
        diag_.crc6 = 1;
        diag_.crc6 = crc_calc_ == diag.crc6;
        if (!diag_.crc6) {
            crc_err_count_++;
        } else {
            if (!diag_.err) {
                diag_err_count_++;
            }
            if (!diag_.warn) {
                diag_warn_count_++;
            }
        }

        if (diag_.crc6 && diag_.err) {
            int32_t diff = (int32_t) (raw_value_ - last_raw_value_);
            value_ += diff;
            last_raw_value_ = raw_value_;
        }

        diag_raw_ = diag;
        return get_value();
    }
    uint32_t clz(uint32_t val) {
        uint32_t zeros;
        asm("clz %[zeros], %[val]": [zeros] "=r" (zeros) : [val] "r" (val));
        return zeros;
    }
    void clear_faults() {
        crc_err_count_ = 0;
        diag_err_count_ = 0;
        diag_warn_count_ = 0;
    }
    int32_t get_value() const { return value_; }
    bool index_received() { return true; }
    DiagProcessed diag_ = {};
    uint8_t crc_calc_ = {};
    Diag diag_raw_ = {};
    uint32_t crc_err_count_ = 0;
    uint32_t diag_err_count_ = 0;
    uint32_t diag_warn_count_ = 0;
    uint32_t raw_value_ = 0;
    uint32_t leading_zeros=0;
    uint8_t byte_ind=0;

 private:
    SPIDMA &spi_dma_;
    uint8_t length_ = 16;
    uint8_t data_out_[20] = {};
    uint8_t data_in_[20] = {};
    int32_t value_ = 0;
    uint32_t last_raw_value_ = 0;

    friend void config_init();
};


// CRC6 calc from RLS CRCD01
uint8_t tableCRC6[64] = {
 0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
 0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
 0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
 0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};
/*32-bit input data, right alignment, Calculation over 36 bits (mult. of 6) */
uint8_t CRC_BiSS_43_36bit (uint64_t w_InputData)
{
 uint8_t b_Index = 0;
 uint8_t b_CRC = 0;

 b_Index = (uint8_t )(((uint64_t)w_InputData >> 30u) & 0x0000003Fu);

 b_CRC = (uint8_t )(((uint64_t)w_InputData >> 24u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];
 b_CRC = (uint8_t )(((uint64_t)w_InputData >> 18u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )(((uint64_t)w_InputData >> 12u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )(((uint64_t)w_InputData >> 6u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )((uint64_t)w_InputData & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = tableCRC6[b_Index];

 return b_CRC;
} 

#endif  // UNHUMAN_MOTORLIB_RESOLUTE_ENCODER_H_
