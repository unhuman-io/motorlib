#pragma once

uint8_t CRC_BiSS_43_24bit (uint32_t w_InputData);

class BISSBitBang {
 public:
    enum {ERROR_ACK = -1, ERROR_CRC = -2};
    BISSBitBang(GPIO &clk, GPIO &dat, int nbits) : clk_(clk), dat_(dat), nbits_(nbits) {}
    int transfer_one(bool cdm_out, bool &cds_in, uint32_t &value) {
        for (int i=0; i<(nbits_+preamble_bits_+6); i++) {
            clk_.clear();
            raw_ |= dat_.get_value() << i; // reverse stored
            ns_delay(200);
            clk_.set();
            ns_delay(200);
        }
        clk_.set_value(cdm_out);
        us_delay(21);
        clk_.set();

        // parse the data
        bool ack_bit = raw_ & (1 << preamble_bits_);
        bool start_bit = raw_ & (1 << (preamble_bits_+1));
        if (ack_bit || !start_bit) {
            return ERROR_ACK;
        }
        cds_in = raw_ & (1 << (preamble_bits_+2));
        for (int i=0; i<nbits_; i++) {
            value |= (raw_ & (1 << (i+preamble_bits_+3))) << i;
        }
        bool nerr = raw_ & (1 << (nbits_+preamble_bits_+4));
        bool nwarn = raw_ & (1 << (nbits_+preamble_bits_+5));
        uint8_t crc = 0;
        for (int i=0; i<6; i++) {
            crc |= (raw_ & (1 << (i+nbits_+preamble_bits_+6))) << i;
        }
        if (crc != CRC_BiSS_43_24bit(value << 2 | nerr << 1 | nwarn)) {
            return ERROR_CRC;
        }

        return raw_;
    }

 private:
    GPIO &clk_, &dat_;
    int nbits_;
    uint32_t raw_;
    int preamble_bits_ = 2;
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
/*32-bit input data, right alignment, Calculation over 24 bits (mult. of 6) */
uint8_t CRC_BiSS_43_24bit (uint32_t w_InputData)
{
 uint8_t b_Index = 0;
 uint8_t b_CRC = 0;

 b_Index = (uint8_t )(((uint32_t)w_InputData >> 18u) & 0x0000003Fu);

 b_CRC = (uint8_t )(((uint32_t)w_InputData >> 12u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )(((uint32_t)w_InputData >> 6u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )((uint32_t)w_InputData & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = tableCRC6[b_Index];

 return b_CRC;
} 
