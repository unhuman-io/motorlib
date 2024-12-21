#pragma once
#include <stdio.h>
#include "biss_crc.h"

class BISSBitBang {
 public:
    enum {ERROR_ACK = 1, ERROR_CRC = 2, WARNING = 4, ERROR = 8};
    BISSBitBang(GPIO &clk, GPIO &dat, int nbits) : clk_(clk), dat_(dat), nbits_(nbits) {}
    int read_register(uint8_t addr) {
        bool cds_in;
        uint32_t value;
        int err = 0;
        for (int i=0; i<14; i++) {
            err |= transfer_one(false, cds_in, value);
        }
        err |= transfer_one(true, cds_in, value); // start bit
        err |= transfer_one(true, cds_in, value); // cts bit -> register communication
        uint8_t id = 0; // 3 bits
        for (int i=0; i<3; i++) {
            err |= transfer_one((id >> (2-i)) & 1, cds_in, value);
        }
        for (int i=0; i<7; i++) {
            err |= transfer_one((addr >> (6-i)) & 1, cds_in, value);
        }
        uint8_t crc = ~biss_crc4(1<<10 | addr); // 7; // for 0: 0x6;
        for (int i=0; i<4; i++) {
            err |= transfer_one((crc >> (3-i)) & 1, cds_in, value);
        }
        err |= transfer_one(true, cds_in, value); // R
        err |= transfer_one(false, cds_in, value); // W
        err |= transfer_one(true, cds_in, value); // S
        err |= transfer_one(true, cds_in, value); // S response
        uint8_t data = 0;
        for (int i=0; i<8; i++) {
            err |= transfer_one(false, cds_in, value);
            data |= cds_in << (7-i);
        }
        uint8_t crc_in = 0;
        for (int i=0; i<4; i++) {
            err |= transfer_one(false, cds_in, value);
            crc_in |= cds_in << (3-i);
        }
        crc_in = ~crc_in & 0xf;
        err |= transfer_one(false, cds_in, value); // P
        if (crc_in != biss_crc4(data)) {
            err |= ERROR_CRC;
        }
        printf("err: %d, data: %x, crc: %x\n", err, data, crc_in);
        return data;
    }

    int write_register(uint8_t addr, uint8_t data) {
        bool cds_in;
        uint32_t value;
        int err = 0;
        for (int i=0; i<14; i++) {
            err |= transfer_one(false, cds_in, value);
        }
        err |= transfer_one(true, cds_in, value); // start bit
        err |= transfer_one(true, cds_in, value); // cts bit -> register communication
        uint8_t id = 0; // 3 bits
        for (int i=0; i<3; i++) {
            err |= transfer_one((id >> (2-i)) & 1, cds_in, value);
        }
        for (int i=0; i<7; i++) {
            err |= transfer_one((addr >> (6-i)) & 1, cds_in, value);
        }
        uint8_t crc = ~biss_crc4(1<<10 | addr); // 7; // for 0: 0x6;
        for (int i=0; i<4; i++) {
            err |= transfer_one((crc >> (3-i)) & 1, cds_in, value);
        }
        err |= transfer_one(false, cds_in, value); // R
        err |= transfer_one(true, cds_in, value); // W
        err |= transfer_one(true, cds_in, value); // S
        for (int i=0; i<8; i++) {
            err |= transfer_one((data >> (7-i)) & 1, cds_in, value);
        }
        crc = ~biss_crc4(data);
        for (int i=0; i<4; i++) {
            err |= transfer_one((crc >> (3-i)) & 1, cds_in, value);
        }
        err |= transfer_one(false, cds_in, value); // P
        printf("err: %d, data: %x\n", err, data);
        return data;
    }

    int biss_command() {
        bool cds_in;
        uint32_t value;
        int ids = 0;
        int err = 0;
        for (int i=0; i<14; i++) {
            err |= transfer_one(false, cds_in, value);
        }
        err |= transfer_one(true, cds_in, value); // start bit
        err |= transfer_one(false, cds_in, value); // cts bit -> biss command
        for (int i=0; i<8; i++) {
            err |= transfer_one((ids >> (6-i)) & 1, cds_in, value);
        }
        uint8_t cmd = 1; // 2 bits
        for (int i=0; i<2; i++) {
            err |= transfer_one((cmd >> (1-i)) & 1, cds_in, value);
        }

        uint8_t crc = 0xc;
        for (int i=0; i<4; i++) {
            err |= transfer_one((crc >> (3-i)) & 1, cds_in, value);
        }
        err |= transfer_one(true, cds_in, value); // S
        err |= transfer_one(true, cds_in, value); // EX

        printf("err: %d\n", err);
        return 0;
    }
    int transfer_one(bool cdm_out, bool &cds_in, uint32_t &value) {
        value = 0;
        raw_ = 0;
        cds_in = false;
        int retval = 0;
        clk_.set();
        ns_delay(200);
        for (int i=0; i<(preamble_bits_ + 3 + nbits_ + 2 + 6); i++) {
            clk_.clear();
            raw_ |= dat_.get_value() << i; // reverse stored
            ns_delay(200);
            clk_.set();
            ns_delay(200);
        }
        clk_.set_value(!cdm_out);
        us_delay(21);
        //clk_.set();
        //us_delay(21); // not sure how long idle needs to be

        // parse the data
        bool ack_bit = raw_ & (1 << preamble_bits_);
        bool start_bit = raw_ & (1 << (preamble_bits_+1));
        if (ack_bit || !start_bit) {
            retval |= ERROR_ACK;
        }
        cds_in = raw_ & (1 << (preamble_bits_+2));
        for (int i=0; i<nbits_; i++) {
            uint8_t bit_pos = i + preamble_bits_ + 3;
            uint8_t shift = nbits_ - i - 1;
            value |= (bool) (raw_ & (1 << bit_pos)) << shift;
        }
        bool nerr = raw_ & (1 << (nbits_+preamble_bits_+3));
        if (!nerr) {
            retval |= ERROR;
        }
        bool nwarn = raw_ & (1 << (nbits_+preamble_bits_+4));
        if (!nwarn) {
            retval |= WARNING;
        }
        uint8_t crc = 0;
        for (int i=0; i<6; i++) {
            uint8_t bit_pos = i + preamble_bits_ + nbits_ + 5;
            uint8_t shift = 6 - i - 1;
            crc |= (bool) (raw_ & (1 << bit_pos)) << shift;
        }
        if (crc != (~CRC_BiSS_43_24bit(value << 2 | nerr << 1 | nwarn) & 0x3f)) {
            retval |= ERROR_CRC;
        }

        return retval;
    }

 private:
    GPIO &clk_, &dat_;
    int nbits_;
    uint32_t raw_;
    int preamble_bits_ = 2;
};
