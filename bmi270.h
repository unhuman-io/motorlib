#pragma once

// bit 7: 0 write, 1 read
// bits 0:6: address
// if read 1 byte of 0 follows

class BMI270 {
 public:
    BMI270(SPIDMA &spi_dma) : spi_dma_(spi_dma) {}
    void init() {
        write_reg(0x7D, 0x0E);
        write_reg(0x40, 0xA8);
        write_reg(0x42, 0xA9);
        write_reg(0x7C, 0x02);
    }
    void read() {
        data_out_[0] = 0x80; // chip id

        data_out_[0] = 0x8c;
        data_out_[1] = 0x02;
        spi_dma_.readwrite(data_out_, data_in_, 13);
    }

    void write_reg(uint8_t address, uint8_t value) {
        uint8_t data_out[3] = {address, 0, value};
        uint8_t data_in[3];
        spi_dma_.readwrite(data_out, data_in, 3);
        ns_delay(2000);
        data_out[0] |= 0x80;
        spi_dma_.readwrite(data_out, data_in, 3);
    }
    std::string get_string() const { char s[100]; std::sprintf(s, "0x%02X", data_in_[2]); return s; }
 private:
    SPIDMA &spi_dma_;
    uint8_t data_out_[13] = {};
    uint8_t data_in_[13] = {};
};
