#pragma once
#include "logger.h"
#include "bmi270_data.c"

// bit 7: 0 write, 1 read
// bits 0:6: address
// if read 1 byte of 0 follows

extern "C" {
void system_init();
}
class BMI270 {
 public:
    struct bmi270_data {
        int16_t acc_x, acc_y, acc_z;
        int16_t gyr_x, gyr_y, gyr_z;
    };
    BMI270(SPIDMA &spi_dma) : spi_dma_(spi_dma) {
        register_operation_ = spi_dma_.register_operation_;
    }
    void init() {
        write_reg(0x7C, 0x00);
        us_delay(450);
        write_reg(0x59, 0x00);
        burst_write(0x5E, bmi270_config_file, sizeof(bmi270_config_file));
        write_reg(0x59, 0x01);

        ms_delay(20);
        uint8_t init_status = read_reg(0x21);
        logger.log_printf("bmi270 init status: %02x", init_status);

        write_reg(0x7D, 0x0E);
        write_reg(0x40, 0xAA);  // high performance osr2? filter, output rate 400 Hz
        write_reg(0x42, 0xE9);
        write_reg(0x7C, 0x02);
    }

    void read(bool register_operation = false) {
        //data_out_[0] = 0x80; // chip id

        data_out_[0] = 0x8c;
        spi_dma_.readwrite(data_out_, data_in_, 14, register_operation);
        // logger.log(bytes_to_hex(data_in_, 14));
        data_.acc_x = (int16_t) (data_in_[3] << 8 | data_in_[2]);
        data_.acc_y = (int16_t) (data_in_[5] << 8 | data_in_[4]);
        data_.acc_z = (int16_t) (data_in_[7] << 8 | data_in_[6]);
        data_.gyr_x = (int16_t) (data_in_[9] << 8 | data_in_[8]);
        data_.gyr_y = (int16_t) (data_in_[11] << 8 | data_in_[10]);
        data_.gyr_z = (int16_t) (data_in_[13] << 8 | data_in_[12]);
        // logger.log_printf("ax: %0.3f, ay: %0.3f, az: %0.3f, gx: %d, gy: %d, gz: %d",
        //     data_.acc_x*8./pow(2,15), data_.acc_y*8./pow(2,15), data_.acc_z*8./pow(2,15),
        //     data_.gyr_x, data_.gyr_y, data_.gyr_z);
    }

    void read_with_restore() {
        (*register_operation_)++;
        spi_dma_.save_state();
        read(true);
        spi_dma_.restore_state();
        (*register_operation_)--;
    }

    void burst_write(uint8_t address, const uint8_t data[], uint16_t length) {
        (*register_operation_)++;
        uint8_t data_out[1] = {address};
        uint8_t data_in[1];
        spi_dma_.start_readwrite(data_out, data_in, 1, true);
        us_delay(2);
        spi_dma_.start_write(data, length, true);
        spi_dma_.finish_readwrite(true);
        us_delay(3);
        (*register_operation_)--;
    }

    void write_reg(uint8_t address, uint8_t value) {
        (*register_operation_)++;
        uint8_t data_out[2] = {address, value};
        uint8_t data_in[2];
        spi_dma_.readwrite(data_out, data_in, 2, true);
        us_delay(3);
        (*register_operation_)--;
    }

    uint8_t read_reg(uint8_t address) {
        (*register_operation_)++;
        uint8_t data_out[3] = {(uint8_t) (address | 0x80)};
        uint8_t data_in[3];
        spi_dma_.readwrite(data_out, data_in, 3, true);
        us_delay(3);
        (*register_operation_)--;
        return data_in[2];
    }

    std::string get_string() const { char s[100]; std::sprintf(s, "0x%02X", data_in_[2]); return s; }

    volatile int *register_operation_;
 private:
    SPIDMA &spi_dma_;
    uint8_t data_out_[14] = {};
    uint8_t data_in_[14] = {};
    bmi270_data data_;
    friend void config_init();
    friend void system_init();
};
