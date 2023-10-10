#ifndef UNHUMAN_MOTORLIB_MB85RC64_H_
#define UNHUMAN_MOTORLIB_MB85RC64_H_

class MB85RC64 {
 public:
    union Address {
      struct {
         uint8_t low;
         uint8_t high;
      };
      uint16_t word;
    };

   union Uint32 {
      struct {
         uint8_t byte1;
         uint8_t byte2;
         uint8_t byte3;
         uint8_t byte4;
      };
      uint32_t word;
    };

    MB85RC64(I2C_DMA &i2c_dma, uint8_t address = 4) : i2c_dma_(i2c_dma) {
      address_ = address | 0x50;
    }
    void read(uint16_t address, uint32_t *value) {
      Address addr = {.word = address};
      uint8_t data[2] = {addr.high, addr.low};
      i2c_dma_.write(address_, 2, data, false);
      uint8_t data_out[4];
      i2c_dma_.read(address_, 4, data_out);
      Uint32 word = {.byte1 = data_out[3], .byte2 = data_out[2], .byte3 = data_out[1], .byte4 = data_out[0]};
      *value = word.word;
    }

    void write(uint16_t address, uint32_t value) {
      Address addr = {.word = address};
      Uint32 word = {.word = value};
      uint8_t data[6] = {addr.high, addr.low, word.byte4, word.byte3, word.byte2, word.byte1};
      i2c_dma_.write(address_, 6, data, true);
    }
 private:
    I2C_DMA &i2c_dma_;
    uint8_t address_;
 
};

#endif // UNHUMAN_MOTORLIB_MB85RC64_H_