#ifndef UNHUMAN_MOTORLIB_MB85RC64_H_
#define UNHUMAN_MOTORLIB_MB85RC64_H_
#include "logger.h"

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

    void init() {
      uint32_t sequence_num1, sequence_num2, tmp;
      read(block1_.word, &sequence_num1);
      read(block1_.word + block_length_ - 4, &tmp);
      if (sequence_num1 == tmp) {
         // valid block 1
         current_block_ = &block1_;
         sequence_num_ = sequence_num1;
      }
      read(block2_.word, &sequence_num2);
      read(block2_.word + block_length_ - 4, &tmp);
      if (sequence_num2 == tmp) {
         if (sequence_num2 > sequence_num1) {
            current_block_ = &block2_;
            sequence_num_ = sequence_num2;
         }
      }

      logger.log_printf("fram sequence_num start: %u, block %u", sequence_num_, current_block_->word);
    }

    void read_block(uint16_t address, uint32_t *value) {
      read(current_block_->word + 4 + address, value);
    }

    void write_block(uint16_t address, uint32_t value) {
      write(current_block_->word + 4 + address, value);
    }

    void next_block() {
      write_sequence_num();
      sequence_num_++;
      if (current_block_ == &block1_) {
         current_block_ = &block2_;
      } else {
         current_block_ = &block1_;
      }
    }

    void write_sequence_num() {
      write(current_block_->word, sequence_num_);
      write(current_block_->word + block_length_ - 4, sequence_num_);
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
    uint32_t sequence_num_ = 0;
    const Address block1_ = {.word = 0};
    const uint16_t block_length_ = 0x100;
    const Address block2_ = {.word = (uint16_t) (block1_.word + block_length_)};
    const Address *current_block_ = &block1_;
 
};

#endif // UNHUMAN_MOTORLIB_MB85RC64_H_