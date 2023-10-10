#ifndef UNHUMAN_MOTORLIB_MB85RC64_H_
#define UNHUMAN_MOTORLIB_MB85RC64_H_
#include "logger.h"
#include <cstring>

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

    void write_log(const uint8_t *bytes, uint8_t length) {
      uint32_t log_pointer;
      read(log_address_.word, &log_pointer);
      if (log_pointer+length >= log_max_) {
         log_pointer = 0;
      }
      uint32_t new_log_pointer = log_pointer + length;
      write(log_address_.word + log_pointer + 4, bytes, length);
      write(log_address_.word, new_log_pointer);
    }

    std::string get_log() {
      char c[255];
      read(log_address_.word+4, (uint8_t*) c, 255);
      std::string s(c);
      return s;
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

   void read(uint16_t address, uint8_t *bytes, uint8_t length) {
      Address addr = {.word = address};
      uint8_t data[2] = {addr.high, addr.low};
      i2c_dma_.write(address_, 2, data, false);
      i2c_dma_.read(address_, length, bytes);
    }

    void write(uint16_t address, uint32_t value) {
      Address addr = {.word = address};
      Uint32 word = {.word = value};
      uint8_t data[6] = {addr.high, addr.low, word.byte4, word.byte3, word.byte2, word.byte1};
      i2c_dma_.write(address_, 6, data, true);
    }

    void write(uint16_t address, const uint8_t *bytes, uint8_t length) {
      Address addr = {.word = address};
      uint8_t data[2+length] = {addr.high, addr.low};
      std::memcpy(data+2, bytes, length);
      i2c_dma_.write(address_, length+2, data, true);
    }
 private:
    I2C_DMA &i2c_dma_;
    uint8_t address_;
    uint32_t sequence_num_ = 0;
    const Address block1_ = {.word = 0};
    const uint16_t block_length_ = 0x100;
    const Address block2_ = {.word = (uint16_t) (block1_.word + block_length_)};
    const Address *current_block_ = &block1_;
    const Address log_address_ = {.word = (uint16_t) (block2_.word + block_length_)};
    //const uint16_t log_max_ = 0x2000 - 4 - log_address_.word; // todo: figure out longer log
    const uint16_t log_max_ = 256;
 
};

#endif // UNHUMAN_MOTORLIB_MB85RC64_H_