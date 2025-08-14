#ifndef UNHUMAN_MOTORLIB_MB85RC64_H_
#define UNHUMAN_MOTORLIB_MB85RC64_H_
#include "logger.h"
#include "parameter_api.h"
#include "messages.h"
#include <cstring>
#include "control_fun.h"

class MB85RC64 {
 public:
  union Address {
    struct {
      uint8_t low;
      uint8_t high;
    };
    uint16_t word;
  };
  static constexpr float bit_time_us = 1.5;

  MB85RC64(I2C_DMA& i2c_dma, uint8_t address = 4) : i2c_dma_(i2c_dma) { address_ = address | 0x50; }

  bool init() {
    // attempting reset sequence from datasheet. It's not entirely clear.
    uint8_t data_out = 0xff;
    i2c_dma_.write(address_, 1, &data_out, false, 100);
    i2c_dma_.write(address_, 1, &data_out, true, 100);

    // read device id
    uint8_t address = address_ << 1;
    i2c_dma_.write(0x7c, 1, &address, false, 100);
    uint8_t data_in[3] = {};
    i2c_dma_.read(0x7c, 3, data_in, 100);
    logger.log_printf("MB85RC64: init, data_in: %02x %02x %02x", data_in[0], data_in[1],
                        data_in[2]);
    return data_in[2] == 0x58;
  }

  void read(uint16_t address, uint8_t* bytes, uint16_t length) {
    uint16_t timeout_us1 = 3 * 8 * bit_time_us;
    uint16_t timeout_us2 = (3 + length) * 8 * bit_time_us;
    Address addr = {.word = address};
    uint8_t data[2] = {addr.high, addr.low};
    i2c_dma_.write(address_, 2, data, false, timeout_us1);
    i2c_dma_.read(address_, length, bytes, timeout_us2);
  }

  void write(uint16_t address, const uint8_t* bytes, uint16_t length) {
    uint16_t timeout_us = (3 + length) * 8 * bit_time_us;
    Address addr = {.word = address};
    uint8_t data[2 + length] = {addr.high, addr.low};
    std::memcpy(data + 2, bytes, length);
    i2c_dma_.write(address_, length + 2, data, true, timeout_us);
  }

 private:
  I2C_DMA& i2c_dma_;
  uint8_t address_;
};

// a double buffered block
class FRAMBlock {
 public:
  FRAMBlock(MB85RC64& fram, uint16_t address = 0, uint16_t block_length = 12)
      : fram_(fram), block_length_(block_length) {
    block1_.word = address;
    block2_.word = address + block_length;
  }
  bool init() {
    uint32_t sequence_num1, sequence_num2, tmp;
    bool any_valid_block = false;
    read(block1_.word, reinterpret_cast<uint8_t*>(&sequence_num1), sizeof(sequence_num1));
    read(block1_.word + block_length_ - 4, reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp));
    if (sequence_num1 == tmp) {
      // valid block 1
      any_valid_block = true;
      current_block_ = &block1_;
      sequence_num_ = sequence_num1;
    }
    read(block2_.word, reinterpret_cast<uint8_t*>(&sequence_num2), sizeof(sequence_num2));
    read(block2_.word + block_length_ - 4, reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp));
    if (sequence_num2 == tmp) {
      if (any_valid_block) {
        // valid block 2, but block 1 is also valid, choose the one with higher sequence number
        if (static_cast<int32_t>(sequence_num2 - sequence_num1) > 0) {
          current_block_ = &block2_;
          sequence_num_ = sequence_num2;
        }
      } else {
        // only block 2 is valid
        any_valid_block = true;
        current_block_ = &block2_;
        sequence_num_ = sequence_num2;
      }
    }
    if (!any_valid_block) {
      // no valid block, start with block 1
      current_block_ = &block1_;
      sequence_num_ = 0;
      logger.log("MB85RC64: no valid block found, starting with block 0");
      return false;
    } else {
      logger.log_printf("fram sequence_num start: %u, address %u", sequence_num_,
                        current_block_->word);
    }
    return true;
  }
  void read_block(uint8_t* data, uint16_t length) { read(current_block_->word, data, length); }
  void write_block(const uint8_t* data, uint16_t length) {
    write(current_block_->word, data, length);
  }

 protected:
  void read(uint16_t address, uint8_t* data, uint16_t length) {
    fram_.read(address, (uint8_t*)data, length);
  }
  void write(uint16_t address, const uint8_t* data, uint16_t length) {
    fram_.write(address, (const uint8_t*)data, length);
  }
  void next_block() {
    sequence_num_++;
    if (current_block_ == &block1_) {
      current_block_ = &block2_;
    } else {
      current_block_ = &block1_;
    }
  }

  MB85RC64& fram_;
  uint32_t sequence_num_;
  uint16_t block_length_;
  MB85RC64::Address block1_;
  MB85RC64::Address block2_;
  MB85RC64::Address* current_block_ = &block1_;
};

template <typename T, uint16_t block_length = sizeof(T) + 8>
class TypedFRAMBlock : public FRAMBlock {
 public:
  TypedFRAMBlock(MB85RC64& fram, uint16_t address = 0) : FRAMBlock(fram, address, block_length) {}
  struct Data {
    uint32_t sequence_num1;
    T data;
    uint32_t reserved[(block_length - sizeof(T)) / 4 - 2];
    uint32_t sequence_num2;
  } data;
  static_assert(block_length == sizeof(Data));
  bool init() {
    if (!FRAMBlock::init()) {
      // no valid block found, initialize data
      std::memset(&data, 0, sizeof(Data));
      return false;
    }
    read();
    return true;
  }
  T& read() {
    read_block(reinterpret_cast<uint8_t*>(&data), sizeof(Data));
    return get_data();
  }
  void write() {
    next_block();
    data.sequence_num1 = sequence_num_;
    data.sequence_num2 = sequence_num_;
    write_block(reinterpret_cast<const uint8_t*>(&data), sizeof(Data));
  }
  T& get_data() { return data.data; }
};

// non double buffered stream of data
// prefixed by a 4 byte pointer to the current end of the log
class FRAMLog {
 public:
  FRAMLog(MB85RC64& fram, uint16_t address, uint16_t size)
      : fram_(fram), address_(address), log_max_(size - 4) {}

  void write_log(const uint8_t* bytes, uint8_t length) {
    uint32_t log_pointer;
    fram_.read(address_.word, reinterpret_cast<uint8_t*>(&log_pointer), sizeof(log_pointer));
    if (log_pointer + length >= log_max_) {
      log_pointer = 0;
    }
    uint32_t new_log_pointer = log_pointer + length;
    fram_.write(address_.word + log_pointer + 4, bytes, length);
    fram_.write(address_.word, reinterpret_cast<const uint8_t*>(&new_log_pointer),
                sizeof(log_pointer));
  }

  std::string get_log() const {
    char c[log_max_];
    uint32_t read_length = log_max_;
    uint32_t read_ptr = 0;
    while (read_length > 255) {
      fram_.read(address_.word + 4 + read_ptr, (uint8_t*)c + read_ptr, 255);
      read_length -= 255;
      read_ptr += 255;
    }
    if (read_length > 0) {
      fram_.read(address_.word + 4 + read_ptr, (uint8_t*)c + read_ptr, read_length);
    }
    std::string s(c, log_max_);
    return s;
  }

 private:
  MB85RC64& fram_;
  const MB85RC64::Address address_;
  const uint16_t log_max_;
};

class StandardMB85RC64 {
 public:
  struct FRAM1 {
    uint32_t total_uptime_s;
    uint32_t enabled_time_s;
    uint32_t revolution_count;
    uint32_t total_energy_j;
  };
  StandardMB85RC64(MB85RC64& fram)
      : fram_(fram), fram_block1_(fram), fram_log_(fram, 0x200, 2048) {}

  void init(ParameterAPI& api) {
    fram_block1_.init();
    logger.log_printf("total_uptime_start: %u", fram1_.total_uptime_s);
    total_uptime_start_ = fram1_.total_uptime_s;
    logger.log_printf("revolution_count: %u", fram1_.revolution_count);
    logger.log_printf("enabled_time: %u", fram1_.enabled_time_s);
    logger.log_printf("total_energy_j: %u", fram1_.total_energy_j);
    {
      std::string s = "startup at " + std::to_string(fram1_.total_uptime_s) + "\n";
      fram_log_.write_log((uint8_t*)s.c_str(), s.size());
    }
    api.add_api_variable("total_uptime", new const APIUint32(&fram1_.total_uptime_s));
    api.add_api_variable("revolution_count", new const APIUint32(&fram1_.revolution_count));
    api.add_api_variable("total_enabled_time", new const APIUint32(&fram1_.enabled_time_s));
    api.add_api_variable("total_energy_j", new const APIUint32(&fram1_.total_energy_j));
  }

  void update(const MainLoopStatus& status) {
    if (first_run_) {
      first_run_ = false;
      last_fast_loop_timestamp_ = status.fast_loop.timestamp;
      last_energy_uJ_ = status.fast_loop.energy_uJ;
      output_position_hyst_.set_value(status.output_position);
    }
    fram1_.total_uptime_s = total_uptime_start_ + get_uptime();
    float new_output_position = output_position_hyst_.step(status.output_position);

    accumulated_output_position_ += fabsf(new_output_position - last_output_position_);
    if (accumulated_output_position_ > 2 * M_PI) {
      accumulated_output_position_ -= 2 * M_PI;
      fram1_.revolution_count++;
    }
    last_output_position_ = new_output_position;

    float diff_uj = std::abs((int32_t)(status.fast_loop.energy_uJ - last_energy_uJ_));
    sum_uj_ += diff_uj;
    if (sum_uj_ >= 1'000'000) {
      fram1_.total_energy_j += 1;
      sum_uj_ -= 1'000'000;
    }
    last_energy_uJ_ = status.fast_loop.energy_uJ;

    if (status.fast_loop.mode != 0 && status.fast_loop.mode != 1) {
      sum_enabled_ += status.fast_loop.timestamp - last_fast_loop_timestamp_;
      if (sum_enabled_ > CPU_FREQUENCY_HZ) {
        fram1_.enabled_time_s += 1;
        sum_enabled_ -= CPU_FREQUENCY_HZ;
      }
    }
    last_fast_loop_timestamp_ = status.fast_loop.timestamp;

    fram_block1_.write();

    if (status.error.fault && !last_fault) {
      char s[100];
      std::sprintf(s, "fault detected, error: %08lx\n", status.error.all);
      fram_log_.write_log((uint8_t*)s, std::strlen(s));
    }
    last_fault = status.error.fault;
  }

  MB85RC64& fram_;
  TypedFRAMBlock<FRAM1, 0x20> fram_block1_;
  FRAM1& fram1_ = fram_block1_.get_data();
  FRAMLog fram_log_;
  uint32_t total_uptime_start_;

 private:
  bool last_fault = false;
  uint32_t last_fast_loop_timestamp_ = 0;
  uint32_t last_energy_uJ_ = 0;
  uint32_t sum_uj_ = 0;
  uint32_t sum_enabled_ = 0;
  float last_output_position_ = 0;
  float accumulated_output_position_ = 0;
  Hysteresis output_position_hyst_{0.01};
  bool first_run_ = true;
};

#endif  // UNHUMAN_MOTORLIB_MB85RC64_H_
