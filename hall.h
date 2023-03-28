#ifndef UNHUMAN_MOTORLIB_HALL_H_
#define UNHUMAN_MOTORLIB_HALL_H_

#include "encoder.h"

class GPIO;
#include "gpio.h"

class HallEncoder : public EncoderBase {
 public:
  HallEncoder(GPIO &gpio_a, GPIO &gpio_b, GPIO &gpio_c)
      : gpio_a_(gpio_a), gpio_b_(gpio_b), gpio_c_(gpio_c) {}

  bool init() { return true; }

  int32_t read() {
    uint8_t hall_read = gpio_a_.get_value() | (gpio_b_.get_value() << 1) |
                        (gpio_c_.get_value() << 2);
    uint8_t hall_count = hall_table_[hall_read];
    if (hall_count != 0) {
      int8_t diff = hall_count - last_hall_count_;
      last_hall_count_ = hall_count;
      if (diff < -3) {
        diff += 6;
      } else if (diff > 3) {
        diff -= 6;
      }
      count_ += diff;
    }
    return count_;
  }
  int32_t get_value() const { return count_; }

 private:
  GPIO &gpio_a_;
  GPIO &gpio_b_;
  GPIO &gpio_c_;
  int32_t count_ = 0;
  uint8_t last_hall_count_ = 0;
  static uint8_t hall_table_[8];
};

#endif  // UNHUMAN_MOTORLIB_HALL_H_
