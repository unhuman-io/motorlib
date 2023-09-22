#ifndef UNHUMAN_MOTORLIB_GPIO_H_
#define UNHUMAN_MOTORLIB_GPIO_H_

#include <cstdint>
#include "st_device.h"    //TODO remove

class GPIO {
 public:
    enum Direction {INPUT, OUTPUT};
    GPIO(GPIO_TypeDef &regs, uint8_t pin, Direction direction = INPUT);
    void set();
    void clear();
    uint8_t get_value() const;
    void set_value(uint8_t value);
    bool is_set() const { return get_value(); }
    bool is_clear() const { return !get_value(); }
    void set_direction(Direction);
 private:
    GPIO_TypeDef &regs_;
    uint32_t mask_;
    uint8_t pin_;
};

class GPIODebounce : public GPIO {
 public:
   GPIODebounce(GPIO_TypeDef &regs, uint8_t pin, Direction direction = INPUT, uint16_t debounce_num = 100) :
      GPIO(regs, pin, direction), debounce_num_(debounce_num) {
         value_ = GPIO::get_value();
      }
   void update();
   uint8_t get_value() const { return value_; }
   bool is_set() const { return get_value(); }
   bool is_clear() const { return !get_value(); }
 private:
   uint16_t debounce_num_;
   uint16_t debounce_count_ = 0;
   uint8_t value_;
   uint8_t next_value_;
};

#endif  // UNHUMAN_MOTORLIB_GPIO_H_
