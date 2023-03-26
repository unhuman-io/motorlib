#include "gpio.h"

GPIO::GPIO(GPIO_TypeDef &regs, uint8_t pin, Direction direction)
    : regs_(regs), mask_(1 << pin), pin_(pin) {
  set_direction(direction);
}

void GPIO::set() { regs_.BSRR |= mask_; }

void GPIO::clear() { regs_.BSRR |= mask_ << 16; }

uint8_t GPIO::get_value() const { return (regs_.IDR & mask_) >> pin_; }

void GPIO::set_value(uint8_t value) { regs_.ODR |= value << pin_; }

// TODO
void GPIO::set_direction(Direction direction) {
  switch (direction) {
    default:
    case INPUT:
      break;
    case OUTPUT:
      break;
  }
}

void GPIODebounce::update() {
  uint8_t value = GPIO::get_value();
  if (value != next_value_) {
    debounce_count_ = 0;
    next_value_ = value;
  } else {
    debounce_count_++;
  }

  if (debounce_count_ >= debounce_num_) {
    value_ = next_value_;
  }
}