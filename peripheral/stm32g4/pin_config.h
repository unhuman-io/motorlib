#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_PIN_CONFIG_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_PIN_CONFIG_H_

#define MASK_SET(var, item, val) var = (var & ~item##_Msk) | (val << item##_Pos)
#define GPIO_SETL(gpio, pin, mode, speed, af)                     \
  MASK_SET(GPIO##gpio->MODER, GPIO_MODER_MODE##pin, mode);        \
  MASK_SET(GPIO##gpio->OSPEEDR, GPIO_OSPEEDR_OSPEED##pin, speed); \
  MASK_SET(GPIO##gpio->AFR[0], GPIO_AFRL_AFSEL##pin, af)

#define GPIO_SETH(gpio, pin, mode, speed, af)                     \
  MASK_SET(GPIO##gpio->MODER, GPIO_MODER_MODE##pin, mode);        \
  MASK_SET(GPIO##gpio->OSPEEDR, GPIO_OSPEEDR_OSPEED##pin, speed); \
  MASK_SET(GPIO##gpio->AFR[1], GPIO_AFRH_AFSEL##pin, af)

enum GPIO_MODE { INPUT, OUTPUT, ALT_FUN, ANALOG };
enum GPIO_SPEED { LOW, MEDIUM, HIGH, VERY_HIGH };
enum GPIO_PULL { NONE, UP, DOWN };
enum GPIO_OTYPE { PUSHPULL, OPEN_DRAIN };

union gpio_bits {
  struct {
    volatile uint32_t bit0 : 1;
    volatile uint32_t bit1 : 1;
    volatile uint32_t bit2 : 1;
    volatile uint32_t bit3 : 1;
    volatile uint32_t bit4 : 1;
    volatile uint32_t bit5 : 1;
    volatile uint32_t bit6 : 1;
    volatile uint32_t bit7 : 1;
    volatile uint32_t bit8 : 1;
    volatile uint32_t bit9 : 1;
    volatile uint32_t bit10 : 1;
    volatile uint32_t bit11 : 1;
    volatile uint32_t bit12 : 1;
    volatile uint32_t bit13 : 1;
    volatile uint32_t bit14 : 1;
    volatile uint32_t bit15 : 1;
    volatile uint32_t bit16 : 1;
    volatile uint32_t bit17 : 1;
    volatile uint32_t bit18 : 1;
    volatile uint32_t bit19 : 1;
    volatile uint32_t bit20 : 1;
    volatile uint32_t bit21 : 1;
    volatile uint32_t bit22 : 1;
    volatile uint32_t bit23 : 1;
    volatile uint32_t bit24 : 1;
    volatile uint32_t bit25 : 1;
    volatile uint32_t bit26 : 1;
    volatile uint32_t bit27 : 1;
    volatile uint32_t bit28 : 1;
    volatile uint32_t bit29 : 1;
    volatile uint32_t bit30 : 1;
    volatile uint32_t bit31 : 1;
  };
  volatile uint32_t word;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_PIN_CONFIG_H_
