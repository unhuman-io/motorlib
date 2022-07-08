#pragma once


#define MASK_SET(var, item, val) var = (var & ~item##_Msk) | (val << item##_Pos)
#define GPIO_SETL(gpio, pin, mode, speed, af) \
    MASK_SET(GPIO##gpio->MODER, GPIO_MODER_MODE##pin, mode); \
    MASK_SET(GPIO##gpio->OSPEEDR, GPIO_OSPEEDR_OSPEED##pin, speed); \
    MASK_SET(GPIO##gpio->AFR[0], GPIO_AFRL_AFSEL##pin, af)

#define GPIO_SETH(gpio, pin, mode, speed, af) \
    MASK_SET(GPIO##gpio->MODER, GPIO_MODER_MODE##pin, mode); \
    MASK_SET(GPIO##gpio->OSPEEDR, GPIO_OSPEEDR_OSPEED##pin, speed); \
    MASK_SET(GPIO##gpio->AFR[1], GPIO_AFRH_AFSEL##pin, af)

enum GPIO_MODE {INPUT, OUTPUT, ALT_FUN, ANALOG};
enum GPIO_SPEED {LOW, MEDIUM, HIGH, VERY_HIGH};
enum GPIO_PULL {NONE, UP, DOWN};
enum GPIO_OTYPE {PUSHPULL, OPEN_DRAIN};

