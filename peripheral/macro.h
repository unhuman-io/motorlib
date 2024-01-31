#pragma once

#define DEBUG_PINS_ENABLE

#ifndef FIGURE_ASSERT
#define FIGURE_ASSERT(x, ...) \
do {                          \
  if (!(x)) {                 \
    while(1);                 \
  }                           \
} while(0)
#endif // #ifndef FIGURE_ASSERT

#ifndef FIGURE_COUNTOF
#define FIGURE_COUNTOF(array) (sizeof(array)/sizeof((array)[0]))
#endif // #ifndef FIGURE_COUNTOF

#ifdef DEBUG_PINS_ENABLE

#define DEBUG_PORT GPIOA
#define DEBUG_PIN1 11
#define DEBUG_PIN2 11
#define DEBUG_PIN3 11

#define CONCATENATE_(A, B, C) A##B##C
#define CONCATENATE(A, B, C) CONCATENATE_(A, B, C)


#define DEBUG_PINS_INIT() do{\
  DEBUG_PORT->AFR[1] &= ~(CONCATENATE(GPIO_AFRH_AFSEL, DEBUG_PIN1, _Msk) \
                        | CONCATENATE(GPIO_AFRH_AFSEL, DEBUG_PIN2, _Msk) \
                        | CONCATENATE(GPIO_AFRH_AFSEL, DEBUG_PIN3, _Msk) ); /* Set AF to 0 for GPIO mode */ \
  \
  DEBUG_PORT->MODER &= ~(CONCATENATE(GPIO_MODER_MODE, DEBUG_PIN1, _Msk) \
                       | CONCATENATE(GPIO_MODER_MODE, DEBUG_PIN2, _Msk) \
                       | CONCATENATE(GPIO_MODER_MODE, DEBUG_PIN3, _Msk)); \
  \
  DEBUG_PORT->MODER |=  (CONCATENATE(GPIO_MODER_MODE, DEBUG_PIN1, _0) \
                       | CONCATENATE(GPIO_MODER_MODE, DEBUG_PIN2, _0) \
                       | CONCATENATE(GPIO_MODER_MODE, DEBUG_PIN3, _0)); /* Output mode */ \
  \
  DEBUG_PORT->ODR &= ~(CONCATENATE(GPIO_ODR_OD, DEBUG_PIN1, ) \
                     | CONCATENATE(GPIO_ODR_OD, DEBUG_PIN2, ) \
                     | CONCATENATE(GPIO_ODR_OD, DEBUG_PIN3, )); \
}while(0)

// PC10
#define DEBUG_PIN1_SET()   do{DEBUG_PORT->ODR |= CONCATENATE(GPIO_ODR_OD, DEBUG_PIN1,);}while(0)
#define DEBUG_PIN1_CLEAR() do{DEBUG_PORT->ODR &= ~CONCATENATE(GPIO_ODR_OD, DEBUG_PIN1,);}while(0)

// PC12
#define DEBUG_PIN2_SET()   do{DEBUG_PORT->ODR |= CONCATENATE(GPIO_ODR_OD, DEBUG_PIN2,);}while(0)
#define DEBUG_PIN2_CLEAR() do{DEBUG_PORT->ODR &= ~CONCATENATE(GPIO_ODR_OD, DEBUG_PIN2,);}while(0)

// PC11
#define DEBUG_PIN3_SET()   do{DEBUG_PORT->ODR |= CONCATENATE(GPIO_ODR_OD, DEBUG_PIN3,);}while(0)
#define DEBUG_PIN3_CLEAR() do{DEBUG_PORT->ODR &= ~CONCATENATE(GPIO_ODR_OD, DEBUG_PIN3,);}while(0)

#else

#define DEBUG_PINS_INIT()
#define DEBUG_PIN1_SET()
#define DEBUG_PIN1_CLEAR()

// PC12
#define DEBUG_PIN2_SET()
#define DEBUG_PIN2_CLEAR()

// PC11
#define DEBUG_PIN3_SET()
#define DEBUG_PIN3_CLEAR()

#endif
