#pragma once

#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx.h"

#include "../macro.h"
#include "../comms.h"

#define SPI_DEBUG_PINS_INIT() do{       \
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;  \
  GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL12_Msk); /* Set AF to 0 for GPIO mode */ \
  GPIOC->MODER &= ~(GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE12_Msk); \
  GPIOC->MODER |= (GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0); /* Output mode */ \
  GPIOC->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12); \
}while(0)

#define SPI_DEBUG_PIN1_SET()   do{GPIOC->ODR |= GPIO_ODR_OD10;}while(0)
#define SPI_DEBUG_PIN1_CLEAR() do{GPIOC->ODR &= ~GPIO_ODR_OD10;}while(0)

#define SPI_DEBUG_PIN2_SET()   do{GPIOC->ODR |= GPIO_ODR_OD12;}while(0)
#define SPI_DEBUG_PIN2_CLEAR() do{GPIOC->ODR &= ~GPIO_ODR_OD12;}while(0)

#define SPI_DEBUG_PIN3_SET()   do{GPIOC->ODR |= GPIO_ODR_OD11;}while(0)
#define SPI_DEBUG_PIN3_CLEAR() do{GPIOC->ODR &= ~GPIO_ODR_OD11;}while(0)

class SpiSlave : public Comms
{
  public:
    static SpiSlave* instance;

    struct InitStruct
    {
      SPI_TypeDef*   spi;
      GPIO_TypeDef*  gpioPort;
      uint8_t        gpioPinSs;
      uint8_t        gpioPinSck;
      uint8_t        gpioPinMosi;
      uint8_t        gpioPinMiso;
      uint8_t        gpioAlternateFunction;
      __IO uint32_t* gpioRccEnableRegister;
      uint8_t        gpioRccEnableBit;
      __IO uint32_t* spiRccEnableRegister;
      uint8_t        spiRccEnableBit;
      __IO uint32_t* spiRccResetRegister;
      uint8_t        spiRccResetBit;

      DMA_TypeDef*            rxDma;
      uint32_t                rxDmaIfcrCgif;
      DMA_Channel_TypeDef*    rxDmaChannel;
      DMAMUX_Channel_TypeDef* rxDmaMuxChannel;
      uint8_t                 rxDmaMuxId;
      IRQn_Type               rxDmaIrqN;
      uint8_t                 rxDmaIrqPriority;

      DMA_TypeDef*            txDma;
      uint32_t                txDmaIfcrCgif;
      DMA_Channel_TypeDef*    txDmaChannel;
      DMAMUX_Channel_TypeDef* txDmaMuxChannel;
      uint8_t                 txDmaMuxId;
      IRQn_Type               txDmaIrqN;
      uint8_t                 txDmaIrqPriority;
    };

    SpiSlave(const InitStruct& init_struct);
    ~SpiSlave();

    // Comms API
    void init() final;
    void reset() final;
    void startTransaction(BufferDescriptor descriptor) final;
    void setTransactionCompletedCallback(transactionCompleteCallback callback, void* param) final;

    void resetTransactionCounter() final;
    uint8_t getTransactionCounter() final;

    // These two have to be public so they can be called them from the ISR handler
    void rxInterruptHandler();
    void txInterruptHandler();

  private:
    const InitStruct init_struct_;
    bool is_initialized_;
    uint8_t tx_dummy_byte_;
    uint8_t rx_dummy_byte_;
    volatile uint8_t transaction_counter_;

    transactionCompleteCallback transaction_completed_callback_;
    void*                       transaction_completed_callback_param_;

    void setGpioAlternateFunction(GPIO_TypeDef* port, uint8_t pin, uint8_t alternate_function);
    void initClock();
    void initGpio();
    void initDma();
    void initSpi();
};
