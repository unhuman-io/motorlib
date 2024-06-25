#pragma once

#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx.h"

#include "../macro.h"
#include "../comms.h"

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
    void abortTransaction() final;
    void setTransactionCompletedCallback(commsCallback callback, void* param) final;
    void setErrorCallback(commsCallback callback, void* param) final;

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

    commsCallback transaction_completed_callback_;
    void*         transaction_completed_callback_param_;

    void setGpioAlternateFunction(GPIO_TypeDef* port, uint8_t pin, uint8_t alternate_function);
    void initClock();
    void initGpio();
    void initDma();
    void initSpi();
};
