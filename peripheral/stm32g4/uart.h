#pragma once

#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx.h"
#include "../macro.h"
#include "../comms.h"

class Uart : public Comms
{
  public:
    static Uart* instance;

    struct InitStruct
    {
      USART_TypeDef* usart;
      GPIO_TypeDef*  gpioPort;
      uint8_t        gpioPinTx;
      uint8_t        gpioPinRx;
      uint8_t        gpioAlternateFunction;
      __IO uint32_t* gpioRccEnableRegister;
      uint8_t        gpioRccEnableBit;
      __IO uint32_t* uartRccEnableRegister;
      uint8_t        uartRccEnableBit;
      __IO uint32_t* uartRccResetRegister;
      uint8_t        uartRccResetBit;

      IRQn_Type      uartIrqN;

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

      uint8_t                 irqPriority;

      uint32_t                brrValue;
    };

    Uart(const InitStruct& init_struct);
    ~Uart();

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
    void rxTimerInterruptHandler();
    void txInterruptHandler();
    void errorInterruptHandler();

  private:
    static constexpr size_t kRxBufferSize = 64U;

    const InitStruct init_struct_;
    bool is_initialized_;

    volatile uint8_t transaction_counter_;

    commsCallback transaction_completed_callback_;
    void*         transaction_completed_callback_param_;

    commsCallback error_callback_;
    void*         error_callback_param_;

    volatile uint8_t rx_buffer_[kRxBufferSize];
    volatile uint8_t temp_buffer_[kRxBufferSize];
    volatile size_t rx_buffer_head_;
    volatile size_t rx_buffer_tail_;

    void setGpioAlternateFunction(GPIO_TypeDef* port, uint8_t pin, uint8_t alternate_function);
    void initClock();
    void initGpio();
    void initDma();
    void initUart();
    void initRxTimer();

    size_t  getRxBufferLength();
    uint8_t rxDmaReadByte();
    void    rxDmaReadBuffer(volatile uint8_t* buffer, size_t length);
};
