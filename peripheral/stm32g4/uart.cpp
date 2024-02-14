#include "uart.h"
#include "st_device.h"
#include <cstring>

Uart* Uart::instance = NULL;

// Interrupt handlers have to be inside extern "C" section
// so their names are not mangled
extern "C"
{

void DMA2_Channel3_IRQHandler(void)
{
  if(Uart::instance != NULL)
  {
    Uart::instance->rxInterruptHandler();
  }
}

void DMA2_Channel4_IRQHandler(void)
{
  if(Uart::instance != NULL)
  {
    Uart::instance->txInterruptHandler();
  }
}

void USART1_IRQHandler(void)
{
  if(Uart::instance != NULL)
  {
    Uart::instance->errorInterruptHandler();
  }
}

void TIM6_DAC_IRQHandler(void)
{
  if(Uart::instance != NULL)
  {
    Uart::instance->rxTimerInterruptHandler();
  }
}

}

Uart::Uart(const InitStruct& init_struct) :
    init_struct_(init_struct),
    is_initialized_(false),
    transaction_counter_(0),
    transaction_completed_callback_(NULL),
    rx_buffer_head_(0),
    rx_buffer_tail_(0)
{

}

Uart::~Uart()
{

}

void Uart::setGpioAlternateFunction(GPIO_TypeDef* port, uint8_t pin, uint8_t alternate_function)
{
  FIGURE_ASSERT(port != NULL, "Invalid arg");
  FIGURE_ASSERT(pin <= 15U, "Invalid arg");
  FIGURE_ASSERT(alternate_function <= 15U, "Invalid arg");
  __IO uint32_t* afr_register;

  // Put GPIO into "Alternate Function Mode"
  port->MODER &= ~(0x03 << (pin * 2)); // Each value is 2 bits hence 0x03 mask
  port->MODER |= (0x02 << (pin * 2));

  // Select the appropriate AFR register
  if(pin <= 7U)
  {
    afr_register = &port->AFR[0];
  }
  else
  {
    afr_register = &port->AFR[1];
    pin -= 8U;
  }

  // Apply the value to the register
  *afr_register &= ~(0x0F << (pin * 4));
  *afr_register |= (alternate_function << (pin * 4));
}

void Uart::initClock()
{
  // Enable GPIO clock
  *init_struct_.gpioRccEnableRegister |= (1 << init_struct_.gpioRccEnableBit);

  // Enable UART clock
  *init_struct_.uartRccEnableRegister |= (1 << init_struct_.uartRccEnableBit);

  // Enable DMA clock
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN;
}

void Uart::initGpio()
{
  // Configure GPIO alternate functions
  setGpioAlternateFunction(
    init_struct_.gpioPort,
    init_struct_.gpioPinTx,
    init_struct_.gpioAlternateFunction
  );

  setGpioAlternateFunction(
    init_struct_.gpioPort,
    init_struct_.gpioPinRx,
    init_struct_.gpioAlternateFunction
  );

  // Configure output speed to "Very high speed" for MISO
  init_struct_.gpioPort->OSPEEDR |= (0x03 << (init_struct_.gpioPinTx * 2));
}

void Uart::initDma()
{
  // Config UART Rx Dma channel for continuous receive into a circular buffer
  init_struct_.rxDmaChannel->CPAR = (uint32_t)&init_struct_.usart->RDR;
  init_struct_.rxDmaChannel->CCR = DMA_CCR_CIRC | DMA_CCR_MINC;
  init_struct_.rxDmaMuxChannel->CCR = init_struct_.rxDmaMuxId;
  init_struct_.rxDmaChannel->CNDTR = kRxBufferSize;
  init_struct_.rxDmaChannel->CMAR = (uint32_t)rx_buffer_;
  init_struct_.rxDmaChannel->CCR |= DMA_CCR_EN;

  // UART Tx Dma channel
  init_struct_.txDmaChannel->CPAR = (uint32_t)&init_struct_.usart->TDR;
  init_struct_.txDmaChannel->CNDTR = 0;
  init_struct_.txDmaChannel->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_TCIE;

  init_struct_.txDmaMuxChannel->CCR = init_struct_.txDmaMuxId;

//  NVIC_SetPriority(
//    init_struct_.rxDmaIrqN,
//    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), init_struct_.irqPriority, 0)
//  );

  NVIC_SetPriority(
    init_struct_.txDmaIrqN,
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), init_struct_.irqPriority, 0)
  );

//  NVIC_EnableIRQ(init_struct_.rxDmaIrqN);
  NVIC_EnableIRQ(init_struct_.txDmaIrqN);
}

void Uart::initUart()
{
  init_struct_.usart->BRR = init_struct_.brrValue;
  init_struct_.usart->CR3 = USART_CR3_ONEBIT | USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_OVRDIS;// | USART_CR3_DDRE;
  init_struct_.usart->CR1 = USART_CR1_PCE | USART_CR1_M0 | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

  NVIC_SetPriority(
    init_struct_.uartIrqN,
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), init_struct_.irqPriority, 0)
  );

  NVIC_EnableIRQ(init_struct_.uartIrqN);
}

void Uart::initRxTimer()
{
  // Freeze the timer in debug
  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM6_STOP;

  // Enable clocks
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

  TIM6->DIER = TIM_DIER_UIE;
  TIM6->ARR = CPU_FREQUENCY_HZ / 20000U;
  TIM6->CR1 = TIM_CR1_CEN;

  NVIC_SetPriority(
    TIM6_DAC_IRQn,
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), /*init_struct_.irqPriority*/ 0, 0)
  );

  NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void Uart::init()
{
  FIGURE_ASSERT(is_initialized_ == false, "Already initialized");

  DEBUG_PINS_INIT();

  initClock();
  initGpio();
  initDma();
  initUart();
  initRxTimer();

  instance = this;

  is_initialized_ = true;
}

void Uart::reset()
{
  abortTransaction();

  // Reset UART
  *init_struct_.uartRccResetRegister |= (1 << init_struct_.uartRccResetBit);
  *init_struct_.uartRccResetRegister &= ~(1 << init_struct_.uartRccResetBit);

  initUart();
}

void Uart::startTransaction(BufferDescriptor descriptor)
{
  FIGURE_ASSERT(!(descriptor.rxBuffer != NULL && descriptor.txBuffer != NULL)); // Tx and Rx at the same time is not supported by UART driver

  transaction_counter_++;

  // Configure rxBuffer
  if(descriptor.rxBuffer != NULL)
  {
//    init_struct_.usart->CR1 |= USART_CR1_PEIE; // Parity error interrupt
//    init_struct_.usart->CR3 |= USART_CR3_EIE;  // Error interrupt
//
//    init_struct_.rxDmaChannel->CCR &= ~DMA_CCR_EN;
//    init_struct_.rxDmaChannel->CNDTR = descriptor.length;
//    init_struct_.rxDmaChannel->CMAR = (uint32_t)descriptor.rxBuffer;
//    init_struct_.rxDmaChannel->CCR |= DMA_CCR_EN;
  }

  // Configure txBuffer
  if(descriptor.txBuffer != NULL)
  {
    init_struct_.txDmaChannel->CCR &= ~DMA_CCR_EN;
    init_struct_.txDmaChannel->CNDTR = descriptor.length;
    init_struct_.txDmaChannel->CMAR = (uint32_t)descriptor.txBuffer;
    init_struct_.txDmaChannel->CCR |= DMA_CCR_EN;
  }
}

void Uart::abortTransaction()
{
  // Disable DMA
  //init_struct_.rxDmaChannel->CCR &= ~DMA_CCR_EN;
  init_struct_.txDmaChannel->CCR &= ~DMA_CCR_EN;

  // Flush both Rx and Tx FIFO
  init_struct_.usart->RQR = USART_RQR_TXFRQ | USART_RQR_RXFRQ;
}

void Uart::resetTransactionCounter()
{
  transaction_counter_ = 0;
}

uint8_t Uart::getTransactionCounter()
{
  return transaction_counter_;
}

void Uart::setTransactionCompletedCallback(commsCallback callback, void* param)
{
  transaction_completed_callback_ = callback;
  transaction_completed_callback_param_ = param;
}

void Uart::setErrorCallback(commsCallback callback, void* param)
{
  error_callback_ = callback;
  error_callback_param_ = param;
}

void Uart::rxInterruptHandler()
{
  FIGURE_ASSERT(false, "Should not get here");

  // Clear the interrupt flag
  init_struct_.rxDma->IFCR = init_struct_.rxDmaIfcrCgif;

  // Disable error interrupts
  init_struct_.usart->CR1 &= ~USART_CR1_PEIE; // Parity error interrupt
  init_struct_.usart->CR3 &= ~USART_CR3_EIE;  // Error interrupt

  if(transaction_completed_callback_ != NULL)
  {
    transaction_completed_callback_(transaction_completed_callback_param_);
  }
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

size_t Uart::getRxBufferLength()
{
  size_t length;

  // If buffer is wrapped around
  if(rx_buffer_tail_ > rx_buffer_head_)
  {
    length = kRxBufferSize - rx_buffer_tail_ + rx_buffer_head_;
  }
  else
  {
    length = rx_buffer_head_ - rx_buffer_tail_;
  }

  return length;
}

uint8_t Uart::rxDmaReadByte()
{
  uint8_t result = rx_buffer_[rx_buffer_tail_];
  // Update the tail pointer accounting for the wrap-around
  rx_buffer_tail_ = (rx_buffer_tail_ + 1) % kRxBufferSize;
  return result;
}

void Uart::rxDmaReadBuffer(volatile uint8_t* buffer, size_t length)
{
  size_t chunk1_length = length;
  size_t chunk2_length = 0;

  // See if the data that's being read is wrapping around the rx_buffer_
  if(rx_buffer_tail_ + length > kRxBufferSize)
  {
    chunk1_length = kRxBufferSize - rx_buffer_tail_;
    chunk2_length = length - chunk1_length;
  }

  // Copy the beginning of the data
  std::memcpy(
    (void*)buffer,
    (void*)(rx_buffer_ + rx_buffer_tail_),
    chunk1_length
  );

  if(chunk2_length)
  {
    std::memcpy(
      (void*)(buffer + chunk1_length),
      (void*)rx_buffer_,
      chunk2_length
    );
  }

  // Update the tail pointer accounting for the wrap-around
  rx_buffer_tail_ = (rx_buffer_tail_ + length) % kRxBufferSize;
}

void Uart::rxTimerInterruptHandler()
{
  DEBUG_PIN1_SET();

  // Clear the interrupt flag
  TIM6->SR = 0;

  // Clear the Noise Error flag if present
  if(init_struct_.usart->ISR & USART_ISR_NE)
  {
    init_struct_.usart->ICR = USART_ICR_NECF;
  }

  // Update the head pointer
  rx_buffer_head_ = kRxBufferSize - init_struct_.rxDmaChannel->CNDTR;

  size_t data_length = getRxBufferLength();

  if(data_length > 0)
  {
    rxDmaReadBuffer(temp_buffer_, data_length);

    startTransaction({
      .txBuffer = temp_buffer_,
      .rxBuffer = NULL,
      .length = data_length
    });
  }

  DEBUG_PIN1_CLEAR();
}

#pragma GCC pop_options

void Uart::txInterruptHandler()
{
  // Clear the interrupt flag
  init_struct_.txDma->IFCR = init_struct_.txDmaIfcrCgif;
//
//  if(transaction_completed_callback_ != NULL)
//  {
//    transaction_completed_callback_(transaction_completed_callback_param_);
//  }
}

void Uart::errorInterruptHandler()
{
  // Reset the UART
  reset();

  if(error_callback_ != NULL)
  {
    error_callback_(error_callback_param_);
  }
}
