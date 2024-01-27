#include "uart.h"

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

void USART2_IRQHandler(void)
{
  if(Uart::instance != NULL)
  {
    Uart::instance->errorInterruptHandler();
  }
}

}

Uart::Uart(const InitStruct& init_struct) :
    init_struct_(init_struct),
    is_initialized_(false),
    transaction_counter_(0),
    transaction_completed_callback_(NULL)
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
  // UART Rx Dma channel
  init_struct_.rxDmaChannel->CPAR = (uint32_t)&init_struct_.usart->RDR;
  init_struct_.rxDmaChannel->CNDTR = 0;
  init_struct_.rxDmaChannel->CCR = DMA_CCR_MINC | DMA_CCR_TCIE;

  init_struct_.rxDmaMuxChannel->CCR = init_struct_.rxDmaMuxId;

  // UART Tx Dma channel
  init_struct_.txDmaChannel->CPAR = (uint32_t)&init_struct_.usart->TDR;
  init_struct_.txDmaChannel->CNDTR = 0;
  init_struct_.txDmaChannel->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_TCIE;

  init_struct_.txDmaMuxChannel->CCR = init_struct_.txDmaMuxId;

  NVIC_SetPriority(
    init_struct_.rxDmaIrqN,
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), init_struct_.irqPriority, 0)
  );

  NVIC_SetPriority(
    init_struct_.txDmaIrqN,
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), init_struct_.irqPriority, 0)
  );

  NVIC_EnableIRQ(init_struct_.rxDmaIrqN);
  NVIC_EnableIRQ(init_struct_.txDmaIrqN);
}

void Uart::initUart()
{
  init_struct_.usart->BRR = init_struct_.brrValue;
  init_struct_.usart->CR3 = USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_OVRDIS | USART_CR3_DDRE;
  init_struct_.usart->CR1 = USART_CR1_PCE | USART_CR1_M0 | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

  NVIC_SetPriority(
    init_struct_.uartIrqN,
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), init_struct_.irqPriority, 0)
  );

  NVIC_EnableIRQ(init_struct_.uartIrqN);
}

void Uart::init()
{
  FIGURE_ASSERT(is_initialized_ == false, "Already initialized");

  DEBUG_PINS_INIT();

  initClock();
  initGpio();
  initDma();
  initUart();

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
    init_struct_.usart->CR1 |= USART_CR1_PEIE; // Parity error interrupt
    init_struct_.usart->CR3 |= USART_CR3_EIE;  // Error interrupt

    init_struct_.rxDmaChannel->CCR &= ~DMA_CCR_EN;
    init_struct_.rxDmaChannel->CNDTR = descriptor.length;
    init_struct_.rxDmaChannel->CMAR = (uint32_t)descriptor.rxBuffer;
    init_struct_.rxDmaChannel->CCR |= DMA_CCR_EN;
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
  init_struct_.rxDmaChannel->CCR &= ~DMA_CCR_EN;
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

void Uart::txInterruptHandler()
{
  // Clear the interrupt flag
  init_struct_.txDma->IFCR = init_struct_.txDmaIfcrCgif;

  if(transaction_completed_callback_ != NULL)
  {
    transaction_completed_callback_(transaction_completed_callback_param_);
  }
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
