#include "spi_slave_figure.h"

SpiSlaveFigure* SpiSlaveFigure::instance = NULL;

// Interrupt handlers have to be inside extern "C" section
// so their names are not mangled
extern "C"
{

void DMA2_Channel1_IRQHandler(void)
{
  if(SpiSlaveFigure::instance != NULL)
  {
    SpiSlaveFigure::instance->rxInterruptHandler();
  }
}

void DMA2_Channel2_IRQHandler(void)
{
  if(SpiSlaveFigure::instance != NULL)
  {
    SpiSlaveFigure::instance->txInterruptHandler();
  }
}

} // extern C

SpiSlaveFigure::SpiSlaveFigure(const InitStruct& init_struct) :
    init_struct_(init_struct),
    is_initialized_(false),
    transaction_counter_(0),
    transaction_completed_callback_(NULL)
{

}

SpiSlaveFigure::~SpiSlaveFigure()
{

}

void SpiSlaveFigure::setGpioAlternateFunction(GPIO_TypeDef* port, uint8_t pin, uint8_t alternate_function)
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

void SpiSlaveFigure::initClock()
{
  // Enable GPIO clock
  *init_struct_.gpioRccEnableRegister |= (1 << init_struct_.gpioRccEnableBit);

  // Enable SPI clock
  *init_struct_.spiRccEnableRegister |= (1 << init_struct_.spiRccEnableBit);

  // Enable DMA clock
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN;
}

void SpiSlaveFigure::initGpio()
{
  // Configure GPIO alternate functions
  setGpioAlternateFunction(
    init_struct_.gpioPort,
    init_struct_.gpioPinSs,
    init_struct_.gpioAlternateFunction
  );

  setGpioAlternateFunction(
    init_struct_.gpioPort,
    init_struct_.gpioPinSck,
    init_struct_.gpioAlternateFunction
  );

  setGpioAlternateFunction(
    init_struct_.gpioPort,
    init_struct_.gpioPinMosi,
    init_struct_.gpioAlternateFunction
  );

  setGpioAlternateFunction(
    init_struct_.gpioPort,
    init_struct_.gpioPinMiso,
    init_struct_.gpioAlternateFunction
  );

  // Configure output speed to "Very high speed" for MISO
  init_struct_.gpioPort->OSPEEDR |= (0x03 << (init_struct_.gpioPinMiso * 2));
}

void SpiSlaveFigure::initDma()
{
  // SPI Rx Dma channel
  init_struct_.rxDmaChannel->CPAR = (uint32_t)&init_struct_.spi->DR;
  init_struct_.rxDmaChannel->CNDTR = RX_BUFFER_SIZE;
  init_struct_.rxDmaChannel->CCR = DMA_CCR_MINC | DMA_CCR_CIRC;
  init_struct_.rxDmaChannel->CMAR = (uint32_t)&rx_buffer_;
  init_struct_.rxDmaMuxChannel->CCR = init_struct_.rxDmaMuxId;
  init_struct_.rxDmaChannel->CCR |= DMA_CCR_EN;

  // SPI Tx Dma channel
  init_struct_.txDmaChannel->CPAR = (uint32_t)&init_struct_.spi->DR;
  init_struct_.txDmaChannel->CNDTR = 0;
  init_struct_.txDmaChannel->CCR = DMA_CCR_DIR | DMA_CCR_MINC;// | DMA_CCR_TCIE;

  init_struct_.txDmaMuxChannel->CCR = init_struct_.txDmaMuxId;

  // NVIC_SetPriority(
  //   init_struct_.rxDmaIrqN,
  //   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), init_struct_.rxDmaIrqPriority, 0)
  // );

  // NVIC_EnableIRQ(init_struct_.rxDmaIrqN);

  //  NVIC_SetPriority(init_struct_.txDmaIrqN, init_struct_.txDmaIrqPriority);
  //  NVIC_EnableIRQ(init_struct_.txDmaIrqN);
}

void SpiSlaveFigure::initSpi()
{
  // Set data length to 8 bit
  init_struct_.spi->CR2 =
    SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN |
    SPI_CR2_FRXTH |
    SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;

  // Enable SPI
  init_struct_.spi->CR1 = SPI_CR1_SPE;
}

void SpiSlaveFigure::init()
{
  FIGURE_ASSERT(is_initialized_ == false, "Already initialized");

  DEBUG_PINS_INIT();

  initClock();
  initGpio();
  initDma();
  initSpi();

  instance = this;

  is_initialized_ = true;
}

void SpiSlaveFigure::reset()
{
  // Reset SPI
  *init_struct_.spiRccResetRegister |= (1 << init_struct_.spiRccResetBit);
  *init_struct_.spiRccResetRegister &= ~(1 << init_struct_.spiRccResetBit);
  initSpi();
}

void SpiSlaveFigure::startTransaction(BufferDescriptor descriptor)
{

  transaction_counter_++;

  // // Configure rxBuffer
  // if(descriptor.rxBuffer != NULL)
  // {
  //   init_struct_.usart->CR1 |= USART_CR1_PEIE; // Parity error interrupt
  //   init_struct_.usart->CR3 |= USART_CR3_EIE;  // Error interrupt

  //   init_struct_.rxDmaChannel->CCR &= ~DMA_CCR_EN;
  //   init_struct_.rxDmaChannel->CNDTR = descriptor.length;
  //   init_struct_.rxDmaChannel->CMAR = (uint32_t)descriptor.rxBuffer;
  //   init_struct_.rxDmaChannel->CCR |= DMA_CCR_EN;
  // }

  // Configure txBuffer
  if(descriptor.txBuffer != NULL)
  {
    init_struct_.txDmaChannel->CCR &= ~DMA_CCR_EN;
    init_struct_.txDmaChannel->CNDTR = descriptor.length;
    init_struct_.txDmaChannel->CMAR = (uint32_t)descriptor.txBuffer;
    init_struct_.txDmaChannel->CCR |= DMA_CCR_EN;
  }
}

void SpiSlaveFigure::abortTransaction()
{

}

void SpiSlaveFigure::resetTransactionCounter()
{
  transaction_counter_ = 0;
}

uint8_t SpiSlaveFigure::getTransactionCounter()
{
  return transaction_counter_;
}

void SpiSlaveFigure::setTransactionCompletedCallback(commsCallback callback, void* param)
{
  transaction_completed_callback_ = callback;
  transaction_completed_callback_param_ = param;
}

void SpiSlaveFigure::setErrorCallback(commsCallback callback, void* param)
{
  error_callback_ = callback;
  error_callback_param_ = param;
}

void SpiSlaveFigure::rxInterruptHandler()
{
  // Clear the interrupt flag
  init_struct_.rxDma->IFCR = init_struct_.rxDmaIfcrCgif;

  if(transaction_completed_callback_ != NULL)
  {
    transaction_completed_callback_(transaction_completed_callback_param_);
  }
}

void SpiSlaveFigure::txInterruptHandler()
{
  // Disable the interrupt
  init_struct_.txDmaChannel->CCR &= ~DMA_CCR_TCIE;

  // Clear the interrupt flag
  init_struct_.txDma->IFCR = init_struct_.txDmaIfcrCgif;

  if(transaction_completed_callback_ != NULL)
  {
    transaction_completed_callback_(transaction_completed_callback_param_);
  }
}

void SpiSlaveFigure::errorInterruptHandler()
{
  // Reset the UART
  reset();

  if(error_callback_ != NULL)
  {
    error_callback_(error_callback_param_);
  }
}

uint16_t SpiSlaveFigure::get_current_rx_index() const {
  return RX_BUFFER_SIZE - init_struct_.rxDmaChannel->CNDTR;
}

uint16_t SpiSlaveFigure::get_last_rx_index() const {
  return (2*RX_BUFFER_SIZE - init_struct_.rxDmaChannel->CNDTR - 1) % RX_BUFFER_SIZE;
}

bool SpiSlaveFigure::is_tx_active() const {
  return init_struct_.txDmaChannel->CNDTR == 0;
}

void SpiSlaveFigure::rx_copy(uint8_t * const out_buf, uint8_t * rx_buf_ptr, uint16_t length) {
  // todo optimize with 1 to 2 memcpy for circular buffer
  int index = (rx_buf_ptr - rx_buffer_) % RX_BUFFER_SIZE;
  for(int i=0; i<length; i++) {
    out_buf[i] = rx_buffer_[index++];
    index %= RX_BUFFER_SIZE;
  }
}
