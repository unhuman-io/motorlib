#include "spi_slave.h"

SpiSlave* SpiSlave::instance = NULL;

// Interrupt handlers have to be inside extern "C" section
// so their names are not mangled
extern "C"
{

void DMA2_Channel1_IRQHandler(void)
{
  SPI_DEBUG_PIN1_SET();

  if(SpiSlave::instance != NULL)
  {
    SpiSlave::instance->rxInterruptHandler();
  }

  SPI_DEBUG_PIN1_CLEAR();
}

void DMA2_Channel2_IRQHandler(void)
{
  if(SpiSlave::instance != NULL)
  {
    SpiSlave::instance->txInterruptHandler();
  }
}

}

SpiSlave::SpiSlave(const InitStruct& init_struct) :
  init_struct_(init_struct),
  is_initialized_(false),
  tx_dummy_byte_(0x00),
  rx_dummy_byte_(0x00),
  transaction_counter_(0),
  transaction_completed_callback_(NULL)
{
}

SpiSlave::~SpiSlave()
{
}

void SpiSlave::setGpioAlternateFunction(GPIO_TypeDef* port, uint8_t pin, uint8_t alternate_function)
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

void SpiSlave::initClock()
{
  // Enable GPIO clock
  *init_struct_.gpioRccEnableRegister |= (1 << init_struct_.gpioRccEnableBit);

  // Enable SPI clock
  *init_struct_.spiRccEnableRegister |= (1 << init_struct_.spiRccEnableBit);

  // Enable DMA clock
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN;
}

void SpiSlave::initGpio()
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

void SpiSlave::initDma()
{
  // SPI Rx Dma channel
  init_struct_.rxDmaChannel->CPAR = (uint32_t)&init_struct_.spi->DR;
  init_struct_.rxDmaChannel->CNDTR = 0;
  init_struct_.rxDmaChannel->CCR = DMA_CCR_MINC | DMA_CCR_TCIE;

  init_struct_.rxDmaMuxChannel->CCR = init_struct_.rxDmaMuxId;

  // SPI Tx Dma channel
  init_struct_.txDmaChannel->CPAR = (uint32_t)&init_struct_.spi->DR;
  init_struct_.txDmaChannel->CNDTR = 0;
  init_struct_.txDmaChannel->CCR = DMA_CCR_DIR | DMA_CCR_MINC;// | DMA_CCR_TCIE;

  init_struct_.txDmaMuxChannel->CCR = init_struct_.txDmaMuxId;

  NVIC_SetPriority(
    init_struct_.rxDmaIrqN,
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), init_struct_.rxDmaIrqPriority, 0)
  );

  NVIC_EnableIRQ(init_struct_.rxDmaIrqN);

//  NVIC_SetPriority(init_struct_.txDmaIrqN, init_struct_.txDmaIrqPriority);
//  NVIC_EnableIRQ(init_struct_.txDmaIrqN);
}

void SpiSlave::initSpi()
{
  // Set data length to 8 bit
  init_struct_.spi->CR2 =
    SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN |
    SPI_CR2_FRXTH |
    SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;

  // Enable SPI
  init_struct_.spi->CR1 = SPI_CR1_SPE;
}

void SpiSlave::reset()
{
  // Reset SPI
  *init_struct_.spiRccResetRegister |= (1 << init_struct_.spiRccResetBit);
  *init_struct_.spiRccResetRegister &= ~(1 << init_struct_.spiRccResetBit);
  initSpi();
}

void SpiSlave::startTransaction(BufferDescriptor descriptor)
{
  volatile uint32_t temp;
  SPI_DEBUG_PIN2_SET();

  transaction_counter_++;

  // Disable the DMA
  init_struct_.rxDmaChannel->CCR &= ~DMA_CCR_EN;
  init_struct_.txDmaChannel->CCR &= ~DMA_CCR_EN;

  // Clear OVR flag if present (read DR and then SR)
  if(init_struct_.spi->SR & SPI_SR_OVR)
  {
    temp = init_struct_.spi->DR;
    temp = init_struct_.spi->SR;
    (void)temp;
  }

  // Update buffer pointer and data length
  init_struct_.rxDmaChannel->CNDTR = descriptor.length;
  init_struct_.txDmaChannel->CNDTR = descriptor.length;

  // Configure rxBuffer
  if(descriptor.rxBuffer != NULL)
  {
    init_struct_.rxDmaChannel->CMAR = (uint32_t)descriptor.rxBuffer;
    init_struct_.rxDmaChannel->CCR |= DMA_CCR_MINC;
  }
  else
  {
    init_struct_.rxDmaChannel->CMAR = (uint32_t)&rx_dummy_byte_;
    init_struct_.rxDmaChannel->CCR &= ~DMA_CCR_MINC;
  }

  // Configure txBuffer
  if(descriptor.txBuffer != NULL)
  {
    init_struct_.txDmaChannel->CMAR = (uint32_t)descriptor.txBuffer;
    init_struct_.txDmaChannel->CCR |= DMA_CCR_MINC;
  }
  else
  {
    init_struct_.txDmaChannel->CMAR = (uint32_t)&tx_dummy_byte_;
    init_struct_.txDmaChannel->CCR &= ~DMA_CCR_MINC;
  }

  // Re-enable the DMA
  init_struct_.rxDmaChannel->CCR |= DMA_CCR_EN;
  init_struct_.txDmaChannel->CCR |= DMA_CCR_EN;

  SPI_DEBUG_PIN2_CLEAR();
}

void SpiSlave::rxInterruptHandler()
{
  // Disable the interrupt
//  init_struct_.rxDmaChannel->CCR &= ~DMA_CCR_TCIE;

  // Clear the interrupt flag
  init_struct_.rxDma->IFCR = init_struct_.rxDmaIfcrCgif;

  if(transaction_completed_callback_ != NULL)
  {
    transaction_completed_callback_(transaction_completed_callback_param_);
  }
}

void SpiSlave::txInterruptHandler()
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

void SpiSlave::setTransactionCompletedCallback(transactionCompleteCallback callback, void* param)
{
  transaction_completed_callback_ = callback;
  transaction_completed_callback_param_ = param;
}

void SpiSlave::resetTransactionCounter()
{
  transaction_counter_ = 0;
}

uint8_t SpiSlave::getTransactionCounter()
{
  return transaction_counter_;
}

void SpiSlave::init()
{
  FIGURE_ASSERT(is_initialized_ == false, "Already initialized");

  SPI_DEBUG_PINS_INIT();

  initClock();
  initGpio();
  initDma();
  initSpi();

  instance = this;

  is_initialized_ = true;
}


