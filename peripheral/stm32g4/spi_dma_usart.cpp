#include "spi_dma_usart.h"

 __attribute((init_priority(SPIDMA_INIT_PRIORITY))) SPIPause SPIDMA_USART::spi_usart_pause[NUM_USARTS]{};
