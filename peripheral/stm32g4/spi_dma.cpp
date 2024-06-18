#include "spi_dma.h"

 __attribute((init_priority(SPIDMA_INIT_PRIORITY))) SPIPause SPIDMA::spi_pause[NUM_SPIS]{};
