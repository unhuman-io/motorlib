#include "spi_dma.h"

 __attribute((init_priority(101))) SPIPause SPIDMA::spi_pause[NUM_SPIS]{};
