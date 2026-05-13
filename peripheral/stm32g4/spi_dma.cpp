#include "spi_dma.h"

 __attribute((init_priority(SPIDMA_INIT_PRIORITY))) SPIPause spi_pause[SPIConfig::NUM_SPIS]{};
