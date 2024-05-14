#include "spi_dma.h"

Lock SPIDMA::spi_lock[NUM_SPIS]{};
