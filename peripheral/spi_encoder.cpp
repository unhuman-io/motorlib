#include "spi_encoder.h"
#include "../gpio.h"
#include "../util.h"

void SPIEncoder::trigger() {
    // clear chip select
    gpio_cs_.clear();

    // some devices need a time delay between chip select and clock
    // 80 ns ma732, 300ns AEAT-8800
    ns_delay(start_cs_delay_ns_);

    // SPI is full duplex, to read you first write to the data register, then wait, then read the data register
    regs_.DR = 0;
}

int32_t SPIEncoder::read() {
    // wait until receive not empty flag SPI_FLAG_RXNE is set, ma732 max frequency 25 Mbps, 640 ns (115 cycles at 180 MHz)
    while(!(regs_.SR & SPI_SR_RXNE)); // RXNE: 1 -> data available
    data_ = regs_.DR;
    
    // some devices need a time delay between chip select and clock, 25 ns ma732, 200 ns AEAT-8800
    ns_delay(end_cs_delay_ns_);
    gpio_cs_.set();
    return data_;
}

uint16_t SPIEncoder::send_and_read(uint16_t value) {
    // same as above trigger and read
    gpio_cs_.clear();
    ns_delay(start_cs_delay_ns_);
    regs_.DR = value;
    while(!(regs_.SR & SPI_SR_RXNE)); // RXNE: 1 -> data available
    ns_delay(end_cs_delay_ns_);
    gpio_cs_.set();
    return regs_.DR;
}