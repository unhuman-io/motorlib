#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_SPI_ENCODER_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_SPI_ENCODER_H_

#include "../encoder.h"
#include "st_device.h"
#include "../gpio.h"
#include "../util.h"

template<typename SPI>
class SPIEncoder : public EncoderBase {
 public:
    SPIEncoder(SPI &spi, GPIO &gpio_cs) : EncoderBase(), spi_(spi), gpio_cs_(gpio_cs) {} 
    //void init() {}

    void trigger() {
        // clear chip select
        gpio_cs_.clear();

        // some devices need a time delay between chip select and clock
        // 80 ns ma732, 300ns AEAT-8800
        ns_delay(start_cs_delay_ns_);

        // SPI is full duplex, to read you first write to the data register, then wait, then read the data register
        spi_.write((uint16_t) 0);
        
    }

    int32_t read() {
        // wait until receive not empty flag SPI_FLAG_RXNE is set, ma732 max frequency 25 Mbps, 640 ns (115 cycles at 180 MHz)
        while(!spi_.data_available());
        data_ = spi_.read_u16();
        
        // some devices need a time delay between chip select and clock, 25 ns ma732, 200 ns AEAT-8800
        ns_delay(end_cs_delay_ns_);
        gpio_cs_.set();
        return data_;
    }

    int32_t get_value()  const { 
        return data_;
    }

    uint16_t send_and_read(uint16_t value) {
        // same as above trigger and read
        gpio_cs_.clear();
        ns_delay(start_cs_delay_ns_);
        spi_.write(value);
        while(!spi_.data_available());
        ns_delay(end_cs_delay_ns_);
        gpio_cs_.set();
        return spi_.read_u16();
    }

    bool index_received() { return true; }
    
    SPI &spi_;
    GPIO &gpio_cs_;
    uint16_t data_;
    uint16_t start_cs_delay_ns_ = 80;
    uint16_t end_cs_delay_ns_ = 25;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_SPI_ENCODER_H_
