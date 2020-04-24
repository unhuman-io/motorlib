#pragma once

#include "../encoder.h"
#include "../st_device.h"


class GPIO;

class SPIEncoder : public Encoder {
 public:
    SPIEncoder(SPI_TypeDef &regs, GPIO &gpio_cs) : Encoder(), regs_(regs), gpio_cs_(gpio_cs) {} 
    //void init() {}
    virtual int32_t read()  __attribute__((section (".ccmram")));
    virtual int32_t get_value()  const __attribute__((section (".ccmram")));
    virtual void trigger()  __attribute__((section (".ccmram")));
    virtual bool index_received() { return true; }
    
    // for configuration values, make sure to only call when trigger and read
    // are not running
    uint16_t send_and_read(uint16_t value);
 protected:
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    uint16_t data_;
    uint16_t start_cs_delay_ns_ = 80;
    uint16_t end_cs_delay_ns_ = 25;
};