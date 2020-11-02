#ifndef AMS_ENCODER
#define AMS_ENCODER

#include "../../encoder.h"
#include "../st_device.h"


class GPIO;

class AMSEncoder : public EncoderBase {
 public:
    AMSEncoder(SPI_TypeDef &regs, GPIO &gpio_cs) : EncoderBase(), regs_(regs), gpio_cs_(gpio_cs) {} 
    //void init() {}
    int32_t read()  __attribute__((section (".ccmram")));
    int32_t get_value()  const __attribute__((section (".ccmram")));
    void trigger()  __attribute__((section (".ccmram")));
    bool index_received() { return true; }
 protected:
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    uint16_t last_data_ = 0;
    int32_t count_ = 0;
};

#endif
