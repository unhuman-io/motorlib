#pragma once

extern "C" {
    void system_init();
}

#include "pin_config.h"
class TempSensor {
 public:
    TempSensor() {

    }
    // temperature in C
    void set_value(float value) {
        bias_ = value - read();
    }
    float read() {        
        ADC1->CR |= ADC_CR_JADSTART;
        while(ADC1->CR & ADC_CR_JADSTART);
        value_ = (110.0-30.0)/(*TS_CAL2 - *TS_CAL1) * ((int16_t) ADC1->JDR1 / 3.3 - *TS_CAL1) + 30 + bias_;

        return value_;
    }
    float get_value() const { return value_; }
 private:
    const uint16_t * const TS_CAL1 = (const uint16_t * const) 0x1fff75a8;
    const uint16_t * const TS_CAL2 = (const uint16_t * const) 0x1fff75ca;
    float value_ = 0;
    float bias_ = 0;

    friend void system_init();
};
