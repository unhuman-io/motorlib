#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_TEMP_SENSOR_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_TEMP_SENSOR_H_

extern "C" {
    void system_init();
}

#include <cstdint>
#include "stm32g4xx.h"
#include "../../util.h"

#ifndef V_TEMP_DR
volatile uint32_t default_v_temp_dr = 0;
#define V_TEMP_DR default_v_temp_dr
#endif

#include "pin_config.h"
// Internal STM32G4 temperature sensor, requires v_temp_dr_ to be set up as an ADC input, using GCOMP
class TempSensor {
 public:
    TempSensor(volatile uint32_t &v_temp_dr=V_TEMP_DR) : v_temp_dr_(v_temp_dr) {
        // set up ADC
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
        ADC1->CR &= ~ADC_CR_DEEPPWD;
        ADC1->CR |= ADC_CR_ADVREGEN;
        ns_delay(10);
        ADC1->CR |= ADC_CR_ADCALDIF;
        ns_delay(10);
        ADC1->CR |= ADC_CR_ADCAL;
        while (ADC1->CR & ADC_CR_ADCAL);
        ns_delay(10);

    }
    // temperature in C
    void set_value(float value) {
        bias_ = value - read();
    }
    float read() {        
        value_ = (130.0-30.0)/(*TS_CAL2 - *TS_CAL1) * ((int16_t) v_temp_dr_ / 3.0 - *TS_CAL1) + 30 + bias_;

        return value_;
    }
    float get_value() const { return value_; }
 private:
    const uint16_t * const TS_CAL1 = (const uint16_t * const) 0x1fff75a8;
    const uint16_t * const TS_CAL2 = (const uint16_t * const) 0x1fff75ca;
    float value_ = 0;
    float bias_ = 0;
    volatile uint32_t &v_temp_dr_;

    //friend void system_init();
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_TEMP_SENSOR_H_
