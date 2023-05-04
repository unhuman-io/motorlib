#ifndef UNHUMAN_MOTORLIB_TEMPERATURE_SENSOR_H_
#define UNHUMAN_MOTORLIB_TEMPERATURE_SENSOR_H_

#include <cmath>

class TemperatureSensor {
 public:
    float read() { return get_temperature(); }
    float get_temperature() const { return 0; }

};

class PT1000 : public TemperatureSensor {
 public:
    PT1000(volatile uint32_t& adc, float v3v3 = 3.3) : adc_(adc), v3v3_(v3v3) {}
    float read() {
        float voltage = 1.0/4096*adc_;
        // top side 1k/1k
        float resistance = 1000*(v3v3_/voltage-1);
        temperature_ = (resistance-1000)/(1000*3.91e-3);
        return get_temperature(); 
    }
    float get_temperature() const { return temperature_; }   
 private:
    volatile uint32_t& adc_;
    float v3v3_;
    float temperature_ = 0;
};

class NTC : public TemperatureSensor {
 public:
    NTC(volatile uint32_t& adc) : adc_(adc) {}
    float read() {
        float voltage = 3.3/4096*adc_;
        // bottom side 10k/10k
        // v = 3.3*r/(r10 + r)
        // v/3.3/r = 1/(r10+r)
        // 3.3*r/v = r10 + r
        // r(3.3/v - 1) = r10
        // r = r10/(3.3/v - 1)
        float resistance = 10000/(3.3/voltage-1);
        float B = 3435; // K
        temperature_ = 1/(std::log(resistance/10000)/B + 1/298.15) - 273.15;
        return get_temperature(); 
    }
    float get_temperature() const { return temperature_; }
 private:
    volatile uint32_t& adc_;
    float temperature_ = 0;
};

#endif  // UNHUMAN_MOTORLIB_TEMPERATURE_SENSOR_H_
