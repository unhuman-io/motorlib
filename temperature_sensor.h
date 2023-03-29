#ifndef UNHUMAN_MOTORLIB_TEMPERATURE_SENSOR_H_
#define UNHUMAN_MOTORLIB_TEMPERATURE_SENSOR_H_

class TemperatureSensor {
 public:
  float read() { return get_temperature(); }
  float get_temperature() const { return 0; }
};

class PT1000 : public TemperatureSensor {
 public:
  PT1000(volatile uint32_t& adc) : adc_(adc) {}
  float read() {
    float voltage = 1.0 / 4096 * adc_;
    float resistance = 1000 * (3.3 / voltage - 1);
    temperature_ = (resistance - 1000) / (resistance * 3.91e-3);
    return get_temperature();
  }
  float get_temperature() const { return temperature_; }

 private:
  volatile uint32_t& adc_;
  float temperature_ = 0;
};

#endif  // UNHUMAN_MOTORLIB_TEMPERATURE_SENSOR_H_
