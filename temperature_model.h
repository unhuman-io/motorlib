#pragma once

class TemperatureModel {
 public:
    TemperatureModel() {}
    float step(float current) {
        temperature = Ad_ * temperature + Bd_ * current * current * resistance_;
        return get_value();
    }
    float get_value() const { return temperature; }
 private:
    float temperature = 0;
    float Ad_, Bd_, resistance_;
};
