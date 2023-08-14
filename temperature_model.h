#pragma once
#include <cmath>

// Tdot = 1/Cp*(i^2*r - T/Rth)
// Ad = exp(-1/(Rth*Cp)*dt)
// Bd = -Rth*(Ad - 1)
class TemperatureModel {
 public:
    TemperatureModel() {}
    float step(float id, float iq) {
        temperature_ = Ad_ * temperature_ + Bd_ * (id * id + iq * iq);
        return get_value();
    }
    float get_value() const { return temperature_  + temperature_ambient_; }
    void set_ambient_temperature(float temp) { temperature_ambient_ = temp; }
    void set_param(float resistance, float Cp, float Rth, float dt) {
        resistance_ = resistance*.75; // line to line to phase
        Ad_ = std::exp(-1/(Rth*Cp)*dt);
        Bd_ = -Rth*(Ad_ - 1);
    }
 private:
    float temperature_ = 0;
    float temperature_ambient_ = 0;
    float Ad_, Bd_, resistance_;
};
