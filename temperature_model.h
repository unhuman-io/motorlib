#pragma once
#include <cmath>

// Tdot = 1/Cp*(i^2*r - T/Rth)
// Ad = exp(-1/(Rth*Cp)*dt)
// Bd = -Rth*(Ad - 1)
class TemperatureModel {
 public:
    TemperatureModel() {}
    float step(float id, float iq) {
        temperature_ = Ad_ * temperature_ + (1 + alpha_cu_*(get_value()-25))*Bd_ * (id * id + iq * iq);
        return get_value();
    }
    float get_value() const { return temperature_  + temperature_ambient_; }
    void set_ambient_temperature(float temp) { temperature_ambient_ = temp; }
    void set_param(float resistance, float Cp, float Rth, float dt) {
        Ad_ = std::exp(-1/(Rth*Cp)*dt);
        Bd_ = -.75*resistance*Rth*(Ad_ - 1);   //.75 is for line to line to phase
    }
 private:
    float temperature_ = 0;
    volatile float temperature_ambient_ = 0;
    const float alpha_cu_ = 0.00393;
    float Ad_, Bd_;
};
