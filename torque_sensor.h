#ifndef UNHUMAN_MOTORLIB_TORQUE_SENSOR_H_
#define UNHUMAN_MOTORLIB_TORQUE_SENSOR_H_

#include "messages.h"
#include "sensor.h"

class TorqueSensorBase : public SensorBase {
 public:
    bool init() { return true; }
    void trigger() {}
    float read() { return 0; }
    void set_param(const TorqueSensorParam &param) {
        gain_ = param.gain;
        k_temp_ = param.k_temp;
    }
    // void zero(float current_torque_reading) {
    //     bias_ -= current_torque_reading;
    // }
    float get_value() const { return torque_; }
 //protected:
    float gain_ = 1;
    float k_temp_ = 0;
    float torque_ = -1;
 //   friend class System;
};

#endif  // UNHUMAN_MOTORLIB_TORQUE_SENSOR_H_
