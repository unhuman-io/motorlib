#pragma once

#include "torque_sensor.h"

class MotorTorqueSensor final : public TorqueSensorBase {
 public:
    float read() { return 0; } // todo return kt * measured motor current?
};