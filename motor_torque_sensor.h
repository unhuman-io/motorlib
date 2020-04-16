#pragma once

#include "torque_sensor.h"

class MotorTorqueSensor final : public TorqueSensor {
 public:
    virtual float read() { return 0; } // todo return kt * measured motor current?
};