#pragma once
#include "../messages.h"
typedef ReceiveData MotorCommand;

class Controller {
 public:
    Controller(float dt) : dt_(dt) {}
    float step(const MotorCommand &command, const MainLoopStatus &status) { return 0; }
 private:
    float dt_;
};