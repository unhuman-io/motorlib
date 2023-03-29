#ifndef UNHUMAN_MOTORLIB_CONTROLLER_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_CONTROLLER_H_

#include "../messages.h"
typedef ReceiveData MotorCommand;

class Controller {
 public:
  Controller(float dt) : dt_(dt) {}
  float step(const MotorCommand &command, const MainLoopStatus &status) {
    return 0;
  }

 protected:
  float dt_;
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_CONTROLLER_H_
