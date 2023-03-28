#ifndef UNHUMAN_MOTORLIB_CONTROLLER_TORQUE_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_TORQUE_CONTROLLER_H_

#include "../control_fun.h"
#include "controller.h"

class TorqueController : public Controller {
 public:
  TorqueController(float dt) : Controller(dt), controller_(dt) {}
  void init(const MainLoopStatus &status) { controller_.init(status.torque); }
  float step(const MotorCommand &command, const MainLoopStatus &status) {
    float iq_des = controller_.step(command.torque_desired, 0, status.torque) +
                   command.current_desired;
    return iq_des;
  }
  void set_param(const TorqueControllerParam &param) {
    controller_.set_param(param.torque);
  }

 private:
  PIDController controller_;

  friend class System;
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_TORQUE_CONTROLLER_H_
