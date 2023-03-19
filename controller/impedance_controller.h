#ifndef UNHUMAN_MOTORLIB_CONTROLLER_IMPEDANCE_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_IMPEDANCE_CONTROLLER_H_

#include "controller.h"
#include "../control_fun.h"

class ImpedanceController : public Controller {
 public:
    ImpedanceController(float dt) : Controller(dt), impedance_controller_(dt), torque_controller_(dt) {}
    void init(const MainLoopStatus &status) {
        impedance_controller_.init(status.motor_position);
        torque_controller_.init(status.torque);
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        float torque_des = impedance_controller_.step(command.position_desired, command.velocity_desired, 0, status.motor_position) + \
                  command.torque_desired;
        float iq_des = torque_controller_.step(torque_des, 0, status.torque) + \
                  command.current_desired;
        return iq_des;
    }
    void set_param(const ImpedanceControllerParam &param) {
        impedance_controller_.set_param(param.impedance);
        torque_controller_.set_param(param.torque);
    }
    void set_rollover(float rollover) { impedance_controller_.set_rollover(rollover); }
 private:
    PIDController impedance_controller_;
    PIDController torque_controller_;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_IMPEDANCE_CONTROLLER_H_
