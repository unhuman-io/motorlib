#ifndef UNHUMAN_MOTORLIB_CONTROLLER_ADMITTANCE_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_ADMITTANCE_CONTROLLER_H_

#include "controller.h"
#include "velocity_controller.h"
#include "../control_fun.h"

class AdmittanceController : public Controller {
 public:
    AdmittanceController(float dt) : Controller(dt), velocity_controller_(dt), torque_controller_(dt) {}
    void init(const MainLoopStatus &status) {
        torque_controller_.init(status.torque);
        velocity_controller_.init(status);
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        float velocity_des = torque_controller_.step(command.torque_desired, 0, status.torque);
        MotorCommand velocity_command = {.current_desired = command.current_desired, .velocity_desired = velocity_des};
        float iq_des = velocity_controller_.step(velocity_command, status);
        return iq_des;
    }
    void set_param(const AdmittanceControllerParam &param) {
        torque_controller_.set_param(param.torque);
        velocity_controller_.set_param(param.velocity);
    }
 private:
    VelocityController velocity_controller_;
    PIDController torque_controller_;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_ADMITTANCE_CONTROLLER_H_
