#ifndef UNHUMAN_MOTORLIB_CONTROLLER_JOINT_POSITION_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_JOINT_POSITION_CONTROLLER_H_

#include "controller.h"
#include "../control_fun.h"
#include "velocity_controller.h"

class JointPositionController : public Controller {
 public:
    JointPositionController(float dt) : Controller(dt), velocity_controller_(dt) {}
    void init(const MainLoopStatus &status) {
        velocity_controller_.init(status);
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        float joint_error = command.position_desired - status.output_position;
        MotorCommand command_velocity = {};
        command_velocity.velocity_desired = param_.kpj*joint_error;
        float iq_des = velocity_controller_.step(command_velocity, status); 
        return iq_des;
    }
    void set_param(const JointPositionControllerParam &param) {
        velocity_controller_.set_param(param.velocity);
        param_ = param;
    }
    void set_rollover(float rollover) { velocity_controller_.set_rollover(rollover); }
    bool is_current_saturated() const {
        return velocity_controller_.is_current_saturated();
    }
 private:
    VelocityController velocity_controller_;
    JointPositionControllerParam param_ = {};

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_JOINT_POSITION_CONTROLLER_H_
