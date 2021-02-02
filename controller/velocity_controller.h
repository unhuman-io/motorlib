#include "controller.h"
#include "../control_fun.h"

class VelocityController : public Controller {
 public:
    VelocityController(float dt) : Controller(dt), controller_(dt) {}
    void init(const MainLoopStatus &status) {
        controller_.init(status.fast_loop.motor_position.position);
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        float position_desired = status.fast_loop.motor_position.position + 
            position_change_max_*fsignf(command.velocity_desired);
        float iq_des = controller_.step(position_desired, 
                  command.velocity_desired, status.fast_loop.motor_position.position, command.velocity_desired) +
                  command.current_desired;
        return iq_des;
    }
    void set_param(const VelocityControllerParam &param) {
        controller_.set_param(param.position);
        position_change_max_ = param.position.command_max/param.position.kp;
    }
    void set_rollover(float rollover) { controller_.set_rollover(rollover); }
// private:
    float position_change_max_ = 0;
    PIDController controller_;

    friend class System;
};
