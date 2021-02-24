#include "controller.h"
#include "../control_fun.h"

class PositionController : public Controller {
 public:
    PositionController(float dt) : Controller(dt), controller_(dt) {}
    void init(const MainLoopStatus &status) {
        controller_.init(status.fast_loop.motor_position.position);
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        float iq_des = controller_.step(command.position_desired, command.velocity_desired, status.fast_loop.motor_position.position) + \
                  command.current_desired;
        return iq_des;
    }
    void set_rollover(float rollover) { controller_.set_rollover(rollover); }
    void set_param(const PositionControllerParam &param) {
        controller_.set_param(param.position);
    }
 private:
    PIDController controller_;

    friend class System;
};
