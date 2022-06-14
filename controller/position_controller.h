#include "controller.h"
#include "../control_fun.h"

class PositionController : public Controller {
 public:
    PositionController(float dt) : Controller(dt), controller_(dt), desired_filter_(dt) {}
    void init(const MainLoopStatus &status) {
        controller_.init(status.motor_position);
        desired_filter_.init(status.motor_position);
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        float position_desired = desired_filter_.update(command.position_desired);
        float iq_des = controller_.step(position_desired, command.velocity_desired, status.motor_position, velocity_limit_) + \
                  command.current_desired;
        return iq_des;
    }
    void set_rollover(float rollover) { controller_.set_rollover(rollover); }
    void set_param(const PositionControllerParam &param) {
        controller_.set_param(param.position);
        velocity_limit_ = param.velocity_limit;
        desired_filter_.set_frequency(param.desired_filter_hz);
    }
 private:
    PIDController controller_;
    float velocity_limit_ = INFINITY;
    SecondOrderLowPassFilter desired_filter_;

    friend class System;
    friend void config_init();
};
