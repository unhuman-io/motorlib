#pragma once

#include "controller.h"
#include "../control_fun.h"

class VelocityController : public Controller {
 public:
    VelocityController(float dt) : Controller(dt), controller_(dt), velocity_filter_(dt) {}
    void init(const MainLoopStatus &status) {
        controller_.init(0);    // can only switch into this mode at zero velocity without glitch
        last_motor_position_ = status.motor_position;
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        float velocity_measured = wrap1_diff(status.motor_position, last_motor_position_, rollover_)/dt_;
        last_motor_position_ = status.motor_position;
        velocity_measured_filt_ = velocity_filter_.update(velocity_measured);
        float iq_des = controller_.step(command.velocity_desired, 0, velocity_measured_filt_, acceleration_limit_) + command.current_desired;
        return iq_des;
    }
    void set_param(const VelocityControllerParam &param) {
        controller_.set_param(param.velocity);
        acceleration_limit_ = param.acceleration_limit;
        velocity_filter_.set_frequency(param.velocity.velocity_filter_frequency_hz);
    }
    void set_rollover(float rollover) { rollover_ = rollover; controller_.set_rollover(INFINITY); }
 private:
    float velocity_measured_filt_ = 0;
    float last_motor_position_ = 0;
    float rollover_ = 0;
    PIDController controller_;
    FirstOrderLowPassFilter velocity_filter_;
    float acceleration_limit_ = INFINITY;

    friend class System;
};
