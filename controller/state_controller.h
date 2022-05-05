#include "controller.h"
#include "../control_fun.h"

class StateController : public Controller {
 public:
    StateController(float dt) : Controller(dt), velocity_error_filter_(dt), torque_error_filter_(dt), torque_dot_error_filter_(dt) {}
    void init(const MainLoopStatus &status) {
        position_last_ = status.motor_position;
        torque_last_ = status.torque;
        velocity_error_filter_.init(0);
        torque_error_filter_.init(0);
        torque_dot_error_filter_.init(0);
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        const StateControllerCommand &c = command.state;
        position_error_ = c.position_desired - status.motor_position;
        float velocity = (status.motor_position - position_last_)/dt_;
        velocity_error_ = velocity_error_filter_.update(c.velocity_desired - velocity);
        position_last_ = status.motor_position;

        torque_error_ = torque_error_filter_.update(c.torque_desired - status.torque);
        float torque_dot = (status.torque - torque_last_)/dt_;
        torque_dot_error_ = torque_dot_error_filter_.update(c.torque_dot_desired - torque_dot);
        torque_last_ = status.torque;

        float iq_des = c.kp*position_error_ + c.kd*velocity_error_ + c.kt*torque_error_ + c.ks*torque_dot_error_;
        return iq_des;
    }
    void set_param(const StateControllerParam &param) {
        velocity_error_filter_.set_frequency(param.velocity_filter_frequency_hz);
        torque_error_filter_.set_frequency(param.torque_filter_frequency_hz);
        torque_dot_error_filter_.set_frequency(param.torque_dot_filter_frequency_hz);
    }
    void set_rollover(float rollover) { /* doesn't support rollover */ }
 private:
    float position_error_, velocity_error_;
    float position_last_ = 0;
    float torque_error_, torque_dot_error_;
    float torque_last_ = 0;
    FirstOrderLowPassFilter velocity_error_filter_;
    FirstOrderLowPassFilter torque_error_filter_;
    FirstOrderLowPassFilter torque_dot_error_filter_;
    friend class System;
};
