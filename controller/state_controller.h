#ifndef UNHUMAN_MOTORLIB_CONTROLLER_STATE_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_STATE_CONTROLLER_H_

#include "controller.h"
#include "../control_fun.h"
#include "../parameter_api.h"

class StateController : public Controller {
 public:
    StateController(float dt) : Controller(dt), velocity_error_filter_(dt), torque_error_filter_(dt), 
        torque_dot_error_filter_(dt), output_filter_(dt), position_desired_filter_(dt) {}
    void init(const MainLoopStatus &status) {
        position_last_ = status.motor_position;
        torque_last_ = status.torque;
        velocity_error_filter_.init(0);
        torque_error_filter_.init(0);
        torque_dot_error_filter_.init(0);
        output_filter_.init(0);
        position_desired_filter_.init(0);
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

        iq_des_ = c.kp*position_error_ + c.kd*velocity_error_ + c.kt*torque_error_ + 
            c.ks*torque_dot_error_ + param_.ff_tau*command.torque_desired + command.current_desired;
        float iq_filtered = output_filter_.update(iq_des_);
        float iq_sat = fsat(iq_filtered, param_.command_max);
        return iq_sat;
    }
    void set_param(const StateControllerParam &param) {
        velocity_error_filter_.set_frequency(param.velocity_filter_frequency_hz);
        torque_error_filter_.set_frequency(param.torque_filter_frequency_hz);
        torque_dot_error_filter_.set_frequency(param.torque_dot_filter_frequency_hz);
        output_filter_.set_frequency(param.output_filter_frequency_hz);
        position_desired_filter_.set_frequency(param.position_desired_filter_frequency_hz);
        param_ = param;
    }
    void set_debug_variables(ParameterAPI &api) {
        api.add_api_variable("state_command_max", new APIFloat(&param_.command_max));
        api.add_api_variable("state_ff_tau", new APIFloat(&param_.ff_tau));
        API_ADD_FILTER(state_output_filter, FirstOrderLowPassFilter, output_filter_);
        API_ADD_FILTER(state_velocity_error_filter, FirstOrderLowPassFilter, velocity_error_filter_);
        API_ADD_FILTER(state_torque_error_filter, FirstOrderLowPassFilter, torque_error_filter_);
        API_ADD_FILTER(state_torque_dot_error_filter, FirstOrderLowPassFilter, torque_dot_error_filter_);
        API_ADD_FILTER(state_position_desired_filter, SecondOrderLowPassFilter, position_desired_filter_);
    }
    void set_rollover(float rollover) { /* doesn't support rollover */ }
    bool is_current_saturated() const {
        return std::abs(iq_des_) >= std::abs(param_.command_max);
    }
 private:
    float position_error_, velocity_error_;
    float position_last_ = 0;
    float torque_error_, torque_dot_error_;
    float torque_last_ = 0;
    float iq_des_ = 0, iq_command_ = 0;
    FirstOrderLowPassFilter velocity_error_filter_;
    FirstOrderLowPassFilter torque_error_filter_;
    FirstOrderLowPassFilter torque_dot_error_filter_;
    FirstOrderLowPassFilter output_filter_;
    SecondOrderLowPassFilter position_desired_filter_;
    StateControllerParam param_ = {};
    friend class System;
    friend void config_init();
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_STATE_CONTROLLER_H_
