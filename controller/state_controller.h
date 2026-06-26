#ifndef UNHUMAN_MOTORLIB_CONTROLLER_STATE_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_STATE_CONTROLLER_H_

#include "controller.h"
#include "../control_fun.h"
#include "../parameter_api.h"

#define STATE_CONTROLLER_DEBUG_VARIABLES(api, sc) \
    api.add_api_variable("state_command_max", new APIFloat(&sc.param_.command_max));\
    api.add_api_variable("state_ff_tau", new APIFloat(&sc.param_.ff_tau));\
    API_ADD_FILTER_WITH_API(api, state_output_filter, sc.output_filter_);\
    API_ADD_FILTER_WITH_API(api, state_velocity_error_filter, sc.velocity_error_filter_);\
    API_ADD_FILTER_WITH_API(api, state_torque_error_filter, sc.torque_error_filter_);\
    API_ADD_FILTER_WITH_API(api, state_torque_dot_error_filter, sc.torque_dot_error_filter_);\
    API_ADD_FILTER_WITH_API(api, state_position_desired_filter, sc.position_desired_filter_);\

template<float dt>
class StateController {
 public:
    StateController() : velocity_error_filter_(dt), torque_error_filter_(dt), 
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
        float velocity = (status.motor_position - position_last_)/dt;
        velocity_error_ = velocity_error_filter_.update(c.velocity_desired - velocity);
        position_last_ = status.motor_position;

        torque_error_ = torque_error_filter_.update(c.torque_desired - status.torque);
        float torque_dot = (status.torque - torque_last_)/dt;
        torque_dot_error_ = torque_dot_error_filter_.update(c.torque_dot_desired - torque_dot);
        torque_last_ = status.torque;

        float iq_des = c.kp*position_error_ + c.kd*velocity_error_ + c.kt*torque_error_ + 
            c.ks*torque_dot_error_ + param_.ff_tau*command.torque_desired + command.current_desired;
        float iq_filtered = output_filter_.update(iq_des);
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

    void set_rollover(float rollover) { /* doesn't support rollover */ }
    bool validate_command(const MotorCommand &command) const {
        const StateControllerCommand &c = command.state;
        if (std::isfinite(c.position_desired) && std::isfinite(c.velocity_desired) && 
            std::isfinite(c.torque_desired) && std::isfinite(c.torque_dot_desired) && 
            std::isfinite(c.current_desired) &&
            std::isfinite(c.kp) && std::isfinite(c.kd) && std::isfinite(c.kt) && 
            std::isfinite(c.ks)) {
            return true;
        }
        return false;
    }
 private:
    float position_error_, velocity_error_;
    float position_last_ = 0;
    float torque_error_, torque_dot_error_;
    float torque_last_ = 0;
    FirstOrderLowPassFilter velocity_error_filter_;
    FirstOrderLowPassFilter torque_error_filter_;
    FirstOrderLowPassFilter torque_dot_error_filter_;
    FirstOrderLowPassFilter output_filter_;
    SecondOrderLowPassFilter position_desired_filter_;
    StateControllerParam param_ = {};
    template <typename T> friend class SystemBase;
    friend void config_init();
};

static_assert(IsController<StateController<.0001>>, "State controller fails to meet IsController interface requirement");

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_STATE_CONTROLLER_H_
