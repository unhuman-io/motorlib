#ifndef UNHUMAN_MOTORLIB_CONTROLLER_TORQUE_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_TORQUE_CONTROLLER_H_

#include "controller.h"
#include "../control_fun.h"
#include "../parameter_api.h"

class TorqueController : public Controller {
 public:
    TorqueController(float dt) : Controller(dt), controller_(dt) {}
    void init(const MainLoopStatus &status) {
        controller_.init(status.torque);
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        float iq_des = controller_.step(command.torque_desired, 0, status.torque) + \
                  command.current_desired;
        return iq_des;
    }
    void set_param(const TorqueControllerParam &param) {
        controller_.set_param(param.torque);
    }
    void set_debug_variables(ParameterAPI &api) {
        api.add_api_variable("tkp", new APIFloat(&controller_.kp_));
        api.add_api_variable("tkd", new APIFloat(&controller_.kd_));
        api.add_api_variable("tki", new APIFloat(&controller_.ki_));
        api.add_api_variable("tki_limit", new APIFloat(&controller_.ki_limit_));
        API_ADD_FILTER(t_velocity_filter, SecondOrderLowPassFilter, controller_.velocity_filter_);
        API_ADD_FILTER(t_output_filter, FirstOrderLowPassFilter, controller_.output_filter_);
        api.add_api_variable("tmax", new APIFloat(&controller_.command_max_));
    }
 private:
    PIDController controller_;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_TORQUE_CONTROLLER_H_
