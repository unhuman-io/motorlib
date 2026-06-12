#ifndef UNHUMAN_MOTORLIB_CONTROLLER_TORQUE_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_TORQUE_CONTROLLER_H_

#include "controller.h"
#include "../control_fun.h"
#include "../parameter_api.h"

#define TORQUE_CONTROLLER_DEBUG_VARIABLES(api, tc) \
    api.add_api_variable("tkp", new APIFloat(&tc.controller_.kp_)); \
    api.add_api_variable("tkd", new APIFloat(&tc.controller_.kd_)); \
    api.add_api_variable("tki", new APIFloat(&tc.controller_.ki_)); \
    api.add_api_variable("tki_limit", new APIFloat(&tc.controller_.ki_limit_)); \
    API_ADD_FILTER_WITH_API(api, t_velocity_filter, tc.controller_.velocity_filter_); \
    API_ADD_FILTER_WITH_API(api, t_output_filter, tc.controller_.output_filter_); \
    api.add_api_variable("tmax", new APIFloat(&tc.controller_.command_max_)); \

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

    bool validate_command(const MotorCommand &command) const {
        if (std::isfinite(command.torque_desired) &&
            std::isfinite(command.current_desired)) {
            return true;
        }
        return false;
    }
 private:
    PIDController controller_;

    template <typename T> friend class SystemBase;
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_TORQUE_CONTROLLER_H_
