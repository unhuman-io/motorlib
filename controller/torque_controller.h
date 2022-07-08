#include "controller.h"
#include "../control_fun.h"

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
 private:
    PIDController controller_;

    friend class System;
};
