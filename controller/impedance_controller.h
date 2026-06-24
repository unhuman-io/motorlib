#ifndef UNHUMAN_MOTORLIB_CONTROLLER_IMPEDANCE_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_IMPEDANCE_CONTROLLER_H_

#include "controller.h"
#include "../control_fun.h"

template<float dt>
class ImpedanceController {
 public:
    ImpedanceController() : impedance_controller_(dt), torque_controller_(dt) {}
    void init(const MainLoopStatus &status) {
        impedance_controller_.init(status.motor_position);
        torque_controller_.init(status.torque);
    }
    float step(const MotorCommand &command_raw, const MainLoopStatus &status) {
        const ImpedanceCommand &command = command_raw.impedance;
        if (command.stiffness == 0) {
            impedance_controller_.kp_ = kp_default_;
        } else {
            impedance_controller_.kp_ = command.stiffness;
        }
        if (command.damping == 0) {
            impedance_controller_.kd_ = kd_default_;
        } else {
            impedance_controller_.kd_ = command.damping;
        }
        float torque_des = impedance_controller_.step(command.position_desired, command.velocity_desired, 0, status.motor_position) + \
                  command.torque_desired;
        float torque_dot_des = (torque_des - last_torque_des_) / dt + command.torque_dot_desired;
        last_torque_des_ = torque_des;
        float iq_des = torque_controller_.step(torque_des, torque_dot_des, status.torque) + \
                  command.current_desired;
        return iq_des;
    }
    void set_param(const ImpedanceControllerParam &param) {
        impedance_controller_.set_param(param.impedance);
        torque_controller_.set_param(param.torque);
        kp_default_ = param.impedance.kp;
        kd_default_ = param.impedance.kd;
    }
    void set_rollover(float rollover) { impedance_controller_.set_rollover(rollover); }
    bool validate_command(const MotorCommand &command_raw) const {
        const ImpedanceCommand &command = command_raw.impedance;
        if (std::isfinite(command.position_desired) &&
            std::isfinite(command.velocity_desired) &&
            std::isfinite(command.torque_desired) &&
            std::isfinite(command.current_desired) &&
            std::isfinite(command.torque_dot_desired) &&
            std::isfinite(command.stiffness) &&
            std::isfinite(command.damping)) {
            return true;
        }
        return false;
    }
 private:
    PIDController impedance_controller_;
    PIDController torque_controller_;
    float last_torque_des_ = 0;
    float kp_default_ = 0;
    float kd_default_ = 0;

    template <typename T> friend class SystemBase;
};

static_assert(IsController<ImpedanceController<.0001>>, "Impedance controller fails to meet IsController interface requirement");

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_IMPEDANCE_CONTROLLER_H_
