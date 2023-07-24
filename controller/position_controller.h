#ifndef UNHUMAN_MOTORLIB_CONTROLLER_POSITION_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_POSITION_CONTROLLER_H_

#include "controller.h"
#include "../control_fun.h"

class PositionController : public Controller {
 public:
    PositionController(float dt, uint32_t tracking_timeout_count=5000) : Controller(dt), controller_(dt), desired_filter_(dt),
        tracking_timeout_count_(tracking_timeout_count) {}
    void init(const MainLoopStatus &status) {
        controller_.init(status.motor_position);
        desired_filter_.init(status.motor_position);
        tracking_fault_ = false;
        tracking_count_ = 0;
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        float position_desired = desired_filter_.update(command.position_desired);
        float iq_des = controller_.step(position_desired, command.velocity_desired, status.motor_position, velocity_limit_) + \
                  command.current_desired;
        
        if (tracking_count_ < tracking_timeout_count_) {
            tracking_count_++;
        } else {
            if (fabsf2(position_desired - status.motor_position) > tracking_tolerance_) {
                tracking_fault_ = true;
            }
        }
        return iq_des;
    }
    void set_rollover(float rollover) { controller_.set_rollover(rollover); }
    void set_param(const PositionControllerParam &param) {
        controller_.set_param(param.position);
        velocity_limit_ = param.velocity_limit;
        desired_filter_.set_frequency(param.desired_filter_hz);
        if (param.tracking_tolerance != 0) {
            tracking_tolerance_ = param.tracking_tolerance;
        }
    }
    bool tracking_fault() const {
        return tracking_fault_;
    }
 private:
    PIDController controller_;
    float velocity_limit_ = INFINITY;
    SecondOrderLowPassFilter desired_filter_;
    bool tracking_fault_;
    uint32_t tracking_count_;
    uint32_t tracking_timeout_count_;
    float tracking_tolerance_ = INFINITY;

    friend class System;
    friend void config_init();
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_POSITION_CONTROLLER_H_
