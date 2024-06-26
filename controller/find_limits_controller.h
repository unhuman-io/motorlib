#ifndef UNHUMAN_MOTORLIB_CONTROLLER_FIND_LIMITS_CONTROLLER_H_
#define UNHUMAN_MOTORLIB_CONTROLLER_FIND_LIMITS_CONTROLLER_H_

#include "controller.h"
#include "../logger.h"
#include "../control_fun.h"

class FindLimitsController : public Controller {
 public:
    FindLimitsController(float dt, VelocityController vc, JointPositionController jpc) 
        : Controller(dt), velocity_controller_(vc), joint_position_controller_(jpc),
        iq_find_limits_filter_(dt, 1) {}
    void init(const MainLoopStatus &status) {
        velocity_controller_.init(status);
        joint_position_controller_.init(status);
        find_limits_state_ = FIND_FIRST_LIMIT;
    }
    float step(const MotorCommand &command, const MainLoopStatus &status) {
        const FindLimitsCommand &command_current = command.find_limits

        MotorCommand velocity_command = {.current_desired = command.current_desired, .velocity_desired = velocity_des};
        float iq_des = velocity_controller_.step(velocity_command, status);

        iq_find_limits_filter_.update(status.fast_loop.foc_status.measured.i_q);
        ReceiveData command = {};
        switch (find_limits_state_) {
        case FIND_FIRST_LIMIT:
            command.velocity_desired = command_current_.velocity_desired;
            if (iq_find_limits_filter_.get_value() > command_current_.current_desired) {
            find_limits_state_ = FIND_SECOND_LIMIT;
            // record positive limit
            //motor_positive_limit_ = status_.motor_position;
            //output_positive_limit_ = status_.output_position;
            adjust_output_encoder(-(status_.output_position - encoder_limits_.output_hard_max));
            adjust_motor_encoder(-(status_.motor_position - encoder_limits_.motor_hard_max));
            velocity_controller_.init(status_);
            }
            iq_des = velocity_controller_.step(command, status_);
            break;
        case FIND_SECOND_LIMIT:
            command.velocity_desired = -command_current_.velocity_desired;
            if (iq_find_limits_filter_.get_value() < -command_current_.current_desired) {
            find_limits_state_ = VELOCITY_TO_POSITION;
            // record negative limit
            // change encoder biases around
            logger.log_printf("negative limit measured: output: %f, motor: %f", status_.output_position, status_.motor_position);
            adjust_output_encoder(-(status_.output_position - encoder_limits_.output_hard_min));
            adjust_motor_encoder(-(status_.motor_position - encoder_limits_.motor_hard_min));
            logger.log_printf("obias: %f", output_encoder_bias_);
            velocity_controller_.init(status_);
            }
            iq_des = velocity_controller_.step(command, status_);
            break;
        case VELOCITY_TO_POSITION:
            command.velocity_desired = command_current_.velocity_desired;
            if (status_.motor_position >= command_current_.position_desired) {
            find_limits_state_ = GOTO_POSITION;
            // record negative limit
            // change encoder biases around
            joint_position_controller_.init(status_);
            find_limits_count_ = 0;
            }
            iq_des = velocity_controller_.step(command, status_);
            break;
        case GOTO_POSITION:
            //position_limits_disable_ = position_limits_disable_last_;
            command.position_desired = command_current_.position_desired;
            iq_des = joint_position_controller_.step(command, status_);
            if (find_limits_count_ == 10000) {
            // set startup bias 0

            // set startup bias negative measured
            logger.log_printf("mbias: %f", motor_encoder_bias_);
            } else {
            find_limits_count_++;
            }
            break;
        }

        return iq_des;
    }

 private:
    VelocityController &velocity_controller_;
    JointPositionController &joint_position_controller_;

    uint32_t find_limits_count_ = 0;
    enum FindLimitsState {FIND_FIRST_LIMIT, FIND_SECOND_LIMIT, VELOCITY_TO_POSITION, GOTO_POSITION} find_limits_state_;
    FirstOrderLowPassFilter iq_find_limits_filter_;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_CONTROLLER_FIND_LIMITS_CONTROLLER_H_
