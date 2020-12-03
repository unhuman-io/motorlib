#pragma once

#include "messages.h"

#include <cmath>
#include "control_fun.h"
#include "sincos.h"
#include "led.h"
#include "util.h"
#include "torque_sensor.h"

extern "C" {
void system_init();
}

class MainLoop {
 public:
    MainLoop(FastLoop &fast_loop, PositionController &position_controller,  TorqueController &torque_controller, 
        ImpedanceController &impedance_controller, VelocityController &velocity_controller, Communication &communication,
        LED &led, OutputEncoder &output_encoder, TorqueSensor &torque, const MainLoopParam &param) : 
          fast_loop_(fast_loop), position_controller_(position_controller), torque_controller_(torque_controller), 
          impedance_controller_(impedance_controller), velocity_controller_(velocity_controller), 
          communication_(communication), led_(led), output_encoder_(output_encoder), torque_sensor_(torque) {
          set_param(param);
        }
    void init() {}
    void update() {
      count_++;
      output_encoder_.trigger();
      SendData send_data;
      output_encoder_.read();
      torque_sensor_.trigger();
      
      last_timestamp_ = timestamp_;
      timestamp_ = get_clock();
      dt_ = (timestamp_ - last_timestamp_) * (1.0f/CPU_FREQUENCY_HZ);

      fast_loop_.get_status(&status_.fast_loop);

      ReceiveData receive_data;
      int count_received = communication_.receive_data(&receive_data);
      bool command_received = false;
      if ((count_received && !safe_mode_) || (count_received && receive_data.mode_desired == param_.safe_mode)) {
        no_command_ = 0;
        receive_data_ = receive_data;
        command_received = true;
        safe_mode_ = false;
      } else {
        no_command_++;
        if (no_command_ > 16000)
           no_command_ = 16000;
      } 
        
      if (param_.host_timeout && no_command_ > param_.host_timeout && started_) {
          safe_mode_ = true;
          set_mode(param_.safe_mode);
      }

      if (command_received) {
        if (mode_ != static_cast<MainControlMode>(receive_data_.mode_desired)) {
          set_mode(static_cast<MainControlMode>(receive_data_.mode_desired));
        }
      }

      float torque_corrected = torque_sensor_.read();
      if (torque_corrected != torque_) {
        torque_corrected += param_.torque_correction*status_.fast_loop.foc_status.measured.i_q;
      }
      torque_ = torque_corrected;
   
      //float torque_filtered = torque_filter_.update(torque_corrected);
      status_.torque_filtered = torque_corrected;

      float iq_des = 0;
      float vq_des = 0;
      switch (mode_) {
        case CURRENT:
          iq_des = receive_data_.current_desired;
          break;
        case POSITION:
          iq_des = position_controller_.step(receive_data_, status_);
          break;
        case TORQUE:
          iq_des = torque_controller_.step(receive_data_, status_);
          break;
        case IMPEDANCE:
          iq_des = impedance_controller_.step(receive_data_, status_);
          break;
        case VELOCITY:
          iq_des = velocity_controller_.step(receive_data_, status_);
          break;
        case STEPPER_TUNING:
        case POSITION_TUNING: 
          {
            if (count_received) {
              position_trajectory_generator_.set_amplitude(receive_data_.position_desired);
              position_trajectory_generator_.set_frequency(receive_data_.reserved);
            }
            TrajectoryGenerator::TrajectoryValue traj = position_trajectory_generator_.step(dt_);
            ReceiveData trajectory = {};
            float position_desired = traj.value;
            float velocity_desired = traj.value_dot;
            trajectory.position_desired = position_desired;
            trajectory.velocity_desired = velocity_desired;
            iq_des = position_controller_.step(trajectory, status_);
            if (mode_ == STEPPER_TUNING) {
              if (receive_data_.velocity_desired < 0) {
                vq_des = fabsf(receive_data_.velocity_desired*velocity_desired); // kv in v/rad/s is in receive_data_.velocity_desired
              } else if (receive_data_.velocity_desired >= 0) {
                vq_des = receive_data_.current_desired;
                if (receive_data_.velocity_desired > 0) {
                  fast_loop_.set_stepper_velocity(receive_data_.velocity_desired);
                  break;
                }
              }
              fast_loop_.set_stepper_position(position_desired);
              fast_loop_.set_stepper_velocity(velocity_desired);
            }
            break;
          }
        case CURRENT_TUNING:
          if (count_received) {
            if (receive_data_.current_desired < 0) { // flag for chirp mode
              fast_loop_.set_tuning_amplitude(-receive_data_.current_desired);
              fast_loop_.set_tuning_chirp(true, fabsf(receive_data_.reserved));
            } else {
              fast_loop_.set_tuning_chirp(false, 0);
              fast_loop_.set_tuning_amplitude(receive_data_.current_desired);
              fast_loop_.set_tuning_frequency(receive_data_.reserved);
            }
          }
          break;
        case VOLTAGE:
          vq_des = receive_data_.reserved;
          break;
        case PHASE_LOCK:
          fast_loop_.phase_lock_mode(receive_data_.current_desired);
          break;
        default:
          break;
      }

      fast_loop_.set_iq_des(iq_des);
      fast_loop_.set_vq_des(vq_des);

      send_data.iq = status_.fast_loop.foc_status.measured.i_q;
      send_data.host_timestamp_received = receive_data_.host_timestamp;
      send_data.mcu_timestamp = status_.fast_loop.timestamp;
      send_data.motor_encoder = status_.fast_loop.motor_position.raw;
      send_data.motor_position = status_.fast_loop.motor_position.position;
      send_data.joint_position = output_encoder_.get_value()*2.0*(float) M_PI/param_.output_encoder.cpr;
      send_data.torque = status_.torque_filtered;
      send_data.reserved[0] = torque_;
      send_data.reserved[1] = *reinterpret_cast<float *>(reserved1_);
      send_data.reserved[2] = *reinterpret_cast<float *>(reserved2_);
      communication_.send_data(send_data);
      led_.update();
      last_receive_data_ = receive_data_;
    }


    void set_param(const MainLoopParam &param) {
      position_controller_.set_param(param.position_controller_param);
      torque_controller_.set_param(param.torque_controller_param);
      impedance_controller_.set_param(param.impedance_controller_param);
      velocity_controller_.set_param(param.velocity_controller_param);
      torque_sensor_.set_param(param.torque_sensor);
      param_ = param;
    }
    void set_rollover(float rollover) {
      position_controller_.set_rollover(rollover);
      impedance_controller_.set_rollover(rollover);
      velocity_controller_.set_rollover(rollover);
    }
    void get_status(MainLoopStatus * const main_loop_status) const {}
    void set_mode(MainControlMode mode) {
      started_ = true;
        mode_ = mode;
      switch (mode) {
        case OPEN:
        default:
          fast_loop_.open_mode();
          led_.set_color(LED::AZURE);
          break;
        case DAMPED:
          fast_loop_.brake_mode();
          led_.set_color(LED::ORANGE);
          break;
        case CURRENT:
          fast_loop_.current_mode();
          led_.set_color(LED::GREEN);
          break;
        case CURRENT_TUNING:
          fast_loop_.current_tuning_mode();
          led_.set_color(LED::SPRING);
          break;
        case POSITION_TUNING:
        case POSITION:
          position_controller_.init(status_);
        case VELOCITY:
          fast_loop_.current_mode();
          velocity_controller_.init(status_);
          led_.set_color(LED::BLUE);
          break;
        case IMPEDANCE:
          fast_loop_.current_mode();
          impedance_controller_.init(status_);
          led_.set_color(LED::CHARTREUSE);
          break;
        case TORQUE:
          fast_loop_.current_mode();
          torque_controller_.init(status_);
          led_.set_color(LED::ROSE);
          break;
        case VOLTAGE:
          fast_loop_.voltage_mode();
          led_.set_color(LED::VIOLET);
          break;
        case PHASE_LOCK:
          fast_loop_.phase_lock_mode(0);
          led_.set_color(LED::YELLOW);
          break;
        case STEPPER_TUNING:
          fast_loop_.stepper_mode();
          led_.set_color(LED::CYAN);
          break;
        case BOARD_RESET:
          NVIC_SystemReset();
          break;
  }
  receive_data_.mode_desired = mode;
    }
    
 private:
    LED* led() { return &led_; }
    MainLoopParam param_;
    FastLoop &fast_loop_;
    PositionController &position_controller_;
    TorqueController &torque_controller_;
    ImpedanceController &impedance_controller_;
    VelocityController &velocity_controller_;
    Communication &communication_;
    LED &led_;
    ReceiveData receive_data_ = {};
    ReceiveData last_receive_data_ = {};
    uint64_t count_ = 0;
    uint16_t no_command_ = 0;
    bool safe_mode_ = false;
    bool started_ = false;
    MainLoopStatus status_ = {};
    MainControlMode mode_ = OPEN;
    OutputEncoder &output_encoder_;
    TorqueSensor &torque_sensor_;
    IIRFilter torque_filter_;
    float torque_ = 0;
    float dt_ = 0;
    TrajectoryGenerator position_trajectory_generator_;
    uint32_t timestamp_ = 0;
    uint32_t last_timestamp_ = 0;
    uint32_t *reserved1_ = &timestamp_;
    uint32_t *reserved2_ = &last_timestamp_;

    friend class System;
    friend void system_init();


inline uint16_t minu16(uint16_t a, uint16_t b) {
  if (a > b) {
    return b;
  } else {
    return a;
  }
}
};
