#pragma once

#include "messages.h"

class PIDDeadbandController;
class Communication;
class Encoder;

#include <cmath>
#include "control_fun.h"
#include "sincos.h"
#include "led.h"
#include "util.h"
#include "torque_sensor.h"

template<typename FastLoop>
class MainLoop {
 public:
    MainLoop(FastLoop &fast_loop, PIDController &controller,  PIDController &torque_controller, PIDDeadbandController &impedance_controller, Communication &communication, LED &led, Sensor &output_encoder, TorqueSensor &torque, const MainLoopParam &param) : 
        fast_loop_(fast_loop), controller_(controller), torque_controller_(torque_controller), impedance_controller_(impedance_controller), communication_(communication), led_(led), output_encoder_(output_encoder), torque_sensor_(torque) {
          set_param(param);
        }
    void init() {}
    void update() {
      count_++;
      output_encoder_.trigger();
      SendData send_data;
      output_encoder_.read();
      if(count_ % 10 == 0) torque_sensor_.trigger();
      
      last_timestamp_ = timestamp_;
      timestamp_ = get_clock();
      dt_ = (timestamp_ - last_timestamp_) * (1.0f/CPU_FREQUENCY_HZ);

      fast_loop_.get_status(&fast_loop_status_);

      int count_received = communication_.receive_data(&receive_data_);
      if (count_received) {
        if (mode_ != static_cast<MainControlMode>(receive_data_.mode_desired)) {
          set_mode(static_cast<MainControlMode>(receive_data_.mode_desired));
          controller_.init(fast_loop_status_.motor_position.position);
          torque_controller_.init(torque_);
          impedance_controller_.init(fast_loop_status_.motor_position.position);
        }
      }

      
 
      if(count_ % 10 == 0) {
        float tmp_torque = torque_sensor_.read();
      //  if (fabs(tmp_torque) < 2) {
          torque_ = tmp_torque;
      //  }
      }


      float torque_corrected = torque_+.02*fast_loop_status_.foc_status.measured.i_q;
      float torque_filtered = torque_filter_.update(torque_corrected);

      float iq_des = 0;
      float vq_des = 0;
      switch (mode_) {
        case CURRENT:
          iq_des = receive_data_.current_desired;
          break;
        case POSITION:
          iq_des = controller_.step(receive_data_.position_desired, receive_data_.velocity_desired, fast_loop_status_.motor_position.position) + \
                  receive_data_.current_desired;
          break;
        case TORQUE:
          iq_des = torque_controller_.step(receive_data_.torque_desired, 0, torque_filtered) + \
                  receive_data_.current_desired;
          break;
        case IMPEDANCE:
        {
          float torque_des = impedance_controller_.step(receive_data_.position_desired, receive_data_.velocity_desired, 0, fast_loop_status_.motor_position.position) + \
                  receive_data_.torque_desired;
          iq_des = torque_controller_.step(torque_des, 0, torque_filtered) + \
                  receive_data_.current_desired;
        }
          break;
        case VELOCITY:
          // saturate position so that current = current max due to kp, so error max = 
          iq_des = controller_.step(fast_loop_status_.motor_position.position + param_.controller_param.command_max/param_.controller_param.kp*fsignf(receive_data_.velocity_desired), 
                  receive_data_.velocity_desired, fast_loop_status_.motor_position.position, receive_data_.velocity_desired) + \
                  receive_data_.current_desired;
          break;
        case STEPPER_TUNING:
          vq_des = receive_data_.current_desired;
        case POSITION_TUNING: 
          if (count_received) {
            position_trajectory_generator_.set_amplitude(receive_data_.position_desired);
            position_trajectory_generator_.set_frequency(receive_data_.reserved);
          }
          TrajectoryGenerator::TrajectoryValue traj = position_trajectory_generator_.step(dt_);
          float position_desired = traj.value;
          float velocity_desired = traj.value_dot;
          iq_des = controller_.step(position_desired, velocity_desired, fast_loop_status_.motor_position.position);
          if (mode_ == STEPPER_TUNING) {
            //fast_loop_.set_stepper_position(position_desired);
            //fast_loop_.set_stepper_velocity(velocity_desired);
          }
          break;
        
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

      send_data.iq = fast_loop_status_.foc_status.measured.i_q;
      send_data.host_timestamp_received = receive_data_.host_timestamp;
      send_data.mcu_timestamp = fast_loop_status_.timestamp;
      send_data.motor_encoder = fast_loop_status_.motor_mechanical_position;
      send_data.motor_position = fast_loop_status_.motor_position.position;
      send_data.joint_position = output_encoder_.get_value()*2.0*(float) M_PI/param_.output_encoder.cpr;
      send_data.torque = torque_filtered;
      send_data.reserved[0] = torque_;
      //if(count_ % 4 == 0) {
      communication_.send_data(send_data);
      //}
      led_.update();
      last_receive_data_ = receive_data_;
    }
    void set_param(const MainLoopParam &param) {
      controller_.set_param(param.controller_param);
      torque_controller_.set_param(param.torque_controller_param);
      impedance_controller_.set_param(param.impedance_controller_param);
      torque_sensor_.set_param(param.torque_sensor);
      param_ = param;
    }
    void get_status(MainLoopStatus * const main_loop_status) const {}
    void set_mode(MainControlMode mode) {
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
        case VELOCITY:
          fast_loop_.current_mode();
          led_.set_color(LED::BLUE);
          break;
        case TORQUE:
          fast_loop_.current_mode();
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
        case BOARD_RESET:
          NVIC_SystemReset();
          break;
  }
  receive_data_.mode_desired = mode;
    }
    LED* led() { return &led_; }
 private:
    MainLoopParam param_;
    FastLoop &fast_loop_;
    PIDController &controller_;
    PIDController &torque_controller_;
    PIDDeadbandController &impedance_controller_;
    Communication &communication_;
    LED &led_;
    ReceiveData receive_data_ = {};
    ReceiveData last_receive_data_ = {};
    uint64_t count_ = 0;
    FastLoopStatus fast_loop_status_ = {};
    MainControlMode mode_ = OPEN;
    Sensor &output_encoder_;
    TorqueSensor &torque_sensor_;
    IIRFilter torque_filter_;
    float torque_ = 0;
    float dt_ = 0;
    TrajectoryGenerator position_trajectory_generator_;
    uint32_t timestamp_ = 0;
    uint32_t last_timestamp_ = 0;
    template<typename, typename>
    friend class System;


inline uint16_t minu16(uint16_t a, uint16_t b) {
  if (a > b) {
    return b;
  } else {
    return a;
  }
}
};
