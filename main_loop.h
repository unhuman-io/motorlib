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

void setup_sleep();
void finish_sleep();

class MainLoop {
 public:
    MainLoop(FastLoop &fast_loop, PositionController &position_controller,  TorqueController &torque_controller, 
        ImpedanceController &impedance_controller, VelocityController &velocity_controller, Communication &communication,
        LED &led, OutputEncoder &output_encoder, TorqueSensor &torque, const MainLoopParam &param) : 
          param_(param), fast_loop_(fast_loop), position_controller_(position_controller), torque_controller_(torque_controller), 
          impedance_controller_(impedance_controller), velocity_controller_(velocity_controller), 
          communication_(communication), led_(led), output_encoder_(output_encoder), torque_sensor_(torque),
          output_encoder_correction_table_(param_.output_encoder.table) {
          set_param(param);
        }
    void init() {}
    void update() {
      count_++;
      output_encoder_.trigger();
      SendData send_data;
      torque_sensor_.trigger();
      
      last_timestamp_ = timestamp_;
      timestamp_ = get_clock();
      dt_ = (timestamp_ - last_timestamp_) * (1.0f/CPU_FREQUENCY_HZ);

      fast_loop_.get_status(&status_.fast_loop);

      ReceiveData receive_data;
      int count_received = communication_.receive_data(&receive_data);
      bool command_received = false;
      if (started_ && ((count_received && !safe_mode_) || (count_received && receive_data.mode_desired == param_.safe_mode))) {
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

      // internal command, not recommended in conjunction with host_timeout or safe mode
      if (internal_command_received_) {
        receive_data_ = internal_command_;
        internal_command_received_ = false;
        command_received = true;
      }

      if (command_received) {
        if (mode_ != static_cast<MainControlMode>(receive_data_.mode_desired) || mode_ == SLEEP) {
          set_mode(static_cast<MainControlMode>(receive_data_.mode_desired));
        }
      }

      int32_t output_encoder_raw = output_encoder_.read();
      float output_encoder_x = (output_encoder_raw % (int32_t) param_.output_encoder.cpr) / (float) param_.output_encoder.cpr;
      float output_encoder_rad = output_encoder_raw*2.0*(float) M_PI/param_.output_encoder.cpr;
      status_.output_position = output_encoder_rad + param_.output_encoder.bias 
        + output_encoder_correction_table_.table_interp(output_encoder_x);

      status_.motor_position = status_.fast_loop.motor_position.position + motor_encoder_bias_;

      float torque_corrected = torque_sensor_.read();
      //if (torque_corrected != status_.torque) {
        torque_corrected += param_.torque_correction*status_.fast_loop.foc_status.measured.i_q;
      //}
      status_.torque = torque_corrected;

      if (status_.motor_position > param_.encoder_limits.motor_hard_max ||
          status_.motor_position < param_.encoder_limits.motor_hard_min ||
          status_.output_position > param_.encoder_limits.output_hard_max ||
          status_.output_position < param_.encoder_limits.output_hard_min) {
          safe_mode_ = true;
          set_mode(param_.safe_mode);
      }

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
        case STEPPER_VELOCITY:
          vq_des = receive_data_.stepper_velocity.voltage;
          fast_loop_.set_stepper_velocity(receive_data_.stepper_velocity.velocity);
          break;
        case STEPPER_TUNING:
          {
            if (count_received) {
              position_trajectory_generator_.set_amplitude(receive_data_.stepper_tuning.amplitude);
              position_trajectory_generator_.set_frequency(receive_data_.stepper_tuning.frequency);
              position_trajectory_generator_.set_mode((TuningMode) receive_data_.stepper_tuning.mode);
            }
            TrajectoryGenerator::TrajectoryValue traj = position_trajectory_generator_.step(dt_);
            fast_loop_.set_stepper_position(traj.value);
            fast_loop_.set_stepper_velocity(traj.value_dot);
            vq_des = receive_data_.stepper_tuning.kv*traj.value_dot;
            break;
          }
        case POSITION_TUNING: 
          {
            if (count_received) {
              position_trajectory_generator_.set_amplitude(receive_data_.position_tuning.amplitude);
              position_trajectory_generator_.set_frequency(receive_data_.position_tuning.frequency);
              position_trajectory_generator_.set_mode((TuningMode) receive_data_.position_tuning.mode);
            }
            TrajectoryGenerator::TrajectoryValue traj = position_trajectory_generator_.step(dt_);
            ReceiveData trajectory = {};
            float position_desired = traj.value;
            float velocity_desired = traj.value_dot;
            trajectory.position_desired = position_desired+receive_data_.position_tuning.bias;
            trajectory.velocity_desired = velocity_desired;
            iq_des = position_controller_.step(trajectory, status_);
            break;
          }
        case CURRENT_TUNING:
          if (count_received) {
            fast_loop_.set_tuning_amplitude(receive_data_.current_tuning.amplitude);
            fast_loop_.set_tuning_frequency(receive_data_.current_tuning.frequency);
            fast_loop_.set_tuning_bias(receive_data_.current_tuning.bias);
            fast_loop_.set_tuning_square(receive_data_.current_tuning.mode == TuningMode::SQUARE);
            if (receive_data_.current_tuning.mode == TuningMode::CHIRP) { // flag for chirp mode
              fast_loop_.set_tuning_chirp(true, receive_data_.current_tuning.frequency);
            } else {
              fast_loop_.set_tuning_chirp(false, 0);
            }
          }
          break;
        case VOLTAGE:
          vq_des = receive_data_.voltage.voltage_desired;
          break;
        case PHASE_LOCK:
          fast_loop_.phase_lock_mode(receive_data_.current_desired);
          break;
        default:
          break;
      }

      if ((status_.motor_position > param_.encoder_limits.motor_controlled_max && iq_des >= 0) ||
          (status_.motor_position < param_.encoder_limits.motor_controlled_min && iq_des <= 0)) {
          if (mode_ != VELOCITY && mode_ != param_.safe_mode) {
            set_mode(VELOCITY);
          }
          MotorCommand tmp_receive_data = receive_data_;
          tmp_receive_data.velocity_desired = 0;
          iq_des = velocity_controller_.step(tmp_receive_data, status_);
      }

      fast_loop_.set_iq_des(iq_des);
      fast_loop_.set_vq_des(vq_des);

      send_data.iq = status_.fast_loop.foc_status.measured.i_q;
      send_data.host_timestamp_received = receive_data_.host_timestamp;
      send_data.mcu_timestamp = status_.fast_loop.timestamp;
      send_data.motor_encoder = status_.fast_loop.motor_position.raw;
      send_data.motor_position = status_.motor_position;
      send_data.joint_position = status_.output_position;
      send_data.torque = status_.torque + t0;
      send_data.reserved[0] = 0;
      send_data.reserved[1] = *reinterpret_cast<float *>(reserved1_);
      send_data.reserved[2] = *reinterpret_cast<float *>(reserved2_);
      communication_.send_data(send_data);
      led_.update();
      last_receive_data_ = receive_data_;
      IWDG->KR = 0xAAAA;

      last_torque = status_.torque;
    }


    void set_param(const MainLoopParam &param) {
      position_controller_.set_param(param.position_controller_param);
      torque_controller_.set_param(param.torque_controller_param);
      impedance_controller_.set_param(param.impedance_controller_param);
      velocity_controller_.set_param(param.velocity_controller_param);
      torque_sensor_.set_param(param.torque_sensor);
      if (param_.encoder_limits.motor_hard_max == param_.encoder_limits.motor_hard_min) {
        param_.encoder_limits.motor_hard_max = INFINITY;
        param_.encoder_limits.motor_hard_min = -INFINITY;
      }
      if (param_.encoder_limits.motor_controlled_max == param_.encoder_limits.motor_controlled_min) {
        param_.encoder_limits.motor_controlled_max = INFINITY;
        param_.encoder_limits.motor_controlled_min = -INFINITY;
      }
      if (param_.encoder_limits.output_hard_max == param_.encoder_limits.output_hard_min) {
        param_.encoder_limits.output_hard_max = INFINITY;
        param_.encoder_limits.output_hard_min = -INFINITY;
      }
    }
    void set_rollover(float rollover) {
      position_controller_.set_rollover(rollover);
      impedance_controller_.set_rollover(rollover);
      velocity_controller_.set_rollover(rollover);
    }
    void set_motor_encoder_bias(float bias) { motor_encoder_bias_ = bias; }
    void get_status(MainLoopStatus * const main_loop_status) const { *main_loop_status = status_; }
    void set_started() { started_ = true; }
    void set_mode(MainControlMode mode) {
      if (mode != mode_) {
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
          case STEPPER_VELOCITY:
          case STEPPER_TUNING:
            fast_loop_.stepper_mode();
            led_.set_color(LED::CYAN);
            break;
          case SLEEP:
            led_.set_color(LED::WHITE);
            led_.set_on_dim();
            fast_loop_.open_mode();
            setup_sleep();
            while(!communication_.new_rx_data()) {
              __WFI();
            }
            finish_sleep();
            break;
          case CRASH:
            led_.set_color(LED::RED);
            while(1) {
              led_.update();
              ns_delay(2000);
            }
            break;
          case BOARD_RESET:
            NVIC_SystemReset();
            break;
        }
      }
      receive_data_.mode_desired = mode;
    }

    // use to set the command from another low priority source than communication, 
    // such as from the System or Actuator classes
    void set_command(const MotorCommand &command) {
      internal_command_ = command;
      internal_command_received_ = true;
    }
    
    void set_t0() {
      t0 = -last_torque;
    }

    float get_t0() {
      return t0;
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
    MotorCommand internal_command_;
    bool internal_command_received_ = false;
    uint64_t count_ = 0;
    uint16_t no_command_ = 0;
    bool safe_mode_ = false;
    bool started_ = false;
    MainLoopStatus status_ = {};
    MainControlMode mode_ = NO_MODE;
    OutputEncoder &output_encoder_;
    float motor_encoder_bias_ = 0;
    TorqueSensor &torque_sensor_;
    float dt_ = 0;
    TrajectoryGenerator position_trajectory_generator_;
    uint32_t timestamp_ = 0;
    uint32_t last_timestamp_ = 0;
    uint32_t *reserved1_ = &timestamp_;
    uint32_t *reserved2_ = &last_timestamp_;
    float output_encoder_pos_;
    PChipTable<OUTPUT_ENCODER_TABLE_LENGTH> output_encoder_correction_table_;
    float t0 = 0;
    float last_torque = 0;

    friend class System;
    friend void system_init();
    friend void config_init();
};
