#pragma once

#include "messages.h"

#include <cmath>
#include "control_fun.h"
#include "sincos.h"
#include "led.h"
#include "util.h"
#include "torque_sensor.h"
#include "hardware_brake.h"

extern "C" {
void system_init();
}

void setup_sleep();
void finish_sleep();

class MainLoop;
void load_send_data(const MainLoop &main_loop, SendData * const data);

#ifndef HARDWARE_BRAKE
using HardwareBrake = HardwareBrakeBase;
#endif

class MainLoop {
 public:
    MainLoop(FastLoop &fast_loop, PositionController &position_controller,  TorqueController &torque_controller, 
        ImpedanceController &impedance_controller, VelocityController &velocity_controller, StateController &state_controller, Communication &communication,
        LED &led, OutputEncoder &output_encoder, TorqueSensor &torque, const MainLoopParam &param, GPIO &gpio_in=no_gpio_, GPIO &gpio_out=no_gpio_,
        HardwareBrake &brake=no_brake_) : 
          param_(param), fast_loop_(fast_loop), position_controller_(position_controller), torque_controller_(torque_controller), 
          impedance_controller_(impedance_controller), velocity_controller_(velocity_controller), state_controller_(state_controller),  
          communication_(communication), led_(led), output_encoder_(output_encoder), torque_sensor_(torque),
          output_encoder_correction_table_(param_.output_encoder.table), brake_(brake), gpio_in_(gpio_in), gpio_out_(gpio_out) {
          set_param(param);
          if (param_.vbus_min == 0) {
            param_.vbus_min = 8;  // defaults that can be overridden via api
          }
          if (param_.vbus_max == 0) {
            param_.vbus_max = 60;
          }
        }
    void init() {}
    void update() {
      count_++;
      output_encoder_.trigger();
      
      torque_sensor_.trigger();
      
      last_timestamp_ = timestamp_;
      timestamp_ = get_clock();
      dt_ = (timestamp_ - last_timestamp_) * (1.0f/CPU_FREQUENCY_HZ);

      status_.fast_loop = fast_loop_.get_status();

      ReceiveData receive_data;
      int count_received = communication_.receive_data(&receive_data);
      bool command_received = false;
      if (started_) {
        if (count_received) {
          gpio_out_.set_value(receive_data.mode_misc.gpio);
          no_command_ = 0;
          first_command_received_ = true;
          if (!safe_mode_) {
            command_received = true;
            receive_data_ = receive_data;
          } else if (receive_data.mode_desired == CLEAR_FAULTS) {
              safe_mode_ = false;
              status_.error.all = 0;
              command_received = true;
              first_command_received_ = false;
              receive_data_ = receive_data;
          } else if (receive_data.mode_desired == BOARD_RESET ||
                     receive_data.mode_desired == CRASH ||
                     receive_data.mode_desired == SLEEP ||
                     receive_data.mode_desired == FAULT) {
              set_mode(static_cast<MainControlMode>(receive_data.mode_desired));
          }
        } else {
          no_command_++;
          if (no_command_ > 16000)
            no_command_ = 16000;
        }
      }
        
      if (param_.host_timeout && no_command_ > param_.host_timeout && started_ && first_command_received_) {
        status_.error.sequence = 1;
      }

      status_.error.bus_voltage_low |= status_.fast_loop.vbus < param_.vbus_min;
      status_.error.bus_voltage_high |= status_.fast_loop.vbus > param_.vbus_max;

      // internal command, not recommended in conjunction with host_timeout or safe mode
      if (started_ && internal_command_received_) {
        internal_command_received_ = false;
        if (!safe_mode_) {
          receive_data_ = internal_command_;
          command_received = true;
        }
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

      if ((status_.motor_position > param_.encoder_limits.motor_hard_max ||
          status_.motor_position < param_.encoder_limits.motor_hard_min) && started_) {
        status_.error.motor_encoder_limit = 1;
      }
      status_.error.motor_encoder |= fast_loop_.motor_encoder_error();
      
      if ((status_.output_position > param_.encoder_limits.output_hard_max ||
          status_.output_position < param_.encoder_limits.output_hard_min) && started_) {
          status_.error.output_encoder_limit = 1;
      }
      status_.error.output_encoder |= output_encoder_.error();

      if (!param_.disable_safe_mode && status_.error.all) {
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
        case STATE:
          iq_des = state_controller_.step(receive_data_, status_);
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

      if (((status_.motor_position > param_.encoder_limits.motor_controlled_max && iq_des >= 0) ||
          (status_.motor_position < param_.encoder_limits.motor_controlled_min && iq_des <= 0)) && started_) {
          if (mode_ != VELOCITY && mode_ != param_.safe_mode) {
            set_mode(VELOCITY);
          }
          MotorCommand tmp_receive_data = receive_data_;
          tmp_receive_data.velocity_desired = 0;
          iq_des = velocity_controller_.step(tmp_receive_data, status_);
      }

      fast_loop_.set_iq_des(iq_des);
      fast_loop_.set_vq_des(vq_des);
      
      SendData send_data;
      load_send_data(*this, &send_data);
      if (started_) { // this will prevent sending bad values before calibration
        communication_.send_data(send_data);
      }
      status_stack_.push(status_);
      led_.update();
      last_receive_data_ = receive_data_;
      IWDG->KR = 0xAAAA;
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
    void adjust_output_encoder(float adjustment) { param_.output_encoder.bias += adjustment; }
    void set_motor_encoder_bias(float bias) { motor_encoder_bias_ = bias; }
    const MainLoopStatus & get_status() const { return status_stack_.top(); }
    void set_started() { started_ = true; }
    void set_mode(MainControlMode mode) {
      if (mode != mode_ || safe_mode_ != last_safe_mode_) {
        if(mode_ == HARDWARE_BRAKE && mode != HARDWARE_BRAKE) {
          brake_.off();
        }
        switch (mode) {
          default:
            mode = OPEN;
          case OPEN:
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
          case STATE:
            fast_loop_.current_mode();
            state_controller_.init(status_);
            led_.set_color(LED::MAGENTA);
          case VOLTAGE:
            fast_loop_.voltage_mode();
            led_.set_color(LED::VIOLET);
            break;
          case PHASE_LOCK:
            fast_loop_.phase_lock_mode(0);
            led_.set_color(LED::YELLOW);
            break;
          case HARDWARE_BRAKE:
            brake_.on();
            led_.set_color(LED::ORANGE);
            break;
          case STEPPER_VELOCITY:
          case STEPPER_TUNING:
            fast_loop_.stepper_mode();
            led_.set_color(LED::CYAN);
            break;
          case FAULT:
            status_.error.host_fault = 1;
            // the main loop will detect this and set safe mode
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
        if (safe_mode_) {
          led_.set_color(LED::RED);
          led_.set_rate(2);
        } else {
          led_.set_rate(1);
        }
      }
      mode_ = mode;
      last_safe_mode_ = safe_mode_;
      status_.mode = mode;
      receive_data_.mode_desired = mode; // todo: what is this for?
    }

    // use to set the command from another low priority source than communication, 
    // such as from the System or Actuator classes
    void set_command(const MotorCommand &command) {
      internal_command_ = command;
      internal_command_received_ = true;
    }
    bool is_started() const { return started_; }
    bool first_command_received() const { return first_command_received_; }
 private:
    LED* led() { return &led_; }
    MainLoopParam param_;
    FastLoop &fast_loop_;
    PositionController &position_controller_;
    TorqueController &torque_controller_;
    ImpedanceController &impedance_controller_;
    VelocityController &velocity_controller_;
    StateController &state_controller_;
    Communication &communication_;
    LED &led_;
    ReceiveData receive_data_ = {};
    ReceiveData last_receive_data_ = {};
    MotorCommand internal_command_;
    bool internal_command_received_ = false;
    uint64_t count_ = 0;
    uint16_t no_command_ = 0;
    bool safe_mode_ = false;
    bool last_safe_mode_ = false;
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
    uint32_t *reserved0_ = reinterpret_cast<uint32_t *>(&status_.fast_loop.vbus);
    uint32_t *reserved1_ = &timestamp_;
    uint32_t *reserved2_ = &last_timestamp_;
    PChipTable<OUTPUT_ENCODER_TABLE_LENGTH> output_encoder_correction_table_;
    CStack<MainLoopStatus,2> status_stack_;
    bool first_command_received_ = false;
    HardwareBrake brake_;
    static HardwareBrakeBase no_brake_;

    GPIO &gpio_in_, &gpio_out_;
    static GPIO no_gpio;


    friend class System;
    friend void system_init();
    friend void system_maintenance();
    friend void config_init();
    friend void config_maintenance();
    friend void load_send_data(const MainLoop &main_loop, SendData *const data);
};

#ifndef CUSTOM_SENDDATA
void load_send_data(const MainLoop &main_loop, SendData * const data) {
    data->iq = main_loop.status_.fast_loop.foc_status.measured.i_q;
    data->host_timestamp_received = main_loop.receive_data_.host_timestamp;
    data->mcu_timestamp = main_loop.status_.fast_loop.timestamp;
    data->motor_encoder = main_loop.status_.fast_loop.motor_position.raw;
    data->motor_position = main_loop.status_.motor_position;
    data->joint_position = main_loop.status_.output_position;
    data->torque = main_loop.status_.torque;
    data->reserved[0] = *reinterpret_cast<float *>(main_loop.reserved0_);
    data->reserved[1] = *reinterpret_cast<float *>(main_loop.reserved1_);
    data->reserved[2] = *reinterpret_cast<float *>(main_loop.reserved2_);
    data->flags.mode = main_loop.status_.mode;
    data->flags.error = main_loop.status_.error;
    data->flags.misc = main_loop.gpio_in_.get_value();
}
#endif