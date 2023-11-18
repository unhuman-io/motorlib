#ifndef UNHUMAN_MOTORLIB_MAIN_LOOP_H_
#define UNHUMAN_MOTORLIB_MAIN_LOOP_H_

#include "messages.h"

#include <cmath>
#include "control_fun.h"
#include "sincos.h"
#include "led.h"
#include "util.h"
#include "torque_sensor.h"
#include "hardware_brake.h"
#include "round_robin_logger.h"
#include "temperature_model.h"

static const std::string error_bit_strings[32] = ERROR_BIT_STRINGS;

extern "C" {
void system_init();
}
extern volatile uint32_t uptime;

void setup_sleep();
void finish_sleep();

class MainLoop;
void load_send_data(const MainLoop &main_loop, SendData * const data);

#ifndef HARDWARE_BRAKE
using HardwareBrake = HardwareBrakeBase;
#endif  // HARDWARE_BRAKE

class MainLoop {
 public:
    MainLoop(int32_t frequency_hz, FastLoop &fast_loop, PositionController &position_controller,  TorqueController &torque_controller, 
        ImpedanceController &impedance_controller, VelocityController &velocity_controller, StateController &state_controller, 
        JointPositionController &joint_position_controller, AdmittanceController &admittance_controller, Communication &communication,
        LED &led, OutputEncoder &output_encoder, TorqueSensor &torque, Driver &driver, const MainLoopParam &param,
        HardwareBrake &brake=no_brake_) : 
          param_(param), fast_loop_(fast_loop), position_controller_(position_controller), torque_controller_(torque_controller), 
          impedance_controller_(impedance_controller), velocity_controller_(velocity_controller), state_controller_(state_controller),  
          joint_position_controller_(joint_position_controller), admittance_controller_(admittance_controller), 
          communication_(communication), led_(led), frequency_hz_(frequency_hz), output_encoder_(output_encoder), torque_sensor_(torque),
          output_encoder_correction_table_(param_.output_encoder.table), 
          torque_correction_table_(param_.torque_sensor.table), driver_(driver), brake_(brake),
          iq_find_limits_filter_(1.0/frequency_hz, 1), motor_velocity_filter_(1.0/frequency_hz, param.output_filter_hz.motor_velocity), motor_position_filter_(1.0/frequency_hz),
          output_position_filter_(1.0/frequency_hz), output_velocity_filter_(1.0/frequency_hz, param.output_filter_hz.output_velocity), torque_filter_(1.0/frequency_hz) {
          set_param();
        }
    void init() {} // todo: init filters with first status
    void update() {
      count_++;

      output_encoder_.trigger();
      torque_sensor_.trigger();

      if (count_ >= frequency_hz_) {
        count_ = 0;
        uptime++;
      }
      
      last_timestamp_ = timestamp_;
      timestamp_ = get_clock();
      dt_ = (timestamp_ - last_timestamp_) * (1.0f/CPU_FREQUENCY_HZ);

      status_.fast_loop = fast_loop_.get_status();
      if (!driver_.is_enabled()) {
        status_.fast_loop.energy_uJ = 0;
        status_.fast_loop.foc_status.measured.i_d = 0;
        status_.fast_loop.foc_status.measured.i_q = 0;
        status_.fast_loop.iq_filtered = 0;
      }

      ReceiveData receive_data;
      int count_received = communication_.receive_data(&receive_data);
      bool command_received = false;
      if (started_) {
        if (count_received) {
#ifdef GPIO_OUT
          GPIO_OUT = receive_data.misc.gpio;
#endif  // GPIO_OUT
          no_command_ = 0;
          first_command_received_ = true;
          host_timestamp_ = receive_data.host_timestamp;
          if (!safe_mode_) {
            command_received = true;
            receive_data_ = receive_data;
          } else if (receive_data.mode_desired == CLEAR_FAULTS ||
                     receive_data.mode_desired == DRIVER_ENABLE) {
              command_received = true;
              first_command_received_ = false;
              receive_data_ = receive_data;
          } else if (receive_data.mode_desired == BOARD_RESET ||
                     receive_data.mode_desired == CRASH ||
                     receive_data.mode_desired == SLEEP ||
                     receive_data.mode_desired == FAULT ||
                     receive_data.mode_desired == DRIVER_DISABLE) {
              set_mode(static_cast<MainControlMode>(receive_data.mode_desired));
          }
        } else {
          no_command_++;
          if (no_command_ > 16000)
            no_command_ = 16000;
        }
      }
        
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

      if (param_.host_timeout && no_command_ > param_.host_timeout && started_ && first_command_received_) {
        status_.error.sequence = 1;
      }

      status_.error.bus_voltage_low |= status_.fast_loop.vbus < vbus_min_;
      status_.error.bus_voltage_high |= status_.fast_loop.vbus > vbus_max_;

      int32_t output_encoder_raw = output_encoder_.read();
      float output_encoder_x = (output_encoder_raw % (int32_t) param_.output_encoder.cpr) / (float) param_.output_encoder.cpr;
      float output_encoder_rad = output_encoder_raw*2.0*(float) M_PI/param_.output_encoder.cpr;
      status_.output_position = output_encoder_dir_ * output_encoder_rad + output_encoder_bias_ 
        + output_encoder_correction_table_.table_interp(output_encoder_x);

      status_.motor_position = status_.fast_loop.motor_position.position_filtered + motor_encoder_bias_;

      float torque_corrected = torque_sensor_dir_ * (torque_sensor_.read() - param_.torque_sensor.bias) + param_.torque_sensor.bias;
      //if (torque_corrected != status_.torque) {
        torque_corrected += param_.torque_correction*status_.fast_loop.foc_status.measured.i_q;
      //}
      float torque_calibrated = torque_corrected + param_.torque_sensor.table_gain*torque_correction_table_.table_interp(output_encoder_x+output_encoder_bias_*(1.0/(2*M_PI)));
      status_.torque = torque_calibrated;

      if (!position_limits_disable_) {
        if ((status_.motor_position > encoder_limits_.motor_hard_max ||
            status_.motor_position < encoder_limits_.motor_hard_min) && started_) {
          status_.error.motor_encoder_limit = 1;
        }
        status_.error.motor_encoder |= fast_loop_.motor_encoder_error();
        
        if ((status_.output_position > encoder_limits_.output_hard_max ||
            status_.output_position < encoder_limits_.output_hard_min) && started_) {
            status_.error.output_encoder_limit = 1;
        }
        status_.error.output_encoder |= output_encoder_.error();
      }
      status_.error.driver_not_enabled |= !driver_.is_enabled();
      status_.error.driver_fault |= driver_.is_faulted();

      float output_velocity = (status_.output_position - output_position_last_)/dt_;
      output_position_last_ = status_.output_position;
      iq_find_limits_filter_.update(status_.fast_loop.foc_status.measured.i_q);
      status_.motor_velocity_filtered = motor_velocity_filter_.update(status_.motor_position);//(status_.fast_loop.motor_velocity.velocity_filtered);
      status_.motor_position_filtered = motor_position_filter_.update(status_.motor_position);
      status_.output_position_filtered = output_position_filter_.update(status_.output_position);
      status_.output_velocity_filtered = output_velocity_filter_.update(status_.output_position);//(output_velocity);
      status_.torque_filtered = torque_filter_.update(status_.torque);

      status_.motor_temperature_estimate = motor_temperature_model_.step(status_.fast_loop.foc_status.measured.i_d, status_.fast_loop.foc_status.measured.i_q);
      round_robin_logger.log_data(MOTOR_TEMPERATURE_ESTIMATE_INDEX, status_.motor_temperature_estimate);

      if (status_.motor_temperature_estimate > motor_temperature_limit_) {
         status_.error.motor_temperature = 1;
      }

      if (status_.error.all & error_mask_.all && !(receive_data_.mode_desired == DRIVER_ENABLE || receive_data_.mode_desired == CLEAR_FAULTS)) {
          status_.error.fault = 1;
          if (safe_mode_ != true) {
            logger.log_printf("fault detected, error: %08x", status_.error.all);
            std::string s = "fault bits:";
            for (int i=0; i<32; i++) {
              if ((status_.error.all >> i) & 0x1) {
                s += " " + error_bit_strings[i];
              }
            }
            logger.log(s);
          }
          safe_mode_ = true;
          set_mode(param_.safe_mode);
      }

      if (receive_data_.mode_desired == TUNING) {
          command_current_ = set_tuning_command(receive_data_, count_received);
          float desired = *tuning_trajectory_generator_.value();
          dft_desired_.step(desired, tuning_trajectory_generator_.get_frequency(), timestamp_);
          float measured = 0;
          switch (command_current_.mode_desired) {
            case POSITION:
              measured = status_.motor_position;
              break;
            case VELOCITY:
              measured = status_.motor_velocity_filtered;
              break;
            case TORQUE:
              measured = status_.torque;
              break;
          }
          dft_measured_.step(measured, tuning_trajectory_generator_.get_frequency(), timestamp_);
      } else {
          command_current_ = receive_data_;
      }

      float iq_des = 0;
      float vq_des = 0;
      switch (mode_) {
        case CURRENT:
          iq_des = command_current_.current_desired;
          break;
        case POSITION:
          iq_des = position_controller_.step(command_current_, status_);
          if (position_controller_.tracking_fault()) {
            status_.error.controller_tracking = true;
          }
          break;
        case TORQUE:
          iq_des = torque_controller_.step(command_current_, status_);
          break;
        case IMPEDANCE:
          iq_des = impedance_controller_.step(command_current_, status_);
          break;
        case VELOCITY:
          iq_des = velocity_controller_.step(command_current_, status_);
          break;
        case STATE:
          iq_des = state_controller_.step(command_current_, status_);
          break;
        case JOINT_POSITION:
          iq_des = joint_position_controller_.step(command_current_, status_);
          break;
        case ADMITTANCE:
          iq_des = admittance_controller_.step(command_current_, status_);
          break;
        case STEPPER_VELOCITY:
          vq_des = command_current_.stepper_velocity.voltage;
          iq_des = command_current_.stepper_velocity.current;
          fast_loop_.set_stepper_velocity(command_current_.stepper_velocity.velocity);
          break;
        case STEPPER_TUNING:
          {
            if (count_received) {
              position_trajectory_generator_.set_amplitude(command_current_.stepper_tuning.amplitude);
              position_trajectory_generator_.set_frequency(command_current_.stepper_tuning.frequency);
              position_trajectory_generator_.set_mode((TuningMode) command_current_.stepper_tuning.mode);
            }
            TrajectoryGenerator::TrajectoryValue traj = position_trajectory_generator_.step(dt_);
            fast_loop_.set_stepper_position(traj.value);
            fast_loop_.set_stepper_velocity(traj.value_dot);
            vq_des = command_current_.stepper_tuning.kv*traj.value_dot;
            break;
          }
        case POSITION_TUNING: 
          {
            if (count_received) {
              position_trajectory_generator_.set_amplitude(command_current_.position_tuning.amplitude);
              position_trajectory_generator_.set_frequency(command_current_.position_tuning.frequency);
              position_trajectory_generator_.set_mode((TuningMode) command_current_.position_tuning.mode);
            }
            TrajectoryGenerator::TrajectoryValue traj = position_trajectory_generator_.step(dt_);
            ReceiveData trajectory = {};
            float position_desired = traj.value;
            float velocity_desired = traj.value_dot;
            trajectory.position_desired = position_desired+command_current_.position_tuning.bias;
            trajectory.velocity_desired = velocity_desired;
            iq_des = position_controller_.step(trajectory, status_);
            if (position_controller_.tracking_fault()) {
              status_.error.controller_tracking = true;
            }

            break;
          }
        case CURRENT_TUNING:
          if (count_received) {
            fast_loop_.set_tuning_amplitude(command_current_.current_tuning.amplitude);
            fast_loop_.set_tuning_frequency(command_current_.current_tuning.frequency);
            fast_loop_.set_tuning_bias(command_current_.current_tuning.bias);
            fast_loop_.set_tuning_square(command_current_.current_tuning.mode == TuningMode::SQUARE);
            if (command_current_.current_tuning.mode == TuningMode::CHIRP) { // flag for chirp mode
              fast_loop_.set_tuning_chirp(true, command_current_.current_tuning.frequency);
            } else {
              fast_loop_.set_tuning_chirp(false, 0);
            }
          }
          // every cycle
          if (fast_log_ready_) {
            if (current_tuning_rate_limiter_.ready()) {
              if (status_stack_.top().fast_loop.foc_command.desired.i_q < 0 &&
                  status_.fast_loop.foc_command.desired.i_q > 0) {
                fast_loop_.trigger_status_log();
                current_tuning_rate_limiter_.run();
              }
            }
          }
          dft_desired_.step(status_.fast_loop.foc_command.desired.i_q, fast_loop_.get_tuning_frequency(), status_.fast_loop.timestamp);
          dft_measured_.step(status_.fast_loop.foc_status.measured.i_q, fast_loop_.get_tuning_frequency(), status_.fast_loop.timestamp);
          break;
        case VOLTAGE:
          vq_des = command_current_.voltage.voltage_desired;
          break;
        case PHASE_LOCK:
          fast_loop_.set_id_des(command_current_.current_desired);
          break;
        case FIND_LIMITS:
        {
          ReceiveData command = {};
          switch (find_limits_state_) {
            case FIND_FIRST_LIMIT:
              command.velocity_desired = command_current_.velocity_desired;
              if (iq_find_limits_filter_.get_value() > command_current_.current_desired) {
                find_limits_state_ = FIND_SECOND_LIMIT;
                // record positive limit
                //motor_positive_limit_ = status_.motor_position;
                //output_positive_limit_ = status_.output_position;
              }
              iq_des = velocity_controller_.step(command, status_);
              break;
            case FIND_SECOND_LIMIT:
              command.velocity_desired = -command_current_.velocity_desired;
              if (iq_find_limits_filter_.get_value() < -command_current_.current_desired) {
                find_limits_state_ = VELOCITY_TO_POSITION;
                // record negative limit
                // change encoder biases around
                adjust_output_encoder(-(status_.output_position - encoder_limits_.output_hard_min));
                adjust_motor_encoder(-(status_.motor_position - encoder_limits_.motor_hard_min));
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
                position_controller_.init(status_);
              }
              iq_des = velocity_controller_.step(command, status_);
              break;
            case GOTO_POSITION:
              position_limits_disable_ = false;
              command.position_desired = command_current_.position_desired;
              iq_des = position_controller_.step(command, status_);
              break;
          }
          
          break;
        }
        default:
          break;
      }

      if (!position_limits_disable_) {
        if (((status_.motor_position > encoder_limits_.motor_controlled_max && iq_des >= 0) ||
            (status_.motor_position < encoder_limits_.motor_controlled_min && iq_des <= 0)) && started_) {
          if (receive_data_.mode_desired != DRIVER_ENABLE && receive_data_.mode_desired != CLEAR_FAULTS) {
            if (mode_ != VELOCITY && mode_ != param_.safe_mode && first_command_received()) {
              set_mode(VELOCITY);
            }
            MotorCommand tmp_receive_data = command_current_;
            tmp_receive_data.velocity_desired = 0;
            iq_des = velocity_controller_.step(tmp_receive_data, status_);
            status_.error.motor_soft_limit = 1;
          }
        } else {
          if (status_.motor_position < encoder_limits_.motor_controlled_max &&
              status_.motor_position > encoder_limits_.motor_controlled_min) {
            status_.error.motor_soft_limit = 0;
          }
        }
      }

      fast_loop_.set_iq_des(iq_des);
      fast_loop_.set_vq_des(vq_des);
      
      if (communication_.tx_data_ack()) {
        round_robin_logger.get_next_data(&status_.rr_data);
      }

      uint32_t current_energy = status_.fast_loop.energy_uJ;
      status_.power = (int32_t) (current_energy - last_energy_uJ_)*1e-6/dt_;
      last_energy_uJ_ = current_energy;

      SendData send_data;
      load_send_data(*this, &send_data);
      if (started_) { // this will prevent sending bad values before calibration
        communication_.send_data(send_data);
      }
      status_stack_.push(status_);
      led_.update();
      //last_receive_data_ = receive_data_;
      IWDG->KR = 0xAAAA;
    }


    void set_param() {
      position_controller_.set_param(param_.position_controller_param);
      torque_controller_.set_param(param_.torque_controller_param);
      impedance_controller_.set_param(param_.impedance_controller_param);
      velocity_controller_.set_param(param_.velocity_controller_param);
      state_controller_.set_param(param_.state_controller_param);
      joint_position_controller_.set_param(param_.joint_position_controller_param);
      admittance_controller_.set_param(param_.admittance_controller_param);
      torque_sensor_.set_param(param_.torque_sensor);
      if (param_.encoder_limits.motor_hard_max == param_.encoder_limits.motor_hard_min) {
        encoder_limits_.motor_hard_max = INFINITY;
        encoder_limits_.motor_hard_min = -INFINITY;
      } else {
        encoder_limits_.motor_hard_max = param_.encoder_limits.motor_hard_max;
        encoder_limits_.motor_hard_min = param_.encoder_limits.motor_hard_min;
      }
      if (param_.encoder_limits.motor_controlled_max == param_.encoder_limits.motor_controlled_min) {
        encoder_limits_.motor_controlled_max = INFINITY;
        encoder_limits_.motor_controlled_min = -INFINITY;
      } else {
        encoder_limits_.motor_controlled_max = param_.encoder_limits.motor_controlled_max;
        encoder_limits_.motor_controlled_min = param_.encoder_limits.motor_controlled_min;
      }
      if (param_.encoder_limits.output_hard_max == param_.encoder_limits.output_hard_min) {
        encoder_limits_.output_hard_max = INFINITY;
        encoder_limits_.output_hard_min = -INFINITY;
      } else {
        encoder_limits_.output_hard_max = param_.encoder_limits.output_hard_max;
        encoder_limits_.output_hard_min = param_.encoder_limits.output_hard_min;
      }
      if (param_.motor_temperature_model.resistance != 0 && param_.motor_temperature_model.Cp != 0 &&
        param_.motor_temperature_model.Rth !=0) {
        motor_temperature_model_.set_param(param_.motor_temperature_model.resistance, param_.motor_temperature_model.Cp, 
          param_.motor_temperature_model.Rth, 1.0/frequency_hz_);
      }
      motor_temperature_limit_ = param_.motor_temperature_limit == 0 ? 140 : param_.motor_temperature_limit;
      output_encoder_bias_ = param_.output_encoder.bias;
      error_mask_.all = param_.error_mask.all == 0 ? ERROR_MASK_ALL : (param_.error_mask.all & ERROR_MASK_ALL);
      output_encoder_dir_ = param_.output_encoder.dir == 0 ? 1 : param_.output_encoder.dir;
      torque_sensor_dir_ = param_.torque_sensor.dir == 0 ? 1 : param_.torque_sensor.dir;
      vbus_min_ = param_.vbus_min == 0 ? 8 : param_.vbus_min;
      vbus_max_ = param_.vbus_max == 0 ? 58 : param_.vbus_max;

      //motor_velocity_filter_.set_frequency(param_.output_filter_hz.motor_velocity);
      motor_position_filter_.set_frequency(param_.output_filter_hz.motor_position);
      output_position_filter_.set_frequency(param_.output_filter_hz.output_position);
      //output_velocity_filter_.set_frequency(param_.output_filter_hz.output_velocity);
      torque_filter_.set_frequency(param_.output_filter_hz.torque);
    }
    void set_rollover(float rollover) {
      position_controller_.set_rollover(rollover);
      impedance_controller_.set_rollover(rollover);
      velocity_controller_.set_rollover(rollover);
    }
    void adjust_output_encoder(float adjustment) { output_encoder_bias_ += adjustment; }
    void set_motor_encoder_bias(float bias) { motor_encoder_bias_ = bias; }
    void adjust_motor_encoder(float adjustment) { motor_encoder_bias_ += adjustment; }
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
            position_trajectory_generator_.init();
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
            break;
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
          case JOINT_POSITION:
            fast_loop_.current_mode();
            joint_position_controller_.init(status_);
            led_.set_color(LED::BLUE);
            break;
          case ADMITTANCE:
            fast_loop_.current_mode();
            admittance_controller_.init(status_);
            led_.set_color(LED::ROSE);
            break;
          case TUNING:
            if (receive_data_.tuning_command.mode == POSITION ||
                receive_data_.tuning_command.mode == VELOCITY ||
                receive_data_.tuning_command.mode == TORQUE) {
              tuning_trajectory_generator_.init();
              reserved0_ = tuning_trajectory_generator_.value();
              set_mode(static_cast<MainControlMode>(receive_data_.tuning_command.mode));
              mode = static_cast<MainControlMode>(receive_data_.tuning_command.mode);
            }
            break;
          case FIND_LIMITS:
            fast_loop_.current_mode();
            position_limits_disable_ = true;
            find_limits_state_ = FIND_FIRST_LIMIT;
            velocity_controller_.init(status_);
            led_.set_color(LED::BLUE);
            break;
          case DRIVER_ENABLE:
            fast_loop_.open_mode();
            driver_enable_triggered_ = true;
            led_.set_color(LED::AZURE);
            break;
          case DRIVER_DISABLE:
            fast_loop_.open_mode();
            driver_disable_triggered_ = true;
            led_.set_color(LED::WHITE);
            break;
          case CLEAR_FAULTS:
            safe_mode_ = false;
            torque_sensor_.clear_faults();
            fast_loop_.clear_faults();
            output_encoder_.clear_faults();
            status_.error.all = 0;
            led_.set_color(LED::AZURE);
            break;
          case STEPPER_VELOCITY:
            fast_loop_.stepper_mode(static_cast<StepperMode>(receive_data_.stepper_velocity.stepper_mode));
            led_.set_color(LED::CYAN);
            break;
          case STEPPER_TUNING:
            fast_loop_.stepper_mode(static_cast<StepperMode>(receive_data_.stepper_tuning.stepper_mode));
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
            while(!communication_.any_new_rx_data()) {
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
          fast_loop_.trigger_status_log();
          led_.set_color(LED::RED);
          led_.set_rate(2);
          if (param_.safe_mode_driver_disable && !(mode == DRIVER_ENABLE)) {
            logger.log_printf("safe mode driver disable, mode: %d", mode);
            driver_.disable();
          }
        } else {
          led_.set_rate(1);
        }
      }
      mode_ = mode;
      last_safe_mode_ = safe_mode_;
      status_.mode = mode;
      //receive_data_.mode_desired = mode; // todo: what is this for?
    }

    MotorCommand set_tuning_command(ReceiveData &receive_data, bool update_parameters) {
      MotorCommand command = {};
      if (update_parameters) {
        tuning_trajectory_generator_.set_amplitude(receive_data.tuning_command.amplitude);
        tuning_trajectory_generator_.set_frequency(receive_data.tuning_command.frequency);
        tuning_trajectory_generator_.set_mode(static_cast<TuningMode>(receive_data.tuning_command.tuning_mode));
      }
      TrajectoryGenerator::TrajectoryValue traj = tuning_trajectory_generator_.step(dt_);
      switch (receive_data.tuning_command.mode) {
        case POSITION:
          command.position_desired = traj.value + receive_data.tuning_command.bias;
          command.velocity_desired = traj.value_dot;
          command.mode_desired = POSITION;
          break;
        case VELOCITY:
          command.velocity_desired = traj.value + receive_data.tuning_command.bias;
          command.mode_desired = VELOCITY;
          break;
        case TORQUE:
          command.torque_desired = traj.value + receive_data.tuning_command.bias;
          command.torque_dot_desired = traj.value_dot;
          command.mode_desired = TORQUE;
          break;
        default:
          break;
      }
      return command;
    }

    bool driver_enable_triggered() {
      if (driver_enable_triggered_) {
        driver_enable_triggered_ = false;
        return true;
      }
      return false;
    }

    bool driver_disable_triggered() {
      if (driver_disable_triggered_) {
        driver_disable_triggered_ = false;
        return true;
      }
      return false;
    }
    void lock_status_log() {
      fast_log_ready_ = false;
    }
    void unlock_status_log() {
      fast_log_ready_ = true;
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
    const MainLoopParam &param_;
    FastLoop &fast_loop_;
    PositionController &position_controller_;
    TorqueController &torque_controller_;
    ImpedanceController &impedance_controller_;
    VelocityController &velocity_controller_;
    StateController &state_controller_;
    JointPositionController &joint_position_controller_;
    AdmittanceController &admittance_controller_;
    Communication &communication_;
    MainLoopParam::EncoderLimits encoder_limits_;
    MotorError error_mask_;
    float vbus_min_, vbus_max_;
    float torque_sensor_dir_;
    float output_encoder_dir_;
    LED &led_;
    ReceiveData receive_data_ = {};
    MotorCommand command_current_ = {};
    mcu_time host_timestamp_ = {};
    ReceiveData last_receive_data_ = {};
    MotorCommand internal_command_;
    bool internal_command_received_ = false;
    uint32_t frequency_hz_;
    uint32_t count_ = 0;
    uint16_t no_command_ = 0;
    bool safe_mode_ = false;
    bool last_safe_mode_ = false;
    bool started_ = false;
    MainLoopStatus status_ = {};
    MainControlMode mode_ = NO_MODE;
    OutputEncoder &output_encoder_;
    float motor_encoder_bias_ = 0;
    float output_encoder_bias_ = 0;
    TorqueSensor &torque_sensor_;
    FrequencyLimiter current_tuning_rate_limiter_ = {10};
    bool fast_log_ready_ = true;
    float dt_ = 0;
    TrajectoryGenerator position_trajectory_generator_;
    TrajectoryGenerator tuning_trajectory_generator_;
    uint32_t timestamp_ = 0;
    uint32_t last_timestamp_ = 0;
    float *reserved0_ = &status_.fast_loop.vbus;
    PChipTable<OUTPUT_ENCODER_TABLE_LENGTH> output_encoder_correction_table_;
    PChipTable<TORQUE_TABLE_LENGTH> torque_correction_table_;
    CStack<MainLoopStatus,2> status_stack_;
    bool first_command_received_ = false;
    Driver &driver_;
    HardwareBrake brake_;
    static HardwareBrakeBase no_brake_;
    volatile bool driver_enable_triggered_ = false;
    volatile bool driver_disable_triggered_ = false;
    uint32_t last_energy_uJ_ = 0;

    DFT dft_desired_, dft_measured_;

    enum FindLimitsState {FIND_FIRST_LIMIT, FIND_SECOND_LIMIT, VELOCITY_TO_POSITION, GOTO_POSITION} find_limits_state_;
    FirstOrderLowPassFilter iq_find_limits_filter_;
    bool position_limits_disable_ = false;

    float output_position_last_ = 0;
    FIRFilter<> motor_velocity_filter_;
    FirstOrderLowPassFilter motor_position_filter_;
    FirstOrderLowPassFilter output_position_filter_;
    FIRFilter<> output_velocity_filter_;
    FirstOrderLowPassFilter torque_filter_;

    TemperatureModel motor_temperature_model_;
    float motor_temperature_limit_;

    friend class System;
    friend class Actuator;
    friend void system_init();
    friend void system_maintenance();
    friend void config_init();
    friend void config_maintenance();
    friend void load_send_data(const MainLoop &main_loop, SendData *const data);
};

#ifndef CUSTOM_SENDDATA
void load_send_data(const MainLoop &main_loop, SendData * const data) {
    data->iq = main_loop.status_.fast_loop.iq_filtered;
    data->host_timestamp_received = main_loop.host_timestamp_;
    data->mcu_timestamp = main_loop.status_.fast_loop.timestamp;
    data->motor_encoder = main_loop.status_.fast_loop.motor_position.raw;
    data->motor_position = main_loop.status_.motor_position_filtered;
    data->joint_position = main_loop.status_.output_position_filtered;
    data->motor_velocity = main_loop.status_.motor_velocity_filtered;
    data->joint_velocity = main_loop.status_.output_velocity_filtered;
    data->torque = main_loop.status_.torque_filtered;
    data->rr_data = main_loop.status_.rr_data;
    data->reserved = *main_loop.reserved0_;
    data->iq_desired = main_loop.status_.fast_loop.foc_status.command.i_q;
    data->flags.mode = main_loop.status_.mode;
    data->flags.error = main_loop.status_.error;
    data->flags.misc.byte = 0;
#ifdef GPIO_IN
    data->flags.misc.gpio = GPIO_IN;
#endif  // GPIO_IN
}
#endif  // CUSTOM_SENDDATA

#endif  // UNHUMAN_MOTORLIB_MAIN_LOOP_H_
