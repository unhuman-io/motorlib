#ifndef UNHUMAN_MOTORLIB_FAST_LOOP_H_
#define UNHUMAN_MOTORLIB_FAST_LOOP_H_

#include <cstdint>
#include "messages.h"
#include "control_fun.h"
#include "logger.h"

#include "foc.h"
#include "util.h"
#include "sincos.h"
#include "table_interp.h"
#include "cstack.h"

extern "C" void system_init();

class FastLoop {
 public:
    FastLoop(int32_t frequency_hz, PWM &pwm, MotorEncoder &encoder, const FastLoopParam &param, const Calibration &calibration,
      volatile uint32_t *const i_a_dr, volatile uint32_t *const i_b_dr, volatile uint32_t *const i_c_dr, 
      volatile uint32_t *const v_bus_dr) 
      : param_(param), motor_encoder_index_electrical_offset_pos_(calibration.motor_encoder_index_electrical_offset_pos), pwm_(pwm), encoder_(encoder), i_a_dr_(i_a_dr), i_b_dr_(i_b_dr), i_c_dr_(i_c_dr), v_bus_dr_(v_bus_dr),
        motor_correction_table_(param_.motor_encoder.table), cogging_correction_table_(param_.cogging.table),
        iq_filter_(1.0/frequency_hz), motor_velocity_filter_(1.0/frequency_hz), motor_position_filter_(1.0/frequency_hz) {
       frequency_hz_ = frequency_hz;
       float dt = 1.0f/frequency_hz;
       foc_ = new FOC(dt);
       set_param();
       iq_filter_.init(0);
       motor_velocity_filter_.init(0);
       encoder_.trigger();
       int32_t motor_enc = encoder_.read();
       int32_t tmp;
       float tmp2;
       motor_position_filter_.init(motor_enc_to_position(motor_enc, tmp, tmp2));
#ifdef END_TRIGGER_MOTOR_ENCODER
       encoder_.trigger();
#endif
    }
    ~FastLoop() {
       delete foc_;
    }
    void update()  __attribute__((section (".ccmram"))) {
         // trigger encoder read
#ifndef END_TRIGGER_MOTOR_ENCODER
      // probably don't use end trigger on a shared spi bus
      encoder_.trigger();
#endif

      timestamp_ = get_clock();

      // get ADC
      adc1 = *i_a_dr_;
      adc2 = *i_b_dr_;
      adc3 = *i_c_dr_;
      foc_command_.measured.i_a = param_.adc1_gain*(adc1-2048) - ia_bias_;
      foc_command_.measured.i_b = param_.adc2_gain*(adc2-2048) - ib_bias_;
      foc_command_.measured.i_c = param_.adc3_gain*(adc3-2048) - ic_bias_;
      
      // get encoder value, may wait a little
      motor_enc = encoder_.read();
      int32_t motor_enc_diff;
      float motor_x;
      motor_position_ = motor_enc_to_position(motor_enc, motor_enc_diff, motor_x);
      motor_position_filtered_ = motor_position_filter_.update(motor_position_);//(1-alpha10)*motor_position_filtered_ + alpha10*motor_position_;
      motor_velocity_ =  motor_encoder_dir_ * (motor_enc_diff)*(2*(float) M_PI * inv_motor_encoder_cpr_ * frequency_hz_);
      motor_velocity_filtered_ = motor_velocity_filter_.update(motor_velocity_);
      

      // cogging compensation, interpolate in the table
      float iq_ff = param_.cogging.gain * cogging_correction_table_.table_interp(motor_x);

      if (mode_ == CURRENT_TUNING_MODE) {
         // only works down to frequencies of .047 Hz, could use kahansum to go slower
         if (current_tuning_chirp_) {
           tuning_frequency_ = chirp_frequency_.add(chirp_rate_ * dt_);
         }
         phi_ += 2 * (float) M_PI * fabsf(tuning_frequency_) * dt_;   // use id des to set frequency
         if (phi_ > 2 * (float) M_PI) {
         phi_ -= 2 * (float) M_PI;
         }
         Sincos sincos;
         sincos = sincos1(phi_);
         iq_des = tuning_bias_ + tuning_amplitude_ * (tuning_square_ ? fsignf(sincos.sin) : sincos.sin);
      }

      if (beep_) {
        if ((int32_t) (get_clock()-beep_end_) > 0) {
          beep_ = false;
        } else {
          phi_beep_ += 2 * (float) M_PI * fabsf(param_.beep_frequency) * dt_;
          if (phi_beep_ > 2 * (float) M_PI) {
            phi_beep_ -= 2 * (float) M_PI;
          }
          Sincos sincos = sincos1(phi_beep_);
          iq_ff += param_.beep_amplitude*fsignf(sincos.sin);
        }
      }

      // update FOC
      foc_command_.measured.motor_encoder = phase_mode_*(motor_enc_wrap_ - motor_electrical_zero_dir_pos_)*(2*(float) M_PI  * inv_motor_encoder_cpr_);
      foc_command_.desired.i_q = iq_des_gain_ * (iq_des + iq_ff);

      if (mode_ == STEPPER_TUNING_MODE) {
        foc_command_.measured.motor_encoder = stepper_position_;
        motor_position_filtered_ = stepper_position_;
        stepper_position_ += stepper_velocity_ * dt_;
        stepper_position_ = wrap1(stepper_position_, 2*M_PI);
      }
      
      FOCStatus *foc_status = foc_->step(foc_command_);

      // output pwm
      pwm_.set_voltage(&foc_status->command.v_a);

      dt_ = (timestamp_ - last_timestamp_)*(float) (1.0f/CPU_FREQUENCY_HZ);
      last_timestamp_ = timestamp_;

      if (zero_current_sensors_) {
        if ((int32_t) (get_clock()-zero_current_sensors_end_) > 0) {
          zero_current_sensors_ = false;
        } else {
          zero_current_sensors();
        }

      }
      store_status();
#ifdef END_TRIGGER_MOTOR_ENCODER
      encoder_.trigger();
#endif
    }

    float motor_enc_to_position(int32_t motor_enc, int32_t &motor_enc_diff, float &motor_x) {
      motor_enc_diff = motor_enc-last_motor_enc;
      motor_enc_wrap_ = wrap1(motor_enc_wrap_ + motor_enc_diff, param_.motor_encoder.rollover);
      motor_mechanical_position_ = (motor_enc_wrap_ - motor_index_pos_);
      motor_x = motor_mechanical_position_*inv_motor_encoder_cpr_;

      motor_position_ = motor_encoder_dir_ * (2 * (float) M_PI * inv_motor_encoder_cpr_ * motor_enc_wrap_ 
                          + motor_index_pos_set_*motor_correction_table_.table_interp(motor_x));
      last_motor_enc = motor_enc;
      return motor_position_;
    }
    void maintenance() {
      if (encoder_.index_received() && !motor_index_pos_set_) {
         motor_index_pos_ = encoder_.get_index_pos();
         if (param_.motor_encoder.use_index_electrical_offset_pos) {
            // motor_index_electrical_offset_pos is the value of an electrical zero minus the index position
            // motor_electrical_zero_pos is the offset to the initial encoder value
            // motor_electrical_zero_pos_ = param_.motor_encoder.index_electrical_offset_pos + motor_index_pos_;
            motor_electrical_zero_pos_ = motor_encoder_index_electrical_offset_pos_ + motor_index_pos_;
         }
         motor_index_pos_set_ = true;
      }           

      if (mode_ == PHASE_LOCK_MODE) {
         motor_electrical_zero_pos_ = encoder_.get_value();
         if (encoder_.index_received()) {
           motor_index_pos_ = encoder_.get_index_pos();
           int32_t index_offset = motor_electrical_zero_pos_ - motor_index_pos_;
           if (index_offset >= 0) {
            motor_index_electrical_offset_measured_ = index_offset % 
              (param_.motor_encoder.cpr/(uint8_t) param_.foc_param.num_poles);
           } else {
            int32_t m = (param_.motor_encoder.cpr/(uint8_t) param_.foc_param.num_poles);
            motor_index_electrical_offset_measured_ = index_offset -
               m * ((index_offset/m)-1);
           }
        }
      }

      motor_electrical_zero_dir_pos_ = motor_electrical_zero_pos_ + current_direction_*(param_.motor_encoder.cpr/(uint8_t) param_.foc_param.num_poles/2);

      v_bus_ = *v_bus_dr_*param_.vbus_gain;
      pwm_.set_vbus(fmaxf(7, v_bus_));
    }
    void set_id_des(float id) { foc_command_.desired.i_d = id; }
    void set_iq_des(float iq) { if (mode_ == CURRENT_MODE || mode_ == STEPPER_TUNING_MODE) iq_des = iq; }
    void set_vq_des(float vq) { foc_command_.desired.v_q = vq; }
    void set_tuning_amplitude(float amplitude) { tuning_amplitude_ = amplitude; }
    void set_tuning_frequency(float frequency) { tuning_frequency_ = frequency; }
    float get_tuning_frequency() const { return tuning_frequency_; }
    void set_tuning_chirp(bool on, float chirp_rate) { 
      current_tuning_chirp_ = on; 
      chirp_rate_ = chirp_rate; 
      chirp_frequency_.init(0);
    }
    void set_tuning_bias(float bias) { tuning_bias_ = bias; }
    void set_tuning_square(bool square = true) { tuning_square_ = square; }
    void set_stepper_position(float position) { stepper_position_ = position; }
    void set_stepper_velocity(float velocity) { stepper_velocity_ = velocity; }
    void set_reserved(float reserved) { reserved_ = reserved; }
    void phase_lock_mode(float id) {
      phase_mode_ = 0;
      set_id_des(id);
      iq_des_gain_ = 0;
      pwm_.voltage_mode();
      foc_->current_mode();
      mode_ = PHASE_LOCK_MODE;
    }
    void current_mode() {
      phase_mode_ = phase_mode_desired_;
      set_id_des(0);
      iq_des_gain_ = 1;
      pwm_.voltage_mode();
      foc_->current_mode();
      mode_ = CURRENT_MODE;
    }
    void current_tuning_mode() {
      current_mode();
      phi_ = 0;
      mode_ = CURRENT_TUNING_MODE;
    }
    void voltage_mode() {
      phase_mode_ = phase_mode_desired_;
      pwm_.voltage_mode();
      foc_->voltage_mode();
      mode_ = VOLTAGE_MODE;
    }
    void stepper_mode(StepperMode mode) {
      switch (mode) {
        case STEPPER_CURRENT:
        default:
          current_mode();
          break;
        case STEPPER_VOLTAGE:
          voltage_mode();
          break;
      }
      mode_ = STEPPER_TUNING_MODE;
    }
    void brake_mode() {
      pwm_.brake_mode();
      foc_->voltage_mode();
      mode_ = BRAKE_MODE;
    }
    void open_mode() {
      pwm_.open_mode();
      foc_->voltage_mode();
      mode_ = OPEN_MODE;
    }
    void set_param() {
      foc_->set_param(param_.foc_param);
      set_phase_mode();
      motor_encoder_dir_ = param_.motor_encoder.dir >= 0 ? 1 : -1;
      inv_motor_encoder_cpr_ = param_.motor_encoder.cpr != 0 ? 1.f/param_.motor_encoder.cpr : 0;
      ia_bias_ = param_.ia_bias;
      ib_bias_ = param_.ib_bias;
      ic_bias_ = param_.ic_bias;
      max_i_bias_ = param_.max_i_bias == 0 ? 10 : param_.max_i_bias;
      iq_filter_.set_frequency(param_.output_filter_hz.iq);
      motor_velocity_filter_.set_frequency(param_.output_filter_hz.motor_velocity);
      motor_position_filter_.set_frequency(param_.output_filter_hz.motor_position);
      current_direction_ = param_.current_direction;
    }
    const FastLoopStatus &get_status() const {
      return status_.top();
    }
    void store_status() {
      FastLoopStatus &s = status_.next();
      s.motor_mechanical_position = motor_mechanical_position_;
      s.motor_position.position = motor_position_;
      s.motor_position.position_filtered = motor_position_filtered_;
      s.motor_velocity.velocity_filtered = motor_velocity_filtered_;
      s.motor_position.raw = motor_enc;
      s.timestamp = timestamp_;
      s.vbus = v_bus_;
      s.foc_command = foc_command_;
      s.power = s.foc_status.command.v_d * s.foc_status.measured.i_d + 
                s.foc_status.command.v_q * s.foc_status.measured.i_q;
      int32_t energy =  s.power * dt_ * 1e6;
      energy_uJ_ += (uint32_t) energy;
      s.energy_uJ = energy_uJ_;
      foc_->get_status(&s.foc_status);
      s.iq_filtered = iq_filter_.update(s.foc_status.measured.i_q);
      status_.finish();
    }

    void zero_current_sensors() {
      ia_bias_ = (1-alpha_zero_)*ia_bias_ + alpha_zero_* param_.adc1_gain*(adc1-2048);
      ib_bias_ = (1-alpha_zero_)*ib_bias_ + alpha_zero_* param_.adc2_gain*(adc2-2048);
      ic_bias_ = (1-alpha_zero_)*ic_bias_ + alpha_zero_* param_.adc3_gain*(adc3-2048);
    }

    void zero_current_sensors(uint16_t adc1_0, uint16_t adc2_0, uint16_t adc3_0) {
      ia_bias_ = (1-alpha_zero_)*(ia_bias_) + alpha_zero_* param_.adc1_gain*(adc1_0-2048);
      ib_bias_ = (1-alpha_zero_)*(ib_bias_) + alpha_zero_* param_.adc2_gain*(adc2_0-2048);
      ic_bias_ = (1-alpha_zero_)*(ic_bias_) + alpha_zero_* param_.adc3_gain*(adc3_0-2048);
      ia_bias_ = fsat(ia_bias_, max_i_bias_);
      ib_bias_ = fsat(ib_bias_, max_i_bias_);
      ic_bias_ = fsat(ic_bias_, max_i_bias_);
    }

    void set_phase_mode(float phase_mode) {
      phase_mode_desired_ = phase_mode == 0 ? 1 : -1;
      logger.log_printf("phase mode desired: %f", phase_mode_desired_);
    }

    void set_phase_mode() {
      set_phase_mode(param_.phase_mode);
    }

    uint8_t get_phase_mode() {
      return phase_mode_desired_ == 1 ? 0 : 1;
    }

    float get_rollover() const { return 2*M_PI*inv_motor_encoder_cpr_*param_.motor_encoder.rollover; }
    void beep_on(float t_seconds = 1) {
      beep_ = true;
      beep_end_ = get_clock() + t_seconds*CPU_FREQUENCY_HZ;
    }
    void beep_off() {
      beep_ = false;
    }

    void zero_current_sensors_on(float t_seconds = 1) {
      zero_current_sensors_ = true;
      zero_current_sensors_end_ = get_clock() + t_seconds*CPU_FREQUENCY_HZ;
    }
    void zero_current_sensors_off() {
      zero_current_sensors_ = false;
    }
    bool motor_encoder_error() { return encoder_.error(); }
    void trigger_status_log() {
      status_log_.copy(status_);
    }
    void clear_faults() {
      encoder_.clear_faults();
    }
 private:
    FastLoopParam param_; // reallocate tables in ram
    float motor_encoder_index_electrical_offset_pos_;

    FOC *foc_;
    PWM &pwm_;
    enum {OPEN_MODE, BRAKE_MODE, CURRENT_MODE, PHASE_LOCK_MODE, VOLTAGE_MODE, CURRENT_TUNING_MODE, STEPPER_TUNING_MODE} mode_ = CURRENT_MODE;

    float motor_encoder_dir_;
    int32_t motor_enc;
    int32_t last_motor_enc=0;
    float motor_position_ = 0;
    float motor_position_filtered_ = 0;
    float motor_velocity_=0;
    float motor_velocity_filtered_=0;
    float alpha=0.001;
    float alpha10=1;//0.3859;   // 1/10 cutoff frequency
    float phase_mode_ = 1;    // 1: standard or -1: two wires switched
    float phase_mode_desired_ = 1;
    int32_t motor_mechanical_position_ = 0;

    float ia_bias_, ib_bias_, ic_bias_;
    float max_i_bias_ = 0;

    float iq_des = 0;
    float id_des = 0;
    float iq_des_gain_ = 1;
    volatile uint16_t adc1, adc2, adc3;
    FOCCommand foc_command_ = {};

    int32_t motor_index_pos_ = 0;
    bool motor_index_pos_set_ = false;
    int32_t motor_electrical_zero_pos_;
    volatile int32_t motor_electrical_zero_dir_pos_;
    float current_direction_ = 0;
    float motor_index_electrical_offset_measured_ = NAN;
    float inv_motor_encoder_cpr_;
    int32_t frequency_hz_ = 100000;
    float alpha_zero_ = 0.0002;
    float v_bus_ = 12;
    mcu_time timestamp_;
   MotorEncoder &encoder_;
   float reserved_ = 0;
   mcu_time last_timestamp_ = 0;
   float dt_ = 0;
   float phi_ = 0;
   float tuning_amplitude_ = 0;
   float tuning_frequency_ = 0;
   float tuning_bias_ = 0;
   bool tuning_square_ = false;
   float chirp_rate_ = 0;
   bool current_tuning_chirp_ = false;
   KahanSum chirp_frequency_;
   float stepper_position_ = 0;
   float stepper_velocity_ = 0;
   int32_t motor_enc_wrap_ = 0;
   volatile uint32_t *const i_a_dr_;
   volatile uint32_t *const i_b_dr_;
   volatile uint32_t *const i_c_dr_;
   volatile uint32_t *const v_bus_dr_;
   PChipTable<MOTOR_ENCODER_TABLE_LENGTH> motor_correction_table_;
   PChipTable<COGGING_TABLE_SIZE> cogging_correction_table_;
   CStack<FastLoopStatus,100> status_;
   CStack<FastLoopStatus,100> status_log_; // 24*4*100*2 = 19200 bytes
   bool beep_ = false;
   uint32_t beep_end_ = 0;
   bool zero_current_sensors_ = false;
   uint32_t zero_current_sensors_end_ = 0;
   float phi_beep_ = 0;
   uint32_t energy_uJ_ = 0;
  
   FirstOrderLowPassFilter iq_filter_;
   FirstOrderLowPassFilter motor_velocity_filter_;
   FirstOrderLowPassFilter motor_position_filter_;
   
   


   friend class System;
   friend void system_init();
};

#endif  // UNHUMAN_MOTORLIB_FAST_LOOP_H_
