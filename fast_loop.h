#ifndef FAST_LOOP_H
#define FAST_LOOP_H

#include <cstdint>
#include "messages.h"
#include "control_fun.h"

// #include "fast_loop.h"
#include "foc.h"
// #include <cmath>
// #include "peripheral/pwm.h"
#include "util.h"
// #include "encoder.h"
// #include "../st_device.h"
#include "sincos.h"
#include "table_interp.h"

class FastLoop {
 public:
    FastLoop(int32_t frequency_hz, PWM &pwm, MotorEncoder &encoder, const FastLoopParam &param,
      volatile uint32_t *const i_a_dr, volatile uint32_t *const i_b_dr, volatile uint32_t *const i_c_dr, 
      volatile uint32_t *const v_bus_dr) 
      : param_(param), pwm_(pwm), encoder_(encoder), i_a_dr_(i_a_dr), i_b_dr_(i_b_dr), i_c_dr_(i_c_dr), v_bus_dr_(v_bus_dr),
        motor_correction_table_(param_.motor_encoder.table), cogging_correction_table_(param_.cogging.table) {
       frequency_hz_ = frequency_hz;
       float dt = 1.0f/frequency_hz;
       foc_ = new FOC(dt);
       set_param(param);
    }
    ~FastLoop() {
       delete foc_;
    }
    void update()  __attribute__((section (".ccmram"))) {
         // trigger encoder read
      encoder_.trigger();

      timestamp_ = get_clock();

      // get ADC
      adc1 = *i_a_dr_;
      adc2 = *i_b_dr_;
      adc3 = *i_c_dr_;
      foc_command_.measured.i_a = param_.adc1_gain*(adc1-param_.adc1_offset) - ia_bias_;
      foc_command_.measured.i_b = param_.adc2_gain*(adc2-param_.adc2_offset) - ib_bias_;
      foc_command_.measured.i_c = param_.adc3_gain*(adc3-param_.adc3_offset) - ic_bias_;
      
      // get encoder value, may wait a little
      motor_enc = encoder_.read();
      int32_t motor_enc_diff = motor_enc-last_motor_enc;
      motor_enc_wrap_ = wrap1(motor_enc_wrap_ + motor_enc_diff, param_.motor_encoder.rollover);
      //motor_enc_wrap_ = (motor_enc_wrap_ + motor_enc_diff + param_.motor_encoder.rollover ) % (param_.motor_encoder.rollover*2) - param_.motor_encoder.rollover ;
      motor_mechanical_position_ = (motor_enc_wrap_ - motor_index_pos_);
      float motor_x = motor_mechanical_position_*inv_motor_encoder_cpr_;

      motor_position_ = param_.motor_encoder.dir * (2 * (float) M_PI * inv_motor_encoder_cpr_ * motor_enc_wrap_ 
                          + motor_index_pos_set_*motor_correction_table_.table_interp(motor_x));
      float diff = wrap1_diff(motor_position_, last_motor_position_, 2 * (float) M_PI * inv_motor_encoder_cpr_ * param_.motor_encoder.rollover);
      //motor_position_filtered_ = last_motor_position_ + diff;//(1-alpha10)*motor_position_filtered_ + alpha10*motor_position_;
      motor_position_filtered_ = (1-alpha10)*motor_position_filtered_ + alpha10*(last_motor_position_ + diff);
      motor_position_filtered_ = wrap1(motor_position_filtered_,  2 * (float) M_PI * inv_motor_encoder_cpr_ * param_.motor_encoder.rollover);
      // motor_velocity =  param_.motor_encoder.dir * (motor_enc_diff)*(2*(float) M_PI * inv_motor_encoder_cpr_ * frequency_hz_);
      // motor_velocity_filtered = (1-alpha)*motor_velocity_filtered + alpha*motor_velocity;
      last_motor_enc = motor_enc;
      last_motor_position_ = motor_position_;

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
         iq_des = tuning_amplitude_ * (tuning_frequency_ > 0 ? sincos.sin : ((sincos.sin > 0) - (sincos.sin < 0)));
      }

      // update FOC
      foc_command_.measured.motor_encoder = phase_mode_*(motor_enc_wrap_ - motor_electrical_zero_pos_)*(2*(float) M_PI  * inv_motor_encoder_cpr_);
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
      t_seconds_.add(dt_);
    }
    void maintenance() {
      if (encoder_.index_received() && !motor_index_pos_set_) {
         motor_index_pos_ = encoder_.get_index_pos();
         if (param_.motor_encoder.use_index_electrical_offset_pos) {
            // motor_index_electrical_offset_pos is the value of an electrical zero minus the index position
            // motor_electrical_zero_pos is the offset to the initial encoder value
            motor_electrical_zero_pos_ = param_.motor_encoder.index_electrical_offset_pos + motor_index_pos_;
         }
         motor_index_pos_set_ = true;
      }

      if (mode_ == PHASE_LOCK_MODE) {
         motor_electrical_zero_pos_ = encoder_.get_value();
      }

      v_bus_ = *v_bus_dr_*param_.vbus_gain;
      pwm_.set_vbus(fmaxf(7, v_bus_));
    }
    void set_id_des(float id) { foc_command_.desired.i_d = id; }
    void set_iq_des(float iq) { if (mode_ == CURRENT_MODE) iq_des = iq; }
    void set_vq_des(float vq) { foc_command_.desired.v_q = vq; }
    void set_tuning_amplitude(float amplitude) { tuning_amplitude_ = amplitude; }
    void set_tuning_frequency(float frequency) { tuning_frequency_ = frequency; }
    void set_tuning_chirp(bool on, float chirp_rate) { 
      current_tuning_chirp_ = on; 
      chirp_rate_ = chirp_rate; 
      chirp_frequency_.init(0);
    }
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
      pwm_.voltage_mode();
      foc_->voltage_mode();
      mode_ = VOLTAGE_MODE;
    }
    void stepper_mode() {
      pwm_.voltage_mode();
      foc_->voltage_mode();
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
    void set_param(const FastLoopParam &fast_loop_param) {
      param_ = fast_loop_param;
      foc_->set_param(param_.foc_param);
      set_phase_mode();
      inv_motor_encoder_cpr_ = param_.motor_encoder.cpr != 0 ? 1.f/param_.motor_encoder.cpr : 0;
    }
    void get_status(FastLoopStatus *fast_loop_status) {
      foc_->get_status(&(fast_loop_status->foc_status));
      fast_loop_status->motor_mechanical_position = motor_mechanical_position_;
      fast_loop_status->foc_command = foc_command_;
      fast_loop_status->motor_position.position = motor_position_filtered_;
      fast_loop_status->motor_position.velocity = motor_velocity_filtered;
      fast_loop_status->motor_position.raw = motor_enc;
      fast_loop_status->timestamp = timestamp_;
      fast_loop_status->t_seconds = t_seconds_.value();
      fast_loop_status->dt = dt_;
      fast_loop_status->vbus = v_bus_;
    }
    void zero_current_sensors() {
      ia_bias_ = (1-alpha_zero_)*ia_bias_ + alpha_zero_* param_.adc1_gain*(adc1-param_.adc1_offset);
      ib_bias_ = (1-alpha_zero_)*ib_bias_ + alpha_zero_* param_.adc2_gain*(adc2-param_.adc2_offset);
      ic_bias_ = (1-alpha_zero_)*ic_bias_ + alpha_zero_* param_.adc3_gain*(adc3-param_.adc3_offset);
    }
    void set_phase_mode() {
      phase_mode_desired_ = param_.phase_mode == 0 ? 1 : -1;
    }
    float get_rollover() const { return 2*M_PI*inv_motor_encoder_cpr_*param_.motor_encoder.rollover; }
 private:
    FastLoopParam param_;
    FOC *foc_;
    PWM &pwm_;
    enum {OPEN_MODE, BRAKE_MODE, CURRENT_MODE, PHASE_LOCK_MODE, VOLTAGE_MODE, CURRENT_TUNING_MODE, STEPPER_TUNING_MODE} mode_ = CURRENT_MODE;

    int32_t motor_enc;
    int32_t last_motor_enc=0;
    float motor_position_ = 0;
    float last_motor_position_ = 0;
    float motor_position_filtered_ = 0;
    float motor_velocity=0;
    float motor_velocity_filtered=0;
    float alpha=0.001;
    float alpha10=1;//0.3859;   // 1/10 cutoff frequency
    float phase_mode_ = 1;    // 1: standard or -1: two wires switched
    float phase_mode_desired_ = 1;
    int32_t motor_mechanical_position_ = 0;

    float iq_des = 0;
    float id_des = 0;
    float iq_des_gain_ = 1;
    volatile uint16_t adc1, adc2, adc3;
    FOCCommand foc_command_ = {};

    int32_t motor_index_pos_ = 0;
    bool motor_index_pos_set_ = false;
    int32_t motor_electrical_zero_pos_;
    float inv_motor_encoder_cpr_;
    int32_t frequency_hz_ = 100000;
    volatile float ia_bias_ = 0;
    volatile float ib_bias_ = 0;
    volatile float ic_bias_ = 0;
    float alpha_zero_ = 0.001;
    float v_bus_ = 12;
    mcu_time timestamp_;
   MotorEncoder &encoder_;
   float reserved_ = 0;
   KahanSum t_seconds_;
   mcu_time last_timestamp_ = 0;
   float dt_ = 0;
   float phi_ = 0;
   float tuning_amplitude_ = 0;
   float tuning_frequency_ = 0;
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

   friend class System;
};

#endif
