#ifndef UNHUMAN_MOTORLIB_FAST_LOOP_H_
#define UNHUMAN_MOTORLIB_FAST_LOOP_H_

#include <cstdint>

#include "control_fun.h"
#include "cstack.h"
#include "foc.h"
#include "messages.h"
#include "system_types.h"
#include "table_interp.h"

class FastLoop {
 public:
  FastLoop(int32_t frequency_hz, PWMType &pwm, MotorEncoderType &motor_encoder,
           const FastLoopParam &param, volatile uint32_t *const i_a_dr,
           volatile uint32_t *const i_b_dr, volatile uint32_t *const i_c_dr,
           volatile uint32_t *const v_bus_dr)
      : param_(param),
        pwm_(pwm),
        motor_encoder_(motor_encoder),
        i_a_dr_(i_a_dr),
        i_b_dr_(i_b_dr),
        i_c_dr_(i_c_dr),
        v_bus_dr_(v_bus_dr),
        motor_correction_table_(param_.motor_encoder.table),
        cogging_correction_table_(param_.cogging.table) {
    frequency_hz_ = frequency_hz;
    float dt = 1.0f / frequency_hz;
    foc_ = new FOC(dt);
    set_param(param);
#ifdef END_TRIGGER_MOTOR_ENCODER
    motor_encoder_.trigger();
#endif
  }
  ~FastLoop() { delete foc_; }
  void update() __attribute__((section(".ccmram"))) {
    // trigger encoder read
#ifndef END_TRIGGER_MOTOR_ENCODER
    // probably don't use end trigger on a shared spi bus
    motor_encoder_.trigger();
#endif

    timestamp_ = get_clock();

    // get ADC
    adc1 = *i_a_dr_;
    adc2 = *i_b_dr_;
    adc3 = *i_c_dr_;
    foc_command_.measured.i_a =
        param_.adc1_gain * (adc1 - 2048) - param_.ia_bias;
    foc_command_.measured.i_b =
        param_.adc2_gain * (adc2 - 2048) - param_.ib_bias;
    foc_command_.measured.i_c =
        param_.adc3_gain * (adc3 - 2048) - param_.ic_bias;

    // get encoder value, may wait a little
    motor_enc = motor_encoder_.read();
    int32_t motor_enc_diff = motor_enc - last_motor_enc;
    motor_enc_wrap_ =
        wrap1(motor_enc_wrap_ + motor_enc_diff, param_.motor_encoder.rollover);
    motor_mechanical_position_ = (motor_enc_wrap_ - motor_index_pos_);
    float motor_x = motor_mechanical_position_ * inv_motor_encoder_cpr_;

    motor_position_ =
        param_.motor_encoder.dir *
        (2 * (float)M_PI * inv_motor_encoder_cpr_ * motor_enc_wrap_ +
         motor_index_pos_set_ * motor_correction_table_.table_interp(motor_x));
    motor_position_filtered_ = motor_position_;  //(1-alpha10)*motor_position_filtered_
                                                 //+ alpha10*motor_position_;
    motor_velocity = param_.motor_encoder.dir * (motor_enc_diff) *
                     (2 * (float)M_PI * inv_motor_encoder_cpr_ * frequency_hz_);
    motor_velocity_filtered =
        (1 - alpha) * motor_velocity_filtered + alpha * motor_velocity;
    last_motor_enc = motor_enc;

    // cogging compensation, interpolate in the table
    float iq_ff =
        param_.cogging.gain * cogging_correction_table_.table_interp(motor_x);

    if (mode_ == CURRENT_TUNING_MODE) {
      // only works down to frequencies of .047 Hz, could use kahansum to go
      // slower
      if (current_tuning_chirp_) {
        tuning_frequency_ = chirp_frequency_.add(chirp_rate_ * dt_);
      }
      phi_ += 2 * (float)M_PI * fabsf(tuning_frequency_) *
              dt_;  // use id des to set frequency
      if (phi_ > 2 * (float)M_PI) {
        phi_ -= 2 * (float)M_PI;
      }
      Sincos sincos;
      sincos = sincos1(phi_);
      iq_des =
          tuning_bias_ + tuning_amplitude_ *
                             (tuning_square_ ? fsignf(sincos.sin) : sincos.sin);
    }

    if (beep_) {
      if ((int32_t)(get_clock() - beep_end_) > 0) {
        beep_ = false;
      } else {
        phi_beep_ += 2 * (float)M_PI * fabsf(param_.beep_frequency) * dt_;
        if (phi_beep_ > 2 * (float)M_PI) {
          phi_beep_ -= 2 * (float)M_PI;
        }
        Sincos sincos = sincos1(phi_beep_);
        iq_ff += param_.beep_amplitude * fsignf(sincos.sin);
      }
    }

    // update FOC
    foc_command_.measured.motor_encoder =
        phase_mode_ * (motor_enc_wrap_ - motor_electrical_zero_pos_) *
        (2 * (float)M_PI * inv_motor_encoder_cpr_);
    foc_command_.desired.i_q = iq_des_gain_ * (iq_des + iq_ff);

    if (mode_ == STEPPER_TUNING_MODE) {
      foc_command_.measured.motor_encoder = stepper_position_;
      motor_position_filtered_ = stepper_position_;
      stepper_position_ += stepper_velocity_ * dt_;
      stepper_position_ = wrap1(stepper_position_, 2 * (float)M_PI);
    }

    FOCStatus *foc_status = foc_->step(foc_command_);

    // output pwm
    pwm_.set_voltage(&foc_status->command.v_a);

    dt_ = (timestamp_ - last_timestamp_) * (float)(1.0f / CPU_FREQUENCY_HZ);
    last_timestamp_ = timestamp_;

    if (zero_current_sensors_) {
      if ((int32_t)(get_clock() - zero_current_sensors_end_) > 0) {
        zero_current_sensors_ = false;
      } else {
        zero_current_sensors();
      }
    }
    store_status();
#ifdef END_TRIGGER_MOTOR_ENCODER
    motor_encoder_.trigger();
#endif
  }
  void maintenance() {
    if (motor_encoder_.index_received() && !motor_index_pos_set_) {
      motor_index_pos_ = motor_encoder_.get_index_pos();
      if (param_.motor_encoder.use_index_electrical_offset_pos) {
        // motor_index_electrical_offset_pos is the value of an electrical zero
        // minus the index position motor_electrical_zero_pos is the offset to
        // the initial encoder value
        motor_electrical_zero_pos_ =
            param_.motor_encoder.index_electrical_offset_pos + motor_index_pos_;
      }
      motor_index_pos_set_ = true;
    }

    if (mode_ == PHASE_LOCK_MODE) {
      motor_electrical_zero_pos_ = motor_encoder_.get_value();
      if (motor_encoder_.index_received()) {
        motor_index_pos_ = motor_encoder_.get_index_pos();
        int32_t index_offset = motor_electrical_zero_pos_ - motor_index_pos_;
        if (index_offset >= 0) {
          motor_index_electrical_offset_measured_ =
              index_offset %
              (param_.motor_encoder.cpr / (uint8_t)param_.foc_param.num_poles);
        } else {
          int32_t m =
              (param_.motor_encoder.cpr / (uint8_t)param_.foc_param.num_poles);
          motor_index_electrical_offset_measured_ =
              index_offset - m * ((index_offset / m) - 1);
        }
      }
    }

    v_bus_ = *v_bus_dr_ * param_.vbus_gain;
    pwm_.set_vbus(fmaxf(7, v_bus_));
  }
  void set_id_des(float id) { foc_command_.desired.i_d = id; }
  void set_iq_des(float iq) {
    if (mode_ == CURRENT_MODE) iq_des = iq;
  }
  void set_vq_des(float vq) { foc_command_.desired.v_q = vq; }
  void set_tuning_amplitude(float amplitude) { tuning_amplitude_ = amplitude; }
  void set_tuning_frequency(float frequency) { tuning_frequency_ = frequency; }
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
  void stepper_mode() {
    phase_mode_ = phase_mode_desired_;
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
    if (param_.motor_encoder.dir == 0) {
      param_.motor_encoder.dir = 1;
    }
    inv_motor_encoder_cpr_ =
        param_.motor_encoder.cpr != 0 ? 1.f / param_.motor_encoder.cpr : 0;
  }
  const FastLoopStatus &get_status() const { return status_.top(); }
  void store_status() {
    FastLoopStatus &s = status_.next();
    s.motor_mechanical_position = motor_mechanical_position_;
    s.motor_position.position = motor_position_filtered_;
    s.motor_position.raw = motor_enc;
    s.timestamp = timestamp_;
    s.vbus = v_bus_;
    s.foc_command = foc_command_;
    s.power = s.foc_status.command.v_d * s.foc_status.measured.i_d +
              s.foc_status.command.v_q * s.foc_status.measured.i_q;
    int32_t energy = s.power * dt_ * 1e6;
    energy_uJ_ += (uint32_t)energy;
    s.energy_uJ = energy_uJ_;
    foc_->get_status(&s.foc_status);
    status_.finish();
  }

  void zero_current_sensors() {
    param_.ia_bias = (1 - alpha_zero_) * param_.ia_bias +
                     alpha_zero_ * param_.adc1_gain * (adc1 - 2048);
    param_.ib_bias = (1 - alpha_zero_) * param_.ib_bias +
                     alpha_zero_ * param_.adc2_gain * (adc2 - 2048);
    param_.ic_bias = (1 - alpha_zero_) * param_.ic_bias +
                     alpha_zero_ * param_.adc3_gain * (adc3 - 2048);
  }
  void set_phase_mode() {
    phase_mode_desired_ = param_.phase_mode == 0 ? 1 : -1;
  }
  void set_phase_mode(uint8_t phase_mode) {
    param_.phase_mode = phase_mode;
    set_phase_mode();
  }
  uint8_t get_phase_mode() { return param_.phase_mode; }
  float get_rollover() const {
    return 2 * M_PI * inv_motor_encoder_cpr_ * param_.motor_encoder.rollover;
  }
  void beep_on(float t_seconds = 1) {
    beep_ = true;
    beep_end_ = get_clock() + t_seconds * CPU_FREQUENCY_HZ;
  }
  void beep_off() { beep_ = false; }

  void zero_current_sensors_on(float t_seconds = 1) {
    zero_current_sensors_ = true;
    zero_current_sensors_end_ = get_clock() + t_seconds * CPU_FREQUENCY_HZ;
  }
  void zero_current_sensors_off() { zero_current_sensors_ = false; }
  bool motor_encoder_error() { return motor_encoder_.error(); }
  void trigger_status_log() { status_log_.copy(status_); }

 private:
  FastLoopParam param_;
  FOC *foc_;
  PWMType &pwm_;
  enum {
    OPEN_MODE,
    BRAKE_MODE,
    CURRENT_MODE,
    PHASE_LOCK_MODE,
    VOLTAGE_MODE,
    CURRENT_TUNING_MODE,
    STEPPER_TUNING_MODE
  } mode_ = CURRENT_MODE;

  int32_t motor_enc;
  int32_t last_motor_enc = 0;
  float motor_position_ = 0;
  float motor_position_filtered_ = 0;
  float motor_velocity = 0;
  float motor_velocity_filtered = 0;
  float alpha = 0.001;
  float alpha10 = 1;      // 0.3859;   // 1/10 cutoff frequency
  float phase_mode_ = 1;  // 1: standard or -1: two wires switched
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
  float motor_index_electrical_offset_measured_ = NAN;
  float inv_motor_encoder_cpr_;
  int32_t frequency_hz_ = 100000;
  float alpha_zero_ = 0.0002;
  float v_bus_ = 12;
  mcu_time timestamp_;
  MotorEncoderType &motor_encoder_;
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
  CStack<FastLoopStatus, 100> status_;
  CStack<FastLoopStatus, 100> status_log_;
  bool beep_ = false;
  uint32_t beep_end_ = 0;
  bool zero_current_sensors_ = false;
  uint32_t zero_current_sensors_end_ = 0;
  float phi_beep_ = 0;
  uint32_t energy_uJ_ = 0;

  friend class System;
};

#endif  // UNHUMAN_MOTORLIB_FAST_LOOP_H_
