#ifndef UNHUMAN_MOTORLIB_CONTROL_FUN_H_
#define UNHUMAN_MOTORLIB_CONTROL_FUN_H_

#include "messages.h"
#undef _DEFAULT_SOURCE
#include <cmath>
#define M_PI 3.1415926f
#include "sincos.h"

class Hysteresis {
 public:
  float step(float);
  void set_hysteresis(float);
  void set_value(float value) { value_ = value; }

 private:
  float value_ = 0;
  float hysteresis_ = 0;
};

float fsat(float a, float sat);

inline float fsat2(float a, float min, float max) {
  float b = a > max ? max : a;
  b = b < min ? min : b;
  return b;
}

float fsignf(float a);
inline int32_t sign(int32_t a) { return a > 0 ? 1 : (a < 0 ? -1 : 0); }

class KahanSum {
 public:
  float add(float input);  //__attribute__((section (".ccmram")))
  float value() const;     //__attribute__((section (".ccmram")))
  void init(float value = 0);

 private:
  float sum_ = 0;
  float c_ = 0;
};

class FirstOrderLowPassFilter {
 public:
  FirstOrderLowPassFilter(float dt, float frequency_hz = 0);
  void init(float value);
  float update(float value);
  float get_value() const;
  void set_frequency(float frequency_hz);
  float get_frequency() const;

 private:
  float value_ = 0;
  float last_value_ = 0;
  float alpha_ = 0;
  float dt_ = 0;
};

class SecondOrderLowPassFilter {
 public:
  SecondOrderLowPassFilter(float dt, float frequency_hz = 0)
      : low_pass_1_(dt, frequency_hz), low_pass_2_(dt, frequency_hz) {}
  void init(float value);
  float update(float value);
  float get_value() const;
  void set_frequency(float frequency_hz);
  float get_frequency() const;

 private:
  FirstOrderLowPassFilter low_pass_1_;
  FirstOrderLowPassFilter low_pass_2_;
};

constexpr int kIirSize = 4;
class IIRFilter {
 public:
  float update(float value);

 private:
  float x_[kIirSize] = {};
  float y_[kIirSize] = {};
  // note a_[0] ignored
  static constexpr float a_[kIirSize] = {1, -2.87435689267748, 2.7564831952257,
                                         -0.881893130592486};
  static constexpr float b_[kIirSize] = {
      2.91464944656705e-05, 8.74394833970116e-05, 8.74394833970116e-05,
      2.91464944656705e-05};
};

class PIController {
 public:
  ~PIController() {}
  float step(float desired, float measured);
  void set_param(const PIParam &pi_param);
  void initialize() { ki_sum_ = 0; }

 private:
  float kp_ = 0;
  float ki_ = 0;
  float ki_sum_ = 0;
  float ki_limit_ = 0;
  float command_max_ = 0;

  friend class System;
};

class RateLimiter {
 public:
  void set_limit(float limit);
  void init(float value, float velocity = 0);
  float step(float value);
  float get_value() const;
  float get_velocity() const;

 private:
  float limit_ = INFINITY;
  float velocity_ = 0;
  float last_value_ = 0;
};

class PIDController {
 public:
  PIDController(float dt) : dt_(dt), velocity_filter_(dt), output_filter_(dt) {}
  virtual ~PIDController() {}
  void init(float measured);
  virtual float step(float desired, float velocity_desired, float measured,
                     float velocity_limit = INFINITY);
  void set_param(const PIDParam &param);
  float get_error() const { return error_last_; }
  void set_rollover(float rollover) { rollover_ = rollover; }

 protected:
  float error_ = 0;
  float velocity_measured_ = 0;
  float measured_last_ = 0;
  float kp_ = 0;
  float kd_ = 0;
  float ki_ = 0;
  float ki_sum_ = 0;
  float ki_limit_ = 0;
  float command_max_ = 0;
  float error_last_ = 0;
  float last_desired_ = 0;
  float dt_ = 0;
  float rollover_ = 0;
  Hysteresis hysteresis_;
  RateLimiter rate_limit_;
  SecondOrderLowPassFilter velocity_filter_;
  FirstOrderLowPassFilter output_filter_;

  friend class System;
  friend void config_init();
};

class PIDWrapController : public PIDController {
 public:
  virtual float step(float desired, float velocity_desired, float measured,
                     float velocity_limit = INFINITY);
};

class PIDDeadbandController : public PIDController {
 public:
  PIDDeadbandController(float dt) : PIDController(dt) {}
  virtual ~PIDDeadbandController() {}
  virtual float step(float desired, float velocity_desired, float deadband,
                     float measured, float velocity_limit = INFINITY);
};

class PIDInterpolateController : public PIDController {
 public:
  PIDInterpolateController(float dt, float filter_hz)
      : PIDController(dt), filt1_(dt, filter_hz), filt2_(dt, filter_hz) {}
  virtual ~PIDInterpolateController() {}
  virtual float step(float desired, float velocity_desired, float measured,
                     float velocity_limit = INFINITY) {
    desired = filt2_.update(filt1_.update(desired));
    return PIDController::step(desired, velocity_desired, measured,
                               velocity_limit);
  }

 private:
  FirstOrderLowPassFilter filt1_;
  FirstOrderLowPassFilter filt2_;
};

class TrajectoryGenerator {
 public:
  struct TrajectoryValue {
    float value;
    float value_dot;
  };
  void set_frequency(float frequency);
  void set_amplitude(float amplitude);
  void set_mode(TuningMode mode);

  TrajectoryValue &step(float dt);

 private:
  TuningMode mode_ = TuningMode::SINE;
  float frequency_;
  float amplitude_;
  TrajectoryValue trajectory_value_;
  KahanSum phi_;
  KahanSum chirp_frequency_;
  float chirp_rate_;
};

template <class T>
inline T wrap1(T value, T rollover) {
  T diff = 2 * rollover;
  if (value > rollover) {
    value -= diff;
  }
  if (value < -rollover) {
    value += diff;
  }
  return value;
}

template <class T>
inline T unwrap1(T value, T last_value, T rollover) {
  T diff = value - last_value;
  T diff2 = 2 * rollover;
  if (diff > rollover) {
    value -= diff2;
  }
  if (diff < -rollover) {
    value += diff2;
  }
  return value;
}

template <class T>
inline T wrap1_diff(T value, T value2, T rollover) {
  T diff = value - value2;
  T diff2 = 2 * rollover;
  if (diff > rollover) {
    diff = diff - diff2;
  }
  if (diff < -rollover) {
    diff = diff + diff2;
  }
  return diff;
}

#endif  // UNHUMAN_MOTORLIB_CONTROL_FUN_H_
