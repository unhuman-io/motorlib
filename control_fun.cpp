#include "control_fun.h"

#include <cmath>

float fsat(float a, float sat) {
  // Slow version
  // if (a > sat) {
  //     return sat;
  // } else if (a < -sat) {
  //     return -sat;
  // } else {
  //     return a;
  // }

  // Doesn't completely optimize to branchless instructions
  // return a>sat ? sat : (a<-sat ? -sat : a);

  // Optimizes to branchless
  float b = a > sat ? sat : a;
  b = b < -sat ? -sat : b;
  return b;

  // Not so great
  // float s = a>0 ? sat : -sat;
  // return std::abs(a)-sat>0 ? s : a;

  // Assembly
  // asm("vcmpe.f32 %[a], %[sat]\n\t"
  //     "vmrs APSR_nzcv, fpscr\n\t"
  //     "it gt\n\t"
  //     "vmovgt.f32 %[a], %[sat]\n\t"
  //     "vneg.f32 %[sat], %[sat]\n\t"
  //     "vcmpe.f32 %[a], %[sat]\n\t"
  //     "vmrs APSR_nzcv, fpscr\n\t"
  //     "it lt\n\t"
  //     "vmovlt.f32 %[a], %[sat]\n\t"
  //     :
  //     : [a] "t" (a), [sat] "t" (sat));
  // return a;
}

float fsignf(float a) { return a >= 0 ? 1 : -1; }

void Hysteresis::set_hysteresis(float value) { hysteresis_ = value; }

float Hysteresis::step(float value) {
  if (value - value_ > hysteresis_) {
    value_ = value - hysteresis_;
  } else if (value - value_ < -hysteresis_) {
    value_ = value + hysteresis_;
  }
  return value_;
}

float KahanSum::add(float input) {
  float y = input - c_;
  float t = sum_ + y;
  c_ = (t - sum_) - y;
  sum_ = t;
  return sum_;
}
float KahanSum::value() const { return sum_; }
void KahanSum::init(float value) {
  sum_ = value;
  c_ = 0;
}

FirstOrderLowPassFilter::FirstOrderLowPassFilter(float dt, float frequency_hz) {
  dt_ = dt;
  set_frequency(frequency_hz);
}
void FirstOrderLowPassFilter::init(float value) {
  value_ = value;
  last_value_ = value;
}
float FirstOrderLowPassFilter::update(float value) {
  value_ = alpha_ * value + (1 - alpha_) * last_value_;
  last_value_ = value_;
  return get_value();
}
float FirstOrderLowPassFilter::get_value() const { return value_; }
void FirstOrderLowPassFilter::set_frequency(float frequency_hz) {
  if (frequency_hz == 0) {
    alpha_ = 1;
  } else {
    alpha_ =
        2 * M_PI * dt_ * frequency_hz / (2 * M_PI * dt_ * frequency_hz + 1);
  }
}
float FirstOrderLowPassFilter::get_frequency() const {
  return alpha_ / (2 * M_PI * dt_ * (1 - alpha_));
}

void SecondOrderLowPassFilter::init(float value) {
  low_pass_1_.init(value);
  low_pass_2_.init(value);
}
float SecondOrderLowPassFilter::update(float value) {
  return low_pass_2_.update(low_pass_1_.update(value));
}
float SecondOrderLowPassFilter::get_value() const {
  return low_pass_2_.get_value();
}
void SecondOrderLowPassFilter::set_frequency(float frequency_hz) {
  low_pass_1_.set_frequency(frequency_hz);
  low_pass_2_.set_frequency(frequency_hz);
}
float SecondOrderLowPassFilter::get_frequency() const {
  return low_pass_1_.get_frequency();
}

float IIRFilter::update(float value) {
  for (int i = kIirSize - 1; i > 0; i--) {
    x_[i] = x_[i - 1];
    y_[i] = y_[i - 1];
  }
  x_[0] = value;

  float v = 0;
  for (int i = 0; i < kIirSize; i++) {
    v += x_[i] * b_[i];
  }
  float n = 0;
  for (int i = 1; i < kIirSize; i++) {
    n += y_[i] * a_[i];
  }
  y_[0] = v - n;
  return y_[0];
}

void PIController::set_param(const PIParam &pi_param) {
  ki_ = pi_param.ki;
  kp_ = pi_param.kp;
  ki_limit_ = pi_param.ki_limit;
  command_max_ = pi_param.command_max;
}

float PIController::step(float desired, float measured) {
  float error = desired - measured;
  ki_sum_ += ki_ * error;
  ki_sum_ = fsat(ki_sum_, ki_limit_);
  return fsat(kp_ * error + ki_sum_, command_max_);
}

void RateLimiter::set_limit(float limit) {
  limit_ = (limit == 0 ? INFINITY : limit);
}
void RateLimiter::init(float value, float velocity) {
  last_value_ = value;
  velocity_ = velocity;
}
float RateLimiter::step(float value) {
  float out_value = value;
  if (value > (last_value_ + limit_)) {
    out_value = last_value_ + limit_;
    velocity_ = limit_;
  } else if (value < (last_value_ - limit_)) {
    out_value = last_value_ - limit_;
    velocity_ = -limit_;
  } else {
    out_value = value;
    velocity_ = value - last_value_;
  }

  last_value_ = out_value;
  return out_value;
}
float RateLimiter::get_value() const { return last_value_; }
float RateLimiter::get_velocity() const { return velocity_; }

void PIDController::init(float measured) {
  // TODO: Initialize to the current output instead of zero.
  rate_limit_.init(measured);
  ki_sum_ = 0;
  measured_last_ = measured;
  velocity_filter_.init(0);
  output_filter_.init(0);
}
void PIDController::set_param(const PIDParam &param) {
  ki_ = param.ki;
  kp_ = param.kp;
  ki_limit_ = param.ki_limit;
  kd_ = param.kd;
  command_max_ = param.command_max;
  velocity_filter_.set_frequency(param.velocity_filter_frequency_hz);
  output_filter_.set_frequency(param.output_filter_frequency_hz);
  hysteresis_.set_hysteresis(command_max_ / kp_);
}

float PIDController::step(float desired, float velocity_desired, float measured,
                          float velocity_limit) {
  // PID controller with formula
  // out = (ki/s + kp + kd*s) * error
  // with s*error given by velocity_desired - velocity measured from measured
  // with internal 2nd order filter with saturation on ki/s * error: ki_limit
  // saturation on out: command_max
  // output filter is first order filter for lead controller option
  // velocity limit: velocity_limit on command - not if not tracking velocity
  // can exceed this

  rate_limit_.set_limit(fabsf(velocity_limit * dt_));

  // proxy is a prefix for a limited desired value
  float proxy_desired = rate_limit_.step(desired);
  float proxy_velocity_desired = fsat(velocity_desired, velocity_limit);
  error_ = proxy_desired - measured;
  velocity_measured_ =
      velocity_filter_.update((measured - measured_last_) / dt_);
  float error_dot = proxy_velocity_desired - velocity_measured_;
  measured_last_ = measured;
  ki_sum_ += ki_ * dt_ * error_;
  ki_sum_ = fsat(ki_sum_, ki_limit_);
  float filtered_out =
      output_filter_.update(kp_ * error_ + ki_sum_ + kd_ * error_dot);
  return fsat(filtered_out, command_max_);
}

float PIDWrapController::step(float desired, float velocity_desired,
                              float measured, float velocity_limit) {
  // PID controller with formula
  // out = (ki/s + kp + kd*s) * error
  // with s*error given by velocity_desired - velocity measured from measured
  // with internal 2nd order filter with saturation on ki/s * error: ki_limit
  // saturation on out: command_max
  // output filter is first order filter for lead controller option
  // velocity limit: velocity_limit on command - not if not tracking velocity
  // can exceed this rollover allows for going to the closest position if
  // through rollover,
  //      e.g. rollover = 2*pi, measured = 1.5*pi, desired = -1.5*pi, motor will
  //      go through rollover if desired is > 2*pi motor will still calculate
  //      error correctly, but will spin forever

  rate_limit_.set_limit(fabsf(velocity_limit * dt_));
  float desired_wrap = wrap1(desired, rollover_);
  // wrap the rate limiter if necessary
  if (fabsf(desired_wrap - rate_limit_.get_value()) > 1.5 * rollover_) {
    rate_limit_.init(rate_limit_.get_value() + desired_wrap - desired,
                     rate_limit_.get_velocity());
  }

  // proxy is a prefix for a limited desired value
  float proxy_desired = rate_limit_.step(desired_wrap);
  // float proxy_wrap = wrap1(proxy_desired, rollover_);
  // rate_limit_.init(proxy_wrap, rate_limit_.get_velocity());
  // float proxy_dot_desired = fsat(velocity_desired,
  // fabsf(rate_limit_.get_velocity()/dt_));
  error_ = wrap1_diff(proxy_desired, measured, rollover_);
  float error_dot = velocity_filter_.update((error_ - error_last_) / dt_);
  error_last_ = error_;
  ki_sum_ += ki_ * dt_ * error_;
  ki_sum_ = fsat(ki_sum_, ki_limit_);
  float filtered_out =
      output_filter_.update(kp_ * error_ + ki_sum_ + kd_ * error_dot);
  return fsat(filtered_out, command_max_);
}

float PIDDeadbandController::step(float desired, float velocity_desired,
                                  float deadband, float measured,
                                  float velocity_limit) {
  float desired_with_deadband =
      fsignf(desired - measured) *
          fmaxf(fabsf(desired - measured) - deadband, 0) +
      measured;
  return PIDController::step(desired_with_deadband, velocity_desired, measured,
                             velocity_limit);
}

void TrajectoryGenerator::set_frequency(float frequency) {
  frequency_ = frequency;
}

void TrajectoryGenerator::set_amplitude(float amplitude) {
  amplitude_ = amplitude;
}

void TrajectoryGenerator::set_mode(TuningMode mode) {
  if (mode <= TuningMode::CHIRP) {
    mode_ = mode;
    if (mode == TuningMode::CHIRP) {
      chirp_rate_ = frequency_;
      frequency_ = 0;
      chirp_frequency_.init();
    }
  }
}

TrajectoryGenerator::TrajectoryValue &TrajectoryGenerator::step(float dt) {
  // phi_ is a radian counter at the command frequency doesn't get larger than
  // 2*pi
  if (mode_ == TuningMode::CHIRP) {
    frequency_ = chirp_frequency_.add(chirp_rate_ * dt);
  }
  // KahanSum allows for and summing of dt allows for low frequencies without
  // losing resolution
  phi_.add(2 * (float)M_PI * fabsf(frequency_) * dt);
  if (phi_.value() > 2 * (float)M_PI) {
    phi_.add(-2 * (float)M_PI);
  }
  Sincos sincos;
  sincos = sincos1(phi_.value());
  switch (mode_) {
    case TuningMode::SINE:
    case TuningMode::CHIRP:
      trajectory_value_.value = amplitude_ * sincos.sin;
      trajectory_value_.value_dot =
          2 * (float)M_PI * frequency_ * amplitude_ * sincos.cos;
      break;
    case TuningMode::SQUARE:
      trajectory_value_.value = amplitude_ * fsignf(sincos.sin);
      trajectory_value_.value_dot = 0;
      break;
    case TuningMode::TRIANGLE:
      if (phi_.value() < M_PI) {
        trajectory_value_.value =
            amplitude_ * (2 * phi_.value() * (1 / M_PI) - 1);
        trajectory_value_.value_dot = 4 * amplitude_ * frequency_;
      } else {
        trajectory_value_.value =
            amplitude_ * (3 - 2 * phi_.value() * (1 / M_PI));
        trajectory_value_.value_dot = -4 * amplitude_ * frequency_;
      }
      break;
  }
  return trajectory_value_;
}
