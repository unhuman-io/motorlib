#ifndef MOTOR_CONTROL_FUN_H
#define MOTOR_CONTROL_FUN_H

#include "messages.h"
#undef _DEFAULT_SOURCE
#include <cmath>
#define M_PI 3.1415926f
class Hysteresis {
 public:
    float step(float);
    void set_hysteresis(float);
    void set_value(float value) { value_ = value; }
 private:
    float value_ = 0;
    float hysteresis_ = 0;
};

float fsignf(float a);

class KahanSum {
 public:
    float add(float input) //__attribute__((section (".ccmram")))
    {
        float y = input - c_;
        float t = sum_ + y;
        c_ = (t - sum_) - y;
        sum_ = t;
        return sum_;
    }
    float value() const //__attribute__((section (".ccmram")))
    {
        return sum_;
    }
    void init(float value=0) {
        sum_ = value;
        c_ = 0;
    }
 private:
    float sum_ = 0;
    float c_ = 0;

};

class FirstOrderLowPassFilter {
public:
    FirstOrderLowPassFilter(float dt, float frequency_hz=0) {
        dt_ = dt;
        set_frequency(frequency_hz);
    }
    void init(float value) {
        value_ = value;
        last_value_ = value;
    }
    float update(float value) {
        value_ = alpha_*value + (1-alpha_)*last_value_;
        last_value_ = value_;
        return get_value();
    }
    float get_value() const { return value_; }
    void set_frequency(float frequency_hz) {
        if (frequency_hz == 0) {
            alpha_ = 1;
        } else { 
            alpha_ = 2*M_PI*dt_*frequency_hz/(2*M_PI*dt_*frequency_hz + 1);
        }
    }
private:
    float value_ = 0, last_value_ = 0;
    float alpha_, dt_;
};

#define IIRSIZE 4
class IIRFilter {
 public:
    float update(float value) {
        for (int i=IIRSIZE-1; i>0; i--) {
            x_[i] = x_[i-1];
            y_[i] = y_[i-1];
        }
        x_[0] = value;

        float v = 0;
        for (int i=0; i<IIRSIZE; i++) {
            v += x_[i]*b_[i];
        }
        float n = 0;
        for (int i=1; i<IIRSIZE; i++) {
            n += y_[i]*a_[i];
        }
        y_[0] = v - n;
        return y_[0];
    }
 private:
    float x_[IIRSIZE] = {};
    float y_[IIRSIZE] = {};
    // note a_[0] ignored
    float a_[IIRSIZE] = {1,         -2.87435689267748,           2.7564831952257,        -0.881893130592486};
    float b_[IIRSIZE] = {2.91464944656705e-05,      8.74394833970116e-05,      8.74394833970116e-05,      2.91464944656705e-05};
};

class PIController {
public:
    ~PIController() {}
    float step(float desired, float measured);
    void set_param(const PIParam &pi_param);
private:
    float kp_ = 0, ki_ = 0, ki_sum_ = 0, ki_limit_ = 0, command_max_ = 0;

    template<typename, typename>
    friend class System;
};

class RateLimiter {
 public:
    void set_limit(float limit) { limit_ = limit; }
    void init(float value) { last_value_ = value; velocity_ = 0;}
    float step(float value) {
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
    float get_velocity() const { return velocity_; }
 private:
    float limit_ = INFINITY;
    float velocity_ = 0;
    float last_value_ = 0;
};

class PIDController {
public:
    PIDController(float dt) : dt_(dt), error_dot_filter_(dt), output_filter_(dt) {}
    virtual ~PIDController() {}
    void init(float measured) { rate_limit_.init(measured), measured_last_ = measured; error_dot_filter_.init(0); output_filter_.init(0); } // todo init to current output 
    virtual float step(float desired, float velocity_desired, float measured, float velocity_limit = INFINITY);
    void set_param(const PIDParam &param);
private:
    float kp_ = 0, kd_ = 0, ki_ = 0, ki_sum_ = 0, ki_limit_ = 0, command_max_ = 0;
    float measured_last_ = 0;
    float last_desired_ = 0;
    float dt_;
    Hysteresis hysteresis_;
    RateLimiter rate_limit_;
    FirstOrderLowPassFilter error_dot_filter_;
    FirstOrderLowPassFilter output_filter_;
    template<typename, typename>
    friend class System;
};

class PIDDeadbandController : public PIDController {
public:
    PIDDeadbandController(float dt) : PIDController(dt) {}
    virtual ~PIDDeadbandController() {}
    virtual float step(float desired, float velocity_desired, float deadband, float measured, float velocity_limit = INFINITY);
};

class PIDInterpolateController : public PIDController {
 public:
    PIDInterpolateController(float dt, float filter_hz) : PIDController(dt), filt1_(dt, filter_hz), filt2_(dt, filter_hz) {}
    virtual ~PIDInterpolateController() {}
    virtual float step(float desired, float velocity_desired, float measured, float velocity_limit = INFINITY) {
        desired = filt2_.update(filt1_.update(desired));
        return step(desired, velocity_desired, measured, velocity_limit);
    }
 private:
    FirstOrderLowPassFilter filt1_, filt2_;
};




#endif //MOTOR_CONTROL_FUN_H
