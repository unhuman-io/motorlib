#ifndef UNHUMAN_MOTORLIB_CONTROL_FUN_H_
#define UNHUMAN_MOTORLIB_CONTROL_FUN_H_

#include "messages.h"
#undef _DEFAULT_SOURCE
#include <cmath>
#define M_PI 3.1415926f
#include "sincos.h"
#include <algorithm>
#include <vector>

inline float fabsf2(float f) {
    return f >= 0 ? f : -f;
}

class Hysteresis {
 public:
    float step(float);
    void set_hysteresis(float);
    void set_value(float value) { value_ = value; }
 private:
    float value_ = 0;
    float hysteresis_ = 0;
};

template<int size=5>
class MedianFilter {
 public:
    MedianFilter() :
        data_(size, 0), data_tmp_(size) {}
    float update(float value) {
        pos_++;
        if (pos_ >= size) {
            pos_ = 0;
        }
        data_[pos_] = value;
        sort();
        return data_tmp_[size/2];
    }
    std::vector<float> &sort() {
        for (int i=0; i<size; i++) {
            data_tmp_[i] = data_[i];
        }
        std::sort(data_tmp_.begin(), data_tmp_.end());
        return data_tmp_;
    }
 private:
    std::vector<float> data_;
    std::vector<float> data_tmp_;
    int pos_ = 0;
};

float fsat(float a, float sat);

inline float fsat2(float a, float min, float max) {
    float b = a>max ? max : a;
    b = b<min ? min : b;
    return b;
}

float fsignf(float a);
inline int32_t sign(int32_t a) {
    return a > 0 ? 1 : (a < 0 ? -1 : 0);
}

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
    float get_frequency() const {
        return alpha_/(2*M_PI*dt_*(1-alpha_));
    }
private:
    float value_ = 0, last_value_ = 0;
    float alpha_, dt_;
};

class SecondOrderLowPassFilter {
 public:
    SecondOrderLowPassFilter(float dt, float frequency_hz=0) :
        low_pass_1_(dt, frequency_hz), low_pass_2_(dt, frequency_hz) {}
    void init(float value) {
        low_pass_1_.init(value);
        low_pass_2_.init(value);
    }
    float update(float value) {
        return low_pass_2_.update(low_pass_1_.update(value));
    }
    float get_value() const { return low_pass_2_.get_value(); }
    void set_frequency(float frequency_hz) {
        low_pass_1_.set_frequency(frequency_hz);
        low_pass_2_.set_frequency(frequency_hz);
    }
    float get_frequency() const {
        return low_pass_1_.get_frequency();
    }
 private:
    FirstOrderLowPassFilter low_pass_1_, low_pass_2_;
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

template<int size=11>
class FIRFilter {
 public:
    FIRFilter(float dt, const float coeff[size]) : dt_(dt), coeff_(coeff) {
        bool empty_coeff = true;
        for (int i=0; i<size; i++) {
            if (coeff_[i] != 0) {
                empty_coeff = false;
            }
        }
        if (empty_coeff) {
            coeff_ = default_coeff_;
        }
    }
    float update(float value) {
        current_pos_++;
        if (current_pos_ >= size) {
            current_pos_ = 0;
        }
        values_[current_pos_] = value;

        float out = 0;
        for (int i=0; i<size; i++) {        
            current_pos_++;
            if (current_pos_ >= size) {
                current_pos_ = 0;
            }
            out += values_[current_pos_]*coeff_[i];
           
        }
        return out/dt_;
    }
 private:
    float dt_;
    const float *coeff_;
    static const float default_coeff_[size];
    float values_[size] = {};
    int current_pos_;
};

class PIController {
public:
    ~PIController() {}
    float step(float desired, float measured);
    void set_param(const PIParam &pi_param);
    void initialize() { ki_sum_ = 0; }
private:
    float kp_ = 0, ki_ = 0, ki_sum_ = 0, ki_limit_ = 0, command_max_ = 0;

    friend class System;
};

class PI2Controller {
public:
    ~PI2Controller() {}
    float step(float desired, float measured);
    void set_param(const PI2Param &pi_param);
    void initialize() { ki_sum_ = 0; }
    PI2Param get_param() const;
private:
    float kp_ = 0, ki_ = 0, ki_sum_ = 0, ki_limit_ = 0, command_max_ = 0, kp2_ = 0, ki2_ = 0, value2_ = 0, inv_value2_ = 1;

    friend class System;
};

class RateLimiter {
 public:
    void set_limit(float limit) { limit_ = (limit == 0 ? INFINITY : limit); }
    void init(float value, float velocity = 0) { last_value_ = value; velocity_ = velocity;}
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
    float get_limit() const { return limit_; }
    float get_value() const { return last_value_; }
    float get_velocity() const { return velocity_; }
 private:
    float limit_ = INFINITY;
    float velocity_ = 0;
    float last_value_ = 0;
};

class PIDController {
public:
    PIDController(float dt) : velocity_filter_(dt), output_filter_(dt), dt_(dt) {}
    ~PIDController() {}
    void init(float measured) { rate_limit_.init(measured), ki_sum_ = 0; measured_last_ = measured; velocity_filter_.init(0); output_filter_.init(0); } // todo init to current output 
    float step(float desired, float velocity_desired, float measured, float velocity_limit = INFINITY);
    void set_param(const PIDParam &param);
    float get_error() const { return error_last_; }
    void set_rollover(float rollover) { rollover_ = rollover; }
    
    float kp_ = 0, kd_ = 0, ki_ = 0, ki_sum_ = 0, ki_limit_ = 0, command_max_ = 0;
    SecondOrderLowPassFilter velocity_filter_;
    FirstOrderLowPassFilter output_filter_;
protected:
    float error_ = 0, velocity_measured_ = 0;
    float measured_last_ = 0;
    
    float error_last_ = 0;
    float last_desired_ = 0;
    float dt_;
    float rollover_ = 0;
    Hysteresis hysteresis_;
    RateLimiter rate_limit_;


    friend class System;
    friend void config_init();
};

class PIDWrapController : public PIDController {
 public:
    float step(float desired, float velocity_desired, float measured, float velocity_limit = INFINITY);
};

class PIDDeadbandController : public PIDController {
public:
    PIDDeadbandController(float dt) : PIDController(dt) {}
    ~PIDDeadbandController() {}
    float step(float desired, float velocity_desired, float deadband, float measured, float velocity_limit = INFINITY);
};

class PIDInterpolateController : public PIDController {
 public:
    PIDInterpolateController(float dt, float filter_hz) : PIDController(dt), filt1_(dt, filter_hz), filt2_(dt, filter_hz) {}
    ~PIDInterpolateController() {}
    float step(float desired, float velocity_desired, float measured, float velocity_limit = INFINITY) {
        desired = filt2_.update(filt1_.update(desired));
        return PIDController::step(desired, velocity_desired, measured, velocity_limit);
    }
 private:
    FirstOrderLowPassFilter filt1_, filt2_;
};

class TrajectoryGenerator {
 public:
    struct TrajectoryValue {
        float value, value_dot;
    };
    void set_frequency(float frequency) { frequency_ = frequency; }
    void set_amplitude(float amplitude) { amplitude_ = amplitude; }
    void set_mode(TuningMode mode) {
        if (mode <= TuningMode::CHIRP) {
            mode_ = mode;
            if (mode == TuningMode::CHIRP) {
                chirp_rate_ = frequency_;
                frequency_ = 0;
                chirp_frequency_.init();
            }
        }
    }
    void init(float phi=0) { phi_.init(phi); }

    TrajectoryValue &step(float dt) {
        // phi_ is a radian counter at the command frequency doesn't get larger than 2*pi
        if (mode_ == TuningMode::CHIRP) {
           frequency_ = chirp_frequency_.add(chirp_rate_ * dt);
        }
        // KahanSum allows for and summing of dt allows for low frequencies without losing resolution
        phi_.add(2 * (float) M_PI * fabsf(frequency_) * dt);
        if (phi_.value() > 2 * (float) M_PI) {  
            phi_.add(-2 * (float) M_PI);
        }
        Sincos sincos;
        sincos = sincos1(phi_.value());
        switch(mode_) {
            case TuningMode::SINE:
            case TuningMode::CHIRP:
                trajectory_value_.value = amplitude_ * sincos.sin;
                trajectory_value_.value_dot = 2 * (float) M_PI * frequency_ * amplitude_ * sincos.cos;
                break;
            case TuningMode::SQUARE:
                trajectory_value_.value = amplitude_ * fsignf(sincos.sin);
                trajectory_value_.value_dot = 0;
                break;
            case TuningMode::TRIANGLE:
                if (phi_.value() < M_PI) {
                    trajectory_value_.value = amplitude_ * (2 * phi_.value() * (1/M_PI) - 1);
                    trajectory_value_.value_dot = 4 * amplitude_ * frequency_;
                } else {
                    trajectory_value_.value = amplitude_ * (3 - 2 * phi_.value() * (1/M_PI));
                    trajectory_value_.value_dot = -4 * amplitude_ * frequency_;
                }
                break;
        }
        return trajectory_value_;
    }
    float * value() { return &trajectory_value_.value; }
    float get_amplitude() const { return amplitude_; }
    float get_frequency() const { return frequency_; }
 private:
    TuningMode mode_ = TuningMode::SINE;
    float frequency_, amplitude_;
    TrajectoryValue trajectory_value_;
    KahanSum phi_, chirp_frequency_;
    float chirp_rate_;
    friend class System;
};

template<class T>
inline T wrap1(T value, T rollover) {
    T diff = 2*rollover;
    if (value > rollover) {
        value -= diff;
    }
    if (value < -rollover) {
        value += diff;
    }
    return value;
}

template<class T>
inline T unwrap1(T value, T last_value, T rollover) {
    T diff = value - last_value;
    T diff2 = 2*rollover;
    if (diff > rollover) {
        value -= diff2;
    }
    if (diff < -rollover) {
        value += diff2;
    }
    return value;
}

template<class T>
inline T wrap1_diff(T value, T value2, T rollover) {
    T diff = value - value2;
    T diff2 = 2*rollover;
    if (diff > rollover) {
        diff = diff - diff2;
    }
    if (diff < -rollover) {
        diff = diff + diff2;
    }
    return diff;
}

class DFT {
 public:
    DFT(int num_points = 128) : num_points_(num_points) {}
    void step(float value, float frequency_hz);
    float magnitude_last_ = 0;
    float phase_last_ = 0;
    float frequency_last_ = 0;
 private:
    float real_ = 0;
    float imag_ = 0;
    float frequency_ = 0;
    int count_ = 1;
    int num_points_;
};



#endif  // UNHUMAN_MOTORLIB_CONTROL_FUN_H_
