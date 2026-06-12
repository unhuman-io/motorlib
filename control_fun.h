#ifndef UNHUMAN_MOTORLIB_CONTROL_FUN_H_
#define UNHUMAN_MOTORLIB_CONTROL_FUN_H_

#include "messages.h"
#undef _DEFAULT_SOURCE
#include <cmath>
#ifndef M_PI
#define M_PI 3.141592653f
#endif
#include "sincos.h"
#include <algorithm>
#include <vector>
#include "st_device.h"

inline float fabsf2(float f) {
    return f >= 0 ? f : -f;
}

float fminf(float, float);

inline float flimit(float f, float limit) { return fminf(f, limit); }

#ifndef __clang__
inline float sqrtf(float f) {
    float r;
    asm("vsqrt.f32 %[dst], %[src]" : [dst] "=t" (r) : [src] "t" (f));
    return r;
}
#endif

// saturating add
template <class T>
inline T qadd(T a, T b) {
    if constexpr (std::is_same_v<T, uint8_t>) {
        return __UQADD8(a,b);
    } else if constexpr (std::is_same_v<T, uint16_t>) {
        return __UQADD16(a,b);
    } else if constexpr (std::is_same_v<T, uint32_t>) {
        return __UQADD(a,b);
    } else if constexpr (std::is_same_v<T, int8_t>) {
        return __QADD8(a,b);
    } else if constexpr (std::is_same_v<T, int16_t>) {
        return __QADD16(a,b);
    } else if constexpr (std::is_same_v<T, int32_t>) {
        return __QADD(a,b);
    } else {
        static_assert(0, "qadd not implemented for this type");
        return T();
    }
}

class Hysteresis {
 public:  
    Hysteresis(float hysteresis = 0, float value = 0) {
        set_hysteresis(hysteresis);
        set_value(value);
    }
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
    FirstOrderLowPassFilter(float dt=1, float frequency_hz=0) {
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
    void set_dt(float dt) {
        float frequency = get_frequency();
        dt_ = dt;
        if (!std::isfinite(frequency)) {
            frequency = 0;
        }
        set_frequency(frequency);
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

template <> const float FIRFilter<>::default_coeff_[];

class PIController {
public:
    ~PIController() {}
    float step(float desired, float measured);
    void set_param(const PIParam &pi_param);
    void initialize() { ki_sum_ = 0; }
private:
    float kp_ = 0, ki_ = 0, ki_sum_ = 0, ki_limit_ = 0, command_max_ = 0;

    template <typename T> friend class SystemBase;
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

    template <typename T> friend class SystemBase;
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


    template <typename T> friend class SystemBase;
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

class PRBSRand {
 public:
    // The seed can be any number except 0
    PRBSRand(uint32_t seed = 12345678) : val_(seed) {
        if (val_ == 0) {
            val_ = 1; 
        }
    }

    // Generates the next 32-bit random integer
    uint32_t next() {
        uint32_t x = val_;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        val_ = x;
        return val_;
    }
 private:
    uint32_t val_;
};

class BiquadFilter {
 public:
    BiquadFilter() : b0(1.0f), b1(0.0f), b2(0.0f), 
                     a1(0.0f), a2(0.0f) { init(); }

    void set_coeffs(float _b0, float _b1, float _b2, float _a0, float _a1, float _a2) {
        b0 = _b0 / _a0;
        b1 = _b1 / _a0;
        b2 = _b2 / _a0;
        a1 = _a1 / _a0;
        a2 = _a2 / _a0;

        init();
    }

    float get_gain(float cos_w, float cos_2w) const {
        float num = (b0*b0) + (b1*b1) + (b2*b2) 
                  + 2.0f * (b0*b1 + b1*b2) * cos_w 
                  + 2.0f * b0*b2 * cos_2w;
                  
        float den = 1.0f + (a1*a1) + (a2*a2) 
                  + 2.0f * (a1 + a1*a2) * cos_w 
                  + 2.0f * a2 * cos_2w;
                  
        return std::sqrt(num / den);
    }

    void init() {
        u1 = 0.0f; u2 = 0.0f;
        y1 = 0.0f; y2 = 0.0f;
    }

    float update(float u) {
        float y = (b0 * u) + (b1 * u1) + (b2 * u2) 
                - (a1 * y1) - (a2 * y2);
        u2 = u1;
        u1 = u;
        y2 = y1;
        y1 = y;
        return y;
    }
 private:
    // Numerator
    float b0, b1, b2;
    // Denominator
    float a1, a2;
    
    float u1, u2; // Past inputs
    float y1, y2; // Past outputs
};

template <float dt, int order = 6>
class BandPassFilter {
 public:
    void set_frequency(float frequency_start, float frequency_stop) {
        float f0 = std::sqrt(frequency_start * frequency_stop);
        float Q  = f0 / (frequency_stop - frequency_start);

        float w0 = 2.0f * static_cast<float>(M_PI) * f0 * dt;
        float sin_w0 = std::sin(w0);
        float cos_w0 = std::cos(w0);
        float alpha = sin_w0 / (2.0f * Q);

        // 3. Calculate RBJ Biquad Coefficients (Constant Peak Gain)
        float b0 = alpha;
        float b1 = 0.0f;
        float b2 = -alpha;
        float a0 = 1.0f + alpha;
        float a1 = -2.0f * cos_w0;
        float a2 = 1.0f - alpha;

        for (int i = 0; i < order; ++i) {
            filts_[i].set_coeffs(b0, b1, b2, a0, a1, a2);
        }
    }

    float update(float input) {
        float current_val = input;
        for (int i = 0; i < order; ++i) {
            current_val = filts_[i].update(current_val);
        }
        return current_val;
    }
 private:
    BiquadFilter filts_[order];
};

// ---------------------------------------------------------
// 2. N-Order Butterworth Low-Pass Filter
// ---------------------------------------------------------
template <float dt, int order = 4>
class ButterworthLowPass {
private:
    // The number of biquads needed is (order + 1) / 2 (integer division)
    static constexpr int num_stages = (order + 1) / 2;
    BiquadFilter filts_[num_stages];

public:
    void set_cutoff(float cutoff_hz) {
        // Pre-warp the frequency for the Bilinear Transform
        float w0 = 2.0f * static_cast<float>(M_PI) * cutoff_hz * dt;
        float sin_w0 = std::sin(w0);
        float cos_w0 = std::cos(w0);
        
        int stage = 0;

        // 1. Calculate the complex conjugate pole pairs (the 2nd-order sections)
        int num_biquads = order / 2;
        for (int k = 1; k <= num_biquads; ++k) {
            // Calculate the specific Q for this stage in the Butterworth circle
            float theta = static_cast<float>(M_PI) * (2.0f * k - 1.0f) / (2.0f * order);
            float Q = 1.0f / (2.0f * std::sin(theta));
            float alpha = sin_w0 / (2.0f * Q);

            // Calculate RBJ Low-Pass coefficients
            float b0 = (1.0f - cos_w0) / 2.0f;
            float b1 = 1.0f - cos_w0;
            float b2 = (1.0f - cos_w0) / 2.0f;
            float a0 = 1.0f + alpha;
            float a1 = -2.0f * cos_w0;
            float a2 = 1.0f - alpha;

            filts_[stage].set_coeffs(b0, b1, b2, a0, a1, a2);
            stage++;
        }

        // 2. If the order is odd, we have one real pole left over (a 1st-order section)
        if (order % 2 != 0) {
            // Pre-warped tangent for 1st order Bilinear Transform
            float W = std::tan(w0 / 2.0f);
            
            float b0 = W;
            float b1 = W;
            float b2 = 0.0f; // 1st order has no 2nd delay
            
            float a0 = W + 1.0f;
            float a1 = W - 1.0f;
            float a2 = 0.0f; // 1st order has no 2nd feedback
            
            filts_[stage].set_coeffs(b0, b1, b2, a0, a1, a2);
        }
    }

    float get_gain(float cos_w, float cos_2w) const {
        float gain = 1.0f;
        for (int i = 0; i < num_stages; ++i) {
            gain *= filts_[i].get_gain(cos_w, cos_2w);
        }
        return gain;
    }

    float update(float input) {
        float current_val = input;
        for (int i = 0; i < num_stages; ++i) {
            current_val = filts_[i].update(current_val);
        }
        return current_val;
    }
};

template <float dt, int order = 4>
class ButterworthHighPass {
private:
    static constexpr int num_stages = (order + 1) / 2;
    BiquadFilter filts_[num_stages];

public:
    void set_cutoff(float cutoff_hz) {
        float w0 = 2.0f * static_cast<float>(M_PI) * cutoff_hz * dt;
        float sin_w0 = std::sin(w0);
        float cos_w0 = std::cos(w0);
        
        int stage = 0;
        int num_biquads = order / 2;

        // 1. 2nd-order complex pole sections
        for (int k = 1; k <= num_biquads; ++k) {
            float theta = static_cast<float>(M_PI) * (2.0f * k - 1.0f) / (2.0f * order);
            float Q = 1.0f / (2.0f * std::sin(theta));
            float alpha = sin_w0 / (2.0f * Q);

            // RBJ High-Pass coefficients (Zeros change, Poles/Denominator stay same as LP)
            float b0 = (1.0f + cos_w0) / 2.0f;
            float b1 = -(1.0f + cos_w0);
            float b2 = (1.0f + cos_w0) / 2.0f;
            
            float a0 = 1.0f + alpha;
            float a1 = -2.0f * cos_w0;
            float a2 = 1.0f - alpha;

            filts_[stage].set_coeffs(b0, b1, b2, a0, a1, a2);
            stage++;
        }

        // 2. 1st-order real pole section (if order is odd)
        if (order % 2 != 0) {
            float W = std::tan(w0 / 2.0f);
            
            float b0 = 1.0f;
            float b1 = -1.0f;
            float b2 = 0.0f;
            
            float a0 = W + 1.0f;
            float a1 = W - 1.0f;
            float a2 = 0.0f;
            
            filts_[stage].set_coeffs(b0, b1, b2, a0, a1, a2);
        }
    }

    float get_gain(float cos_w, float cos_2w) const {
        float gain = 1.0f;
        for (int i = 0; i < num_stages; ++i) {
            gain *= filts_[i].get_gain(cos_w, cos_2w);
        }
        return gain;
    }

    float update(float input) {
        float current_val = input;
        for (int i = 0; i < num_stages; ++i) {
            current_val = filts_[i].update(current_val);
        }
        return current_val;
    }
};

// ---------------------------------------------------------
// 2. Butterworth Band-Pass (The Cascade Wrapper)
// ---------------------------------------------------------
template <float dt, int order = 8>
class ButterworthBandPass {
private:
    ButterworthHighPass<dt, order> hp_;
    // NOTE: Requires the ButterworthLowPass class from the previous response
    ButterworthLowPass<dt, order> lp_;
    float norm_factor_;

public:

    // Safe runtime update
    void set_frequency(float frequency_start, float frequency_stop) {
        hp_.set_cutoff(frequency_start); // Cut off everything BELOW start
        lp_.set_cutoff(frequency_stop);  // Cut off everything ABOVE stop

        float f0 = std::sqrt(frequency_start * frequency_stop);

        // 3. Pre-calculate the trig values for the magnitude equation
        float w0 = 2.0f * static_cast<float>(M_PI) * f0 * dt;
        float cos_w = std::cos(w0);
        float cos_2w = 2.0f * cos_w * cos_w - 1.0f; // Double angle identity

        // 4. Ask the cascade what its attenuation is at the center frequency
        float total_gain = hp_.get_gain(cos_w, cos_2w) * lp_.get_gain(cos_w, cos_2w);
        float center_comp = (total_gain > 0.00001f) ? (1.0f / total_gain) : 1.0f;

        // 4. Noise Energy compensation (fixes the bandwidth slicing)
        float f_nyq = 0.5f / dt; // Nyquist frequency
        float bw = frequency_stop - frequency_start;
        
        // Prevent division by zero if start and stop are accidentally equal
        float energy_comp = 1.0f;
        if (bw > 0.1f) {
            energy_comp = std::sqrt(f_nyq / bw);
        }

        // 5. Combine them into the final normalization factor
        norm_factor_ = center_comp * energy_comp;
    }

    float update(float input) {
        // Signal flows through High-Pass, then Low-Pass
        return lp_.update(hp_.update(input))*norm_factor_;
    }
};

template <float dt>
class TrajectoryGenerator {
 public:
    struct TrajectoryValue {
        float value, value_dot;
    };
    void set_frequency(float frequency) {
        frequency_ = frequency;
        filter_.set_frequency(frequency, frequency*2);
    }
    void set_amplitude(float amplitude) { amplitude_ = amplitude; }
    void set_mode(TuningMode mode) {
        if (mode <= TuningMode::RANDOM) {
            mode_ = mode;
            if (mode == TuningMode::CHIRP) {
                chirp_rate_ = frequency_;
                frequency_ = 0;
                chirp_frequency_.init();
            }
        }
    }
    void init(float phi=0) { phi_.init(phi); }

    TrajectoryValue &step() {
        if (mode_ == TuningMode::RANDOM) {
            float raw = amplitude_ * (2 * (float) fast_rng_.next() * (1.0 / static_cast<float>(0xFFFF'FFFF)) - 1);
            float raw_scaled = raw * random_scale_;
            
            float value_last = trajectory_value_.value;
            float new_value = filter_.update(raw_scaled);
            trajectory_value_.value = fsat(new_value, amplitude_);
            trajectory_value_.value_dot = (trajectory_value_.value - value_last) / dt;
            return trajectory_value_;
        }

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
    static constexpr float random_scale_ = .5;
    float frequency_, amplitude_;
    TrajectoryValue trajectory_value_;
    KahanSum phi_, chirp_frequency_;
    float chirp_rate_;
    PRBSRand fast_rng_{123456789};
    ButterworthBandPass<dt> filter_;

    template <typename T> friend class SystemBase;
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
    void step(float value, float frequency_hz, mcu_time);
    float magnitude_last_ = 0;
    float phase_last_ = 0;
    float frequency_last_ = 0; 
    float real_last_, imag_last_;
    int count_ = 1;
 private:
    mcu_time time_start_;
    float real_ = 0;
    float imag_ = 0;
    float frequency_ = 0;
    int num_points_;
};

class DFTResponse {
 public:
    DFTResponse(int num_points = 128) : desired_(num_points), measured_(num_points) {}
    void step(float desired, float measured, float frequency, mcu_time time) {
        desired_.step(desired, frequency, time);
        measured_.step(measured, frequency, time);
        if (desired_.count_ == 1) {
            magnitude_ = measured_.magnitude_last_ / desired_.magnitude_last_;
            phase_ = measured_.phase_last_ - desired_.phase_last_;
            if (phase_ > M_PI) {
                phase_ -= 2*M_PI;
            } else if (phase_ < -M_PI) {
                phase_ += 2*M_PI;
            }
        }
    }

    DFT desired_, measured_;
    float magnitude_, phase_;
};



#endif  // UNHUMAN_MOTORLIB_CONTROL_FUN_H_
