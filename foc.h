#ifndef UNHUMAN_MOTORLIB_FOC_H_
#define UNHUMAN_MOTORLIB_FOC_H_

#include "messages.h"
#include "control_fun.h"

class MotorModel {
 public:
    MotorModel(float dt): dt_(dt) {}
    float step(float v, float i);
    void set_param(float R, float L);
 private:
    float i_ = 0;
    float Ad_, Bd_;
    float dt_;
};

class MotorEstimator {
 public:
    MotorEstimator(float dt): dt_(dt), motor_model_(dt), v_emf_filter_(dt), z_filter_(dt) {}
    void update(float i, float v);
    float v_emf() const { return v_emf_; }
 private:
    float dt_;
    MotorModel motor_model_;
    FirstOrderLowPassFilter v_emf_filter_;
    FirstOrderLowPassFilter z_filter_;
    float K_;
    float v_emf_;
};

class SensorlessEstimator {
 public:
    SensorlessEstimator(float dt): dt_(dt), estimator_alpha_(dt), estimator_beta_(dt), velocity_filter_(dt) {
        frequency_hz_ = 1/dt;
    }
    void update(float i_alpha, float i_beta, float v_alpha, float v_beta);
    float angle_estimate() const { return angle_estimate_; }
 private:
    float dt_;
    float frequency_hz_;
    MotorEstimator estimator_alpha_, estimator_beta_;
    int32_t angle_last_ = 0;
    FirstOrderLowPassFilter velocity_filter_;
    float Kspeed_;
    float angle_estimate_;
};

class FOC {
public:
    FOC(float dt);
    ~FOC();
    struct Vdq0 {
        float vd, vq, v0;
    };

    FOCStatus * const step(const FOCCommand &command)  __attribute__((section (".ccmram")));
    void set_param(const FOCParam &param);
    void get_status(FOCStatus *status) const { *status = status_; }
    void voltage_mode();
    void current_mode();
    static void calculate_vdq0(Vdq0 *const, float cos, float sin, float va, float vb, float vc);
    void set_id_limit(float limit) { id_limiter_.set_limit(limit*dt_); }
    void set_iq_limit(float limit) { iq_limiter_.set_limit(limit*dt_); }
    float get_id_limit() const { return id_limiter_.get_limit()/dt_; }
    float get_iq_limit() const { return iq_limiter_.get_limit()/dt_; }

private:
    float num_poles_ = 7;
    volatile float i_gain_ = 0;
    PI2Controller pi_id_, pi_iq_;
    FOCStatus status_;
    static const float Kc[2][3];
    float dt_;
    FirstOrderLowPassFilter id_filter_, iq_filter_;
    RateLimiter id_limiter_, iq_limiter_;
    FOCParam param_;
    SensorlessEstimator sensorless_estimator_;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_FOC_H_
