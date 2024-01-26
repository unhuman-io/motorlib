#ifndef UNHUMAN_MOTORLIB_FOC_H_
#define UNHUMAN_MOTORLIB_FOC_H_

#include "messages.h"
#include "control_fun.h"

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
    float afc_cos_int_ = 0;
    float afc_sin_int_ = 0;
    float afc_out_ = 0;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_FOC_H_
