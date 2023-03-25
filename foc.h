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

private:
    float num_poles_ = 7;
    volatile float i_gain_ = 0;
    PIController pi_id_, pi_iq_;
    FOCStatus status_;
    static const float Kc[2][3];
    float dt_;
    FirstOrderLowPassFilter id_filter_, iq_filter_;
    FOCParam param_;

    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_FOC_H_
