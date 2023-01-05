#include "foc.h"
#include "control_fun.h"
#include "sincos.h"

#include <cmath>

FOC::FOC(float dt) : dt_(dt) {
    pi_id_ = new PIController();
    pi_iq_ = new PIController();
    id_filter_ = new FirstOrderLowPassFilter(dt);
    iq_filter_ = new FirstOrderLowPassFilter(dt);
}

FOC::~FOC() {
    delete pi_id_;
    delete pi_iq_;
    delete id_filter_;
    delete iq_filter_;
}

#define SQRT3 (float) std::sqrt(3)
const float FOC::Kc[2][3] = {{2.0/3, -1.0/3, -1.0/3},
                               {0,      1/SQRT3, -1/SQRT3}};

FOCStatus * const FOC::step(const FOCCommand &command) {
    status_.desired.i_d = command.desired.i_d;
    status_.desired.i_q = command.desired.i_q;
    float i_abc_measured[3] = {command.measured.i_a, command.measured.i_b, command.measured.i_c};
    float electrical_angle = command.measured.motor_encoder * num_poles_;

    //float sin_t = std::sin(electrical_angle);
    //float cos_t = std::cos(electrical_angle);

    Sincos sincos = sincos1(electrical_angle);
    float &sin_t = sincos.sin;
    float &cos_t = sincos.cos;
    float  i_alpha_measured = Kc[0][0] * i_abc_measured[0] +
            Kc[0][1] * i_abc_measured[1] +
            Kc[0][2] * i_abc_measured[2];
    float  i_beta_measured = Kc[1][0] * i_abc_measured[0] +
                     Kc[1][1] * i_abc_measured[1] +
                     Kc[1][2] * i_abc_measured[2];

    float i_d_measured = cos_t * i_alpha_measured - sin_t * i_beta_measured;
    float i_q_measured = sin_t * i_alpha_measured + cos_t * i_beta_measured;

    float i_d_measured_filtered = id_filter_->update(i_d_measured);
    float i_q_measured_filtered = iq_filter_->update(i_q_measured);

    float v_d_desired = i_gain_*pi_id_->step(status_.desired.i_d, i_d_measured_filtered);
    float v_q_desired = i_gain_*pi_iq_->step(status_.desired.i_q, i_q_measured_filtered) + command.desired.v_q;

    float v_alpha_desired = cos_t * v_d_desired + sin_t * v_q_desired;
    float v_beta_desired = -sin_t * v_d_desired + cos_t * v_q_desired;

    float v_a_desired = Kc[0][0] * v_alpha_desired + Kc[1][0] * v_beta_desired;
    float v_b_desired = Kc[0][1] * v_alpha_desired + Kc[1][1] * v_beta_desired;
    float v_c_desired = Kc[0][2] * v_alpha_desired + Kc[1][2] * v_beta_desired;

    status_.command.v_a = v_a_desired;
    status_.command.v_b = v_b_desired;
    status_.command.v_c = v_c_desired;
    status_.command.v_d = v_d_desired;
    status_.command.v_q = v_q_desired;
    status_.measured.i_d = i_d_measured;
    status_.measured.i_q = i_q_measured;
    status_.measured.i_0 = i_abc_measured[0] + i_abc_measured[1] + i_abc_measured[2];
    return &status_;
}

void FOC::set_param(const FOCParam &param) {
    param_ = param;
    pi_id_->set_param(param.pi_d);
    pi_iq_->set_param(param.pi_q);
    num_poles_ = param.num_poles;
    id_filter_->set_frequency(param.current_filter_frequency_hz);
    iq_filter_->set_frequency(param.current_filter_frequency_hz);
}

void FOC::voltage_mode() { 
    i_gain_ = 0;
}

void FOC::current_mode() {
    pi_id_->initialize();
    pi_iq_->initialize();
    i_gain_ = 1;

}

void FOC::calculate_vdq0(Vdq0 *const vdq0, float cos, float sin, float va, float vb, float vc) {
    float  v_alpha = Kc[0][0] * va +
            Kc[0][1] * vb +
            Kc[0][2] * vc;
    float  v_beta = Kc[1][0] * va +
                     Kc[1][1] * vb +
                     Kc[1][2] * vc;

    vdq0->vd = cos * v_alpha - sin * v_beta;
    vdq0->vq = sin * v_alpha + cos * v_beta;
    vdq0->v0 = (1.0/3)*(va + vb + vc);
}