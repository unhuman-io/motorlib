#ifndef UNHUMAN_MOTORLIB_BUCK_FAST_LOOP_BUCK_H_
#define UNHUMAN_MOTORLIB_BUCK_FAST_LOOP_BUCK_H_

#include "buck.h"
#include "st_device.h"

#define NUM_BUCK    3
#define ADC_DR_REGS {ADC1->JDR1, ADC2->JDR1, ADC3->JDR1}
#define ADC_V_OUT_DR_REGS {ADC1->JDR1, ADC2->JDR1, ADC3->JDR1}
#define PWM_REGS {TIM8->CH1, TIM8->CH2, TIM8->CH3}
#define ADC_V_BUS_DR ADC1->JDR1
#define ADC_I_BUS_DR ADC1->JDR1

// runs NUM_BUCK regulators in parallel, regulating current
template<typename PWM>
class FastLoopBuck {
 public:
    FastLoopBuck(FastLoopBuckParam param) : param_(param) {}
    void update()  __attribute__((section (".ccmram"))) {
        float current_desired = current_desired_*(1.0f/NUM_BUCK);
        for (int i=0; i<NUM_BUCK; i++) {
            current_measured_[i] = *adc_dr_[i] * a_per_count + i_bias_[i];
            v_desired_[i] = buck_controller_[i].step(current_desired, current_measured_[i], voltage_feedforward_)
            pwm_[i].set_voltage(v_desired_[i]);
        }
    }
    void set_current_desired(float current_desired) { current_desired_ = current_desired; }
    void set_voltage_feedforward(float voltage) { voltage_feedforward_ = voltage; }
    float get_v_bus() const {
        return ADC_V_BUS_DR * param_.v_bus_v_per_count;
    }
    float get_i_bus() const {
        return ADC_I_BUS_DR * param_.i_bus_a_per_count + param_.i_bus_bias_a;
    }
    float get_v_out() const {
        float v_out = 0; 
        for (int i=0; i<NUM_BUCK; i++) {
            v_out += adc_dr_v_out_[i] * param_.v_out_v_per_count;
        }
        return v_out*(1.0f/NUM_BUCK);
    }
    float get_i_out() const {
        switch (mode_) {
            case OPEN:
                return 0;
                break;
            case CURRENT:
                float current_measured = 0; 
                for (int i=0; i<NUM_BUCK; i++) {
                    current_measured += current_measured_[i];
                }
                break;
        }
    }
    void open_mode() { mode_ = OPEN; }
    void current_mode() { mode_ = CURRENT; }
 private:
    enum {OPEN, CURRENT} mode_ = OPEN;
    float a_per_count = .01;
    float voltage_feedforward_ = 10;
    float *adc_dr_[NUM_BUCK] = ADC_DR_REGS;
    float *adc_dr_v_out_[NUM_BUCK] = ADC_V_OUT_DR_REGS;
    float current_measured_[NUM_BUCK] = {};
    PWM pwm_[NUM_BUCK] = PWM_REGS;
    BuckController buck_controller_[NUM_BUCK];
    float v_desired_[NUM_BUCK] = {};
    float current_desired_ = 0;
    FastLoopBuckParam param_;
};

#endif  // UNHUMAN_MOTORLIB_BUCK_FAST_LOOP_BUCK_H_
