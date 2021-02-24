
#include "pwm_en.h"
#include "stm32f446xx.h"
#include "../../gpio.h"

void PWM_EN::set_voltage(float v_abc[3]) {
    pwm_a_ = v_abc[0] * v_to_pwm_ + half_period_;
    pwm_b_ = v_abc[1] * v_to_pwm_ + half_period_;
    pwm_c_ = v_abc[2] * v_to_pwm_ + half_period_;
}

void PWM_EN::set_vbus(float vbus) {
    v_to_pwm_ = period_/vbus;
}

void PWM_EN::open_mode() {
    enable_.clear();
}

void PWM_EN::brake_mode() {
    enable_.set();
    // CC4E is the driving the interrupt, keep enabled, disable others
    regs_.CCER = TIM_CCER_CC4E;
}

void PWM_EN::voltage_mode() {
    enable_.set();
    regs_.CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    regs_.BDTR |= TIM_BDTR_MOE;
}

void PWM_EN::set_frequency_hz(uint32_t frequency_hz) {
    regs_.ARR = 180e6/2/frequency_hz; // todo not enabled at startup
    period_ = 180e6/2/frequency_hz;
    half_period_ = period_/2; 
}