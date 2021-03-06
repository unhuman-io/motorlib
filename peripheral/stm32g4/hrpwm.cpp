
#include "hrpwm.h"
#include "../../control_fun.h"

void HRPWM::set_voltage(float v_abc[3]) {
    pwm_a_ = fsat2(v_abc[0] * v_to_pwm_ + half_period_, pwm_min_, pwm_max_);
    pwm_b_ = fsat2(v_abc[1] * v_to_pwm_ + half_period_, pwm_min_, pwm_max_);
    pwm_c_ = fsat2(v_abc[2] * v_to_pwm_ + half_period_, pwm_min_, pwm_max_);
}

void HRPWM::set_vbus(float vbus) {
    v_to_pwm_ = period_/vbus;
}

void HRPWM::open_mode() {
    regs_.sCommonRegs.ODISR = 0xFFF;
}
void HRPWM::brake_mode() {
    regs_.sCommonRegs.ODISR = 0x555;
    regs_.sCommonRegs.OENR = 0xAAA;
    regs_.sCommonRegs.CR1 = 0x7F; // disable updates
    regs_.sTimerxRegs[ch_a_].SETx1R = HRTIM_SET1R_SST; // set active state for some reason
    regs_.sTimerxRegs[ch_b_].SETx1R = HRTIM_SET1R_SST;
    regs_.sTimerxRegs[ch_c_].SETx1R = HRTIM_SET1R_SST;
    regs_.sTimerxRegs[ch_a_].RSTx1R = HRTIM_RST1R_SRT; // set inactive state
    regs_.sTimerxRegs[ch_b_].RSTx1R = HRTIM_RST1R_SRT;
    regs_.sTimerxRegs[ch_c_].RSTx1R = HRTIM_RST1R_SRT;
    regs_.sCommonRegs.CR1 = 0x0; // enable updates
    regs_.sTimerxRegs[ch_a_].RSTx1R = HRTIM_RST1R_SRT; // set inactive state again for some reason
    regs_.sTimerxRegs[ch_b_].RSTx1R = HRTIM_RST1R_SRT;
    regs_.sTimerxRegs[ch_c_].RSTx1R = HRTIM_RST1R_SRT;
}

void HRPWM::voltage_mode() {
    regs_.sCommonRegs.CR1 = 0x7F; // disable updates
    regs_.sTimerxRegs[ch_a_].SETx1R = HRTIM_SET1R_CMP1; // set to active on cmp1
    regs_.sTimerxRegs[ch_b_].SETx1R = HRTIM_SET1R_CMP1;
    regs_.sTimerxRegs[ch_c_].SETx1R = HRTIM_SET1R_CMP1;
    regs_.sCommonRegs.CR1 = 0x0; // enable updates
    regs_.sCommonRegs.OENR = 0xFFF;
}

// todo doesn't work at startup before regs exist
void HRPWM::set_frequency_hz(uint32_t frequency_hz, uint16_t min_off_ns, uint16_t min_on_ns) {
    period_ = (double) CPU_FREQUENCY_HZ*32/2/frequency_hz;
    regs_.sTimerxRegs[ch_a_].PERxR = period_;
    regs_.sTimerxRegs[ch_b_].PERxR = period_;
    regs_.sTimerxRegs[ch_c_].PERxR = period_;
    half_period_ = period_/2; 
    pwm_max_ = period_ - fmaxf(2*min_on_ns*count_per_ns_, 65); // seems to require at least 64 to not glitch and go high when should be low
    pwm_min_ = 2*min_off_ns*count_per_ns_;
}