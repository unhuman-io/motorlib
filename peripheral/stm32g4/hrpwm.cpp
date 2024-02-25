
#include "hrpwm.h"
#include "../../control_fun.h"
#include "../../logger.h"

void HRPWM::set_voltage(float v_abc[3]) {
    pwm_a_ = fsat2(v_abc[0] * v_to_pwm_ + half_period_, pwm_min_, pwm_max_);
    pwm_b_ = fsat2(v_abc[1] * v_to_pwm_ + half_period_, pwm_min_, pwm_max_);
    pwm_c_ = fsat2(v_abc[2] * v_to_pwm_ + half_period_, pwm_min_, pwm_max_);
}

bool HRPWM::is_voltage_saturated() const {
    return pwm_a_ == pwm_min_ || pwm_a_ == pwm_max_ ||
           pwm_b_ == pwm_min_ || pwm_b_ == pwm_max_ ||
           pwm_c_ == pwm_min_ || pwm_c_ == pwm_max_;
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
void HRPWM::set_frequency_hz(uint32_t frequency_hz, uint16_t min_off_ns, uint16_t min_on_ns, bool keep_prescaler) {
    if (!keep_prescaler) {
        int desired_prescaler = frequency_hz*2/(CPU_FREQUENCY_HZ/65536);
        int ckpsc;
        if (desired_prescaler >= 32) {
            prescaler_ = 32;
            ckpsc = 0;
        } else if (desired_prescaler >= 16) {
            prescaler_ = 16;
            ckpsc = 1;
        } else {
            prescaler_ = 8; // no slower than this
            ckpsc = 2;
        }
        regs_.sTimerxRegs[ch_a_].TIMxCR |= ckpsc;
        regs_.sTimerxRegs[ch_b_].TIMxCR |= ckpsc;
        regs_.sTimerxRegs[ch_c_].TIMxCR |= ckpsc;
        regs_.sTimerxRegs[5].TIMxCR |= ckpsc;
    }
    
    count_per_ns_ = CPU_FREQUENCY_HZ * prescaler_/ 4 / 1.e9; // Datasheet says /8 not /4, but /4 seems to give correct scale
    period_ = (double) CPU_FREQUENCY_HZ*prescaler_/2/frequency_hz;
    regs_.sTimerxRegs[ch_a_].PERxR = period_;
    regs_.sTimerxRegs[ch_b_].PERxR = period_;
    regs_.sTimerxRegs[ch_c_].PERxR = period_;
    regs_.sTimerxRegs[5].PERxR = period_;
    regs_.sTimerxRegs[5].CMP3xR = period_ - 65; // for low time adc trigger
    half_period_ = period_/2; 
    pwm_max_ = period_ - fmaxf(2*min_on_ns*count_per_ns_, 65); // seems to require at least 64 to not glitch and go high when should be low
    pwm_min_ = 2*min_off_ns*count_per_ns_;
    current_frequency_hz_ = frequency_hz;
}

void HRPWM::set_frequency_multiplier(uint8_t multiplier) {
    if (multiplier > 0x1F) {
        multiplier = 0x1F;
    }
    uint8_t multiplier_int = multiplier;
    uint32_t frequency_hz = base_frequency_hz_;

    frequency_hz *= multiplier_int + 1;
    logger.log_printf("pwm frequency: %d", frequency_hz);

    if (frequency_hz >= current_frequency_hz_) {
        HRTIM1->sCommonRegs.ADCPS1 = multiplier_int << HRTIM_ADCPS1_AD1PSC_Pos | multiplier_int << HRTIM_ADCPS1_AD2PSC_Pos;
        set_frequency_hz(frequency_hz, min_off_ns_, min_on_ns_, true);
    } else {
        set_frequency_hz(frequency_hz, min_off_ns_, min_on_ns_, true);
        HRTIM1->sCommonRegs.ADCPS1 = multiplier_int << HRTIM_ADCPS1_AD1PSC_Pos | multiplier_int << HRTIM_ADCPS1_AD2PSC_Pos;
    }
}

uint8_t HRPWM::get_frequency_multiplier() const {
    return current_frequency_hz_/base_frequency_hz_ - 1;
}

void HRPWM3::voltage_mode() {
    regs_.sCommonRegs.CR1 = 0x7F; // disable updates
    regs_.sTimerxRegs[ch_a_].SETx1R = HRTIM_SET1R_CMP1; // set to active on cmp1
    regs_.sTimerxRegs[ch_b_].SETx2R = HRTIM_SET2R_CMP2;
    regs_.sTimerxRegs[ch_c_].SETx1R = HRTIM_SET1R_CMP1;
    regs_.sCommonRegs.CR1 = 0x0; // enable updates
    regs_.sCommonRegs.OENR = 0xFFF;
}

void HRPWM3::brake_mode() {
    regs_.sCommonRegs.ODISR = 0x555;
    regs_.sCommonRegs.OENR = 0xAAA;
    regs_.sCommonRegs.CR1 = 0x7F; // disable updates
    regs_.sTimerxRegs[ch_a_].SETx1R = HRTIM_SET1R_SST; // set active state for some reason
    regs_.sTimerxRegs[ch_b_].SETx2R = HRTIM_SET2R_SST;
    regs_.sTimerxRegs[ch_c_].SETx1R = HRTIM_SET1R_SST;
    regs_.sTimerxRegs[ch_a_].RSTx1R = HRTIM_RST1R_SRT; // set inactive state
    regs_.sTimerxRegs[ch_b_].RSTx2R = HRTIM_RST2R_SRT;
    regs_.sTimerxRegs[ch_c_].RSTx1R = HRTIM_RST1R_SRT;
    regs_.sCommonRegs.CR1 = 0x0; // enable updates
    regs_.sTimerxRegs[ch_a_].RSTx1R = HRTIM_RST1R_SRT; // set inactive state again for some reason
    regs_.sTimerxRegs[ch_b_].RSTx2R = HRTIM_RST2R_SRT;
    regs_.sTimerxRegs[ch_c_].RSTx1R = HRTIM_RST1R_SRT;
}