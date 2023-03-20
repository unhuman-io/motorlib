
#include "hrpwm.h"

#include "../../control_fun.h"
#include "../../logger.h"
#include "pin_config.h"

HRPWM::HRPWM(HRTIM_TypeDef &regs, volatile uint32_t &pwm_a,
             volatile uint32_t &pwm_b, volatile uint32_t &pwm_c)
    : regs_(regs), pwm_a_(pwm_a), pwm_b_(pwm_b), pwm_c_(pwm_c) {}

HRPWM::HRPWM(uint32_t frequency_hz, HRTIM_TypeDef &regs, uint8_t ch_a,
             uint8_t ch_b, uint8_t ch_c, bool pwm3_mode, uint16_t deadtime_ns,
             uint16_t min_off_ns, uint16_t min_on_ns)
    : regs_(regs),
      pwm_a_(regs.sTimerxRegs[ch_a].CMP1xR),
      pwm_b_(regs.sTimerxRegs[ch_b].CMP1xR),
      pwm_c_(regs.sTimerxRegs[ch_c].CMP1xR),
      ch_a_(ch_a),
      ch_b_(ch_b),
      ch_c_(ch_c),
      pwm3_mode_(pwm3_mode),
      deadtime_ns_(deadtime_ns) {
  base_frequency_hz_ = frequency_hz;
  min_off_ns_ = min_off_ns;
  min_on_ns_ = min_on_ns;
  set_frequency_hz(frequency_hz, min_off_ns, min_on_ns,
                   /*keep_prescaler=*/false);
  set_vbus(12);
  init();
}

void HRPWM::init() {
  for (auto ch : std::vector<uint8_t>{ch_a_, ch_b_, ch_c_}) {
    regs_.sTimerxRegs[ch].TIMxCR2 = HRTIM_TIMCR2_UDM;
    if (pwm3_mode_) {
      regs_.sTimerxRegs[ch].SETx2R = HRTIM_SET2R_SST;
    }
    regs_.sTimerxRegs[ch].TIMxCR |=
        HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT;
  }
  if (!pwm3_mode_) {
    set_deadtime(deadtime_ns_);
  }
  regs_.sTimerxRegs[5].TIMxCR2 = HRTIM_TIMCR2_UDM;
  regs_.sTimerxRegs[5].TIMxCR |=
      HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT;
  MASK_SET(regs_.sTimerxRegs[5].TIMxCR2, HRTIM_TIMCR2_ADROM,
           1);  // adc event generated at 0 on F
  regs_.sCommonRegs.DLLCR =
      HRTIM_DLLCR_CALEN |
      (3 << HRTIM_DLLCR_CALRTE_Pos);  // periodic calibration at 2048*hrtim =
                                      // 12us
  regs_.sCommonRegs.ADC1R = HRTIM_ADC1R_AD1TFPER;  // TODO coded only to F
  regs_.sCommonRegs.ADC2R = HRTIM_ADC2R_AD2TFPER;  // also hrtim trig 2
}

void HRPWM::set_deadtime(uint16_t deadtime_ns) {
  deadtime_ns_ = deadtime_ns;
  uint32_t deadprescale = 0;
  uint32_t deadtime =
      deadtime_ns_ * count_per_ns_;  // 9 bits at 170e6*32/4 gives 376 ns
  for (auto ch : std::vector<uint8_t>{ch_a_, ch_b_, ch_c_}) {
    regs_.sTimerxRegs[ch].OUTxR = HRTIM_OUTR_DTEN;
    regs_.sTimerxRegs[ch].DTxR = (deadtime << HRTIM_DTR_DTF_Pos) |
                                 (deadtime << HRTIM_DTR_DTR_Pos) |
                                 (deadprescale << HRTIM_DTR_DTPRSC_Pos);
  }
}

void HRPWM::set_voltage(float v_abc[3]) {
  pwm_a_ = fsat2(v_abc[0] * v_to_pwm_ + half_period_, pwm_min_, pwm_max_);
  pwm_b_ = fsat2(v_abc[1] * v_to_pwm_ + half_period_, pwm_min_, pwm_max_);
  pwm_c_ = fsat2(v_abc[2] * v_to_pwm_ + half_period_, pwm_min_, pwm_max_);
}

void HRPWM::set_vbus(float vbus) { v_to_pwm_ = period_ / vbus; }

void HRPWM::open_mode() { regs_.sCommonRegs.ODISR = 0xFFF; }
void HRPWM::brake_mode() {
  regs_.sCommonRegs.ODISR = 0x555;
  regs_.sCommonRegs.OENR = 0xAAA;
  regs_.sCommonRegs.CR1 = 0x7F;  // disable updates
  regs_.sTimerxRegs[ch_a_].SETx1R =
      HRTIM_SET1R_SST;  // set active state for some reason
  regs_.sTimerxRegs[ch_b_].SETx1R = HRTIM_SET1R_SST;
  regs_.sTimerxRegs[ch_c_].SETx1R = HRTIM_SET1R_SST;
  regs_.sTimerxRegs[ch_a_].RSTx1R = HRTIM_RST1R_SRT;  // set inactive state
  regs_.sTimerxRegs[ch_b_].RSTx1R = HRTIM_RST1R_SRT;
  regs_.sTimerxRegs[ch_c_].RSTx1R = HRTIM_RST1R_SRT;
  regs_.sCommonRegs.CR1 = 0x0;  // enable updates
  regs_.sTimerxRegs[ch_a_].RSTx1R =
      HRTIM_RST1R_SRT;  // set inactive state again for some reason
  regs_.sTimerxRegs[ch_b_].RSTx1R = HRTIM_RST1R_SRT;
  regs_.sTimerxRegs[ch_c_].RSTx1R = HRTIM_RST1R_SRT;
}

void HRPWM::voltage_mode() {
  regs_.sCommonRegs.CR1 = 0x7F;                        // disable updates
  regs_.sTimerxRegs[ch_a_].SETx1R = HRTIM_SET1R_CMP1;  // set to active on cmp1
  regs_.sTimerxRegs[ch_b_].SETx1R = HRTIM_SET1R_CMP1;
  regs_.sTimerxRegs[ch_c_].SETx1R = HRTIM_SET1R_CMP1;
  regs_.sCommonRegs.CR1 = 0x0;  // enable updates
  regs_.sCommonRegs.OENR = 0xFFF;
}

// TODO: Doesn't work at startup before regs exist.
void HRPWM::set_frequency_hz(uint32_t frequency_hz, uint16_t min_off_ns,
                             uint16_t min_on_ns, bool keep_prescaler) {
  if (!keep_prescaler) {
    int desired_prescaler = frequency_hz * 2 / (CPU_FREQUENCY_HZ / 65536);
    int ckpsc;
    if (desired_prescaler >= 32) {
      prescaler_ = 32;
      ckpsc = 0;
    } else if (desired_prescaler >= 16) {
      prescaler_ = 16;
      ckpsc = 1;
    } else {
      prescaler_ = 8;  // no slower than this
      ckpsc = 2;
    }
    regs_.sTimerxRegs[ch_a_].TIMxCR |= ckpsc;
    regs_.sTimerxRegs[ch_b_].TIMxCR |= ckpsc;
    regs_.sTimerxRegs[ch_c_].TIMxCR |= ckpsc;
    regs_.sTimerxRegs[5].TIMxCR |= ckpsc;
  }

  count_per_ns_ =
      CPU_FREQUENCY_HZ * prescaler_ / 4 /
      1.e9;  // Datasheet says /8 not /4, but /4 seems to give correct scale
  period_ = (double)CPU_FREQUENCY_HZ * prescaler_ / 2 / frequency_hz;
  regs_.sTimerxRegs[ch_a_].PERxR = period_;
  regs_.sTimerxRegs[ch_b_].PERxR = period_;
  regs_.sTimerxRegs[ch_c_].PERxR = period_;
  regs_.sTimerxRegs[5].PERxR = period_;
  half_period_ = period_ / 2;
  pwm_max_ = period_ - fmaxf(2 * min_on_ns * count_per_ns_,
                             65);  // seems to require at least 64 to not glitch
                                   // and go high when should be low
  pwm_min_ = 2 * min_off_ns * count_per_ns_;
  current_frequency_hz_ = frequency_hz;
}

void HRPWM::set_frequency_hz(uint32_t frequency_hz) {
  set_frequency_hz(frequency_hz, /*min_off_ns=*/0, /*min_on_ns=*/0,
                   /*keep_prescaler=*/false);
}

void HRPWM::set_frequency_multiplier(uint8_t multiplier) {
  uint8_t multiplier_int = multiplier;
  HRTIM1->sCommonRegs.ADCPS1 = multiplier_int << HRTIM_ADCPS1_AD1PSC_Pos |
                               multiplier_int << HRTIM_ADCPS1_AD2PSC_Pos;
  uint32_t frequency_hz = base_frequency_hz_;

  frequency_hz *= multiplier_int + 1;
  logger.log_printf("pwm frequency: %d", frequency_hz);
  set_frequency_hz(frequency_hz, min_off_ns_, min_on_ns_, true);
}

uint8_t HRPWM::get_frequency_multiplier() const {
  return current_frequency_hz_ / base_frequency_hz_;
}

HRPWM3::HRPWM3(uint32_t frequency_hz, HRTIM_TypeDef &regs, uint8_t ch_a,
               uint8_t ch_b, uint8_t ch_c, uint16_t min_off_ns,
               uint16_t min_on_ns)
    : HRPWM(regs, regs.sTimerxRegs[ch_a].CMP1xR, regs.sTimerxRegs[ch_b].CMP2xR,
            regs.sTimerxRegs[ch_c].CMP1xR) {
  ch_a_ = ch_a;
  ch_b_ = ch_b;
  ch_c_ = ch_c;
  pwm3_mode_ = true;
  set_frequency_hz(frequency_hz, min_off_ns, min_on_ns,
                   /*keep_prescaler=*/false);
  set_vbus(12);
  init();
}

void HRPWM3::voltage_mode() {
  regs_.sCommonRegs.CR1 = 0x7F;                        // disable updates
  regs_.sTimerxRegs[ch_a_].SETx1R = HRTIM_SET1R_CMP1;  // set to active on cmp1
  regs_.sTimerxRegs[ch_b_].SETx2R = HRTIM_SET2R_CMP2;
  regs_.sTimerxRegs[ch_c_].SETx1R = HRTIM_SET1R_CMP1;
  regs_.sCommonRegs.CR1 = 0x0;  // enable updates
  regs_.sCommonRegs.OENR = 0xFFF;
}

void HRPWM3::brake_mode() {
  regs_.sCommonRegs.ODISR = 0x555;
  regs_.sCommonRegs.OENR = 0xAAA;
  regs_.sCommonRegs.CR1 = 0x7F;  // disable updates
  regs_.sTimerxRegs[ch_a_].SETx1R =
      HRTIM_SET1R_SST;  // set active state for some reason
  regs_.sTimerxRegs[ch_b_].SETx2R = HRTIM_SET2R_SST;
  regs_.sTimerxRegs[ch_c_].SETx1R = HRTIM_SET1R_SST;
  regs_.sTimerxRegs[ch_a_].RSTx1R = HRTIM_RST1R_SRT;  // set inactive state
  regs_.sTimerxRegs[ch_b_].RSTx2R = HRTIM_RST2R_SRT;
  regs_.sTimerxRegs[ch_c_].RSTx1R = HRTIM_RST1R_SRT;
  regs_.sCommonRegs.CR1 = 0x0;  // enable updates
  regs_.sTimerxRegs[ch_a_].RSTx1R =
      HRTIM_RST1R_SRT;  // set inactive state again for some reason
  regs_.sTimerxRegs[ch_b_].RSTx2R = HRTIM_RST2R_SRT;
  regs_.sTimerxRegs[ch_c_].RSTx1R = HRTIM_RST1R_SRT;
}
