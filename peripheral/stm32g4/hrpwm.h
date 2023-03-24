#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_HRPWM_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_HRPWM_H_

#include "../pwm.h"
#include <cstdint>
#include "../../../st_device.h"
#include <vector>
#include "pin_config.h"

extern "C" { void system_init(); }

class HRPWM : public PWMBase {
 public:
    HRPWM(HRTIM_TypeDef &regs, volatile uint32_t &pwm_a, volatile uint32_t &pwm_b, volatile uint32_t &pwm_c) : 
      regs_(regs), pwm_a_(pwm_a), pwm_b_(pwm_b), pwm_c_(pwm_c) {}
    HRPWM(uint32_t frequency_hz, HRTIM_TypeDef &regs, uint8_t ch_a, uint8_t ch_b, uint8_t ch_c, 
      bool pwm3_mode = false, uint16_t deadtime_ns = 50, uint16_t min_off_ns = 0, uint16_t min_on_ns = 0) : 
         regs_(regs),
         pwm_a_(regs.sTimerxRegs[ch_a].CMP1xR), 
         pwm_b_(regs.sTimerxRegs[ch_b].CMP1xR), 
         pwm_c_(regs.sTimerxRegs[ch_c].CMP1xR), 
         ch_a_(ch_a), ch_b_(ch_b), ch_c_(ch_c),
         pwm3_mode_(pwm3_mode),
         deadtime_ns_(deadtime_ns) {
      set_frequency_hz(frequency_hz, min_off_ns, min_on_ns);
      set_vbus(12);
      init();
   }
   void init() {
      for(auto ch : std::vector<uint8_t>{ch_a_, ch_b_, ch_c_}) {
         regs_.sTimerxRegs[ch].TIMxCR2 = HRTIM_TIMCR2_UDM;
         if (pwm3_mode_) {
            regs_.sTimerxRegs[ch].SETx2R = HRTIM_SET2R_SST;
         }
         regs_.sTimerxRegs[ch].TIMxCR |= HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT;
      }
      if (!pwm3_mode_) {
         set_deadtime(deadtime_ns_);
      }
      regs_.sTimerxRegs[5].TIMxCR2 = HRTIM_TIMCR2_UDM;
      regs_.sTimerxRegs[5].TIMxCR |= HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT;
      MASK_SET(regs_.sTimerxRegs[5].TIMxCR2, HRTIM_TIMCR2_ADROM, 1);   // adc event generated at 0 on F
      regs_.sCommonRegs.DLLCR = HRTIM_DLLCR_CALEN | (3 << HRTIM_DLLCR_CALRTE_Pos); // periodic calibration at 2048*hrtim = 12us
      regs_.sCommonRegs.ADC1R = HRTIM_ADC1R_AD1TFPER; // TODO coded only to F
      regs_.sCommonRegs.ADC2R = HRTIM_ADC2R_AD2TFPER; // also hrtim trig 2
   }

   void set_voltage(float v_abc[3])  __attribute__((section (".ccmram")));
   void set_vbus(float vbus);
   void open_mode();
   void brake_mode();
   void voltage_mode();
   void set_deadtime(uint16_t deadtime_ns) {
      deadtime_ns_ = deadtime_ns;
      uint32_t deadprescale = 0;
      uint32_t deadtime = deadtime_ns_ * count_per_ns_; // 9 bits at 170e6*32/4 gives 376 ns
      for(auto ch : std::vector<uint8_t>{ch_a_, ch_b_, ch_c_}) {
         regs_.sTimerxRegs[ch].OUTxR = HRTIM_OUTR_DTEN;
         regs_.sTimerxRegs[ch].DTxR = (deadtime << HRTIM_DTR_DTF_Pos) | (deadtime << HRTIM_DTR_DTR_Pos) | (deadprescale << HRTIM_DTR_DTPRSC_Pos);
      }
   }
   void set_frequency_hz(uint32_t frequency_hz, uint16_t min_off_ns = 0, uint16_t min_on_ns = 0);

   uint16_t period_, half_period_;
   HRTIM_TypeDef &regs_;
   volatile uint32_t &pwm_a_, &pwm_b_, &pwm_c_;
   uint8_t ch_a_, ch_b_, ch_c_;
   float v_to_pwm_;
   bool pwm3_mode_;
   uint16_t deadtime_ns_;
   float pwm_min_ = 0;
   float pwm_max_;
   int prescaler_ = 32;
   float count_per_ns_;
};

class HRPWM3 : public HRPWM {
 public:
   HRPWM3(uint32_t frequency_hz, HRTIM_TypeDef &regs, uint8_t ch_a, uint8_t ch_b, uint8_t ch_c, 
      uint16_t min_off_ns = 0, uint16_t min_on_ns = 0) : 
         HRPWM(regs, regs.sTimerxRegs[ch_a].CMP1xR, regs.sTimerxRegs[ch_b].CMP2xR, regs.sTimerxRegs[ch_c].CMP1xR) {
      ch_a_ = ch_a;
      ch_b_ = ch_b;
      ch_c_ = ch_c;
      pwm3_mode_ = true;
      set_frequency_hz(frequency_hz, min_off_ns, min_on_ns);
      set_vbus(12);
      init();
   }
   void brake_mode();
   void voltage_mode();
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_HRPWM_H_
