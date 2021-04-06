#ifndef HRPWM_H
#define HRPWM_H

#include "../pwm.h"
#include <cstdint>
#include "../../../st_device.h"
#include <vector>
#include "pin_config.h"

class HRPWM final : public PWMBase {
 public:
    HRPWM(uint32_t frequency_hz, HRTIM_TypeDef &regs, uint8_t ch_a, uint8_t ch_b, uint8_t ch_c, bool pwm3_mode = false, uint16_t deadtime_ns = 50) : 
         regs_(regs),
         pwm_a_(regs.sTimerxRegs[ch_a].CMP1xR), 
         pwm_b_(regs.sTimerxRegs[ch_b].CMP1xR), 
         pwm_c_(regs.sTimerxRegs[ch_c].CMP1xR), 
         ch_a_(ch_a), ch_b_(ch_b), ch_c_(ch_c),
         pwm3_mode_(pwm3_mode),
         deadtime_ns_(deadtime_ns) {
      set_frequency_hz(frequency_hz);
      set_vbus(12);
      init();
   }
   void init() {
      for(auto ch : std::vector<uint8_t>{ch_a_, ch_b_, ch_c_}) {
         regs_.sTimerxRegs[ch].TIMxCR2 = HRTIM_TIMCR2_UDM;
         if (pwm3_mode_) {
            regs_.sTimerxRegs[ch].SETx2R = HRTIM_SET2R_SST;
         } else {
            uint32_t deadprescale = 0;
            uint32_t deadtime = deadtime_ns_ * CPU_FREQUENCY_HZ * 32 / 4 / 1.e9; // Datasheet says /8 not /4, but /4 seems to give correct scale
                                                                                 // 9 bits at 170e6*32/4 gives 376 ns
            regs_.sTimerxRegs[ch].OUTxR = HRTIM_OUTR_DTEN;
            regs_.sTimerxRegs[ch].DTxR = (deadtime << HRTIM_DTR_DTF_Pos) | (deadtime << HRTIM_DTR_DTR_Pos) | (deadprescale << HRTIM_DTR_DTPRSC_Pos);
         }
         regs_.sTimerxRegs[ch].TIMxCR = HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT;
      }
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
   void set_frequency_hz(uint32_t frequency_hz);
 private:
   uint16_t period_, half_period_;
   HRTIM_TypeDef &regs_;
   volatile uint32_t &pwm_a_, &pwm_b_, &pwm_c_;
   uint8_t ch_a_, ch_b_, ch_c_;
   float v_to_pwm_;
   bool pwm3_mode_;
   uint16_t deadtime_ns_;
};

#endif
