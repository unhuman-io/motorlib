#ifndef HRPWM_H
#define HRPWM_H

#include "../pwm.h"
#include <cstdint>
#include "../../../st_device.h"

class HRPWM final : public PWM {
 public:
    HRPWM(uint32_t frequency_hz, HRTIM_TypeDef &regs, uint8_t ch_a, uint8_t ch_b, uint8_t ch_c, bool pwm3_mode = false) : 
         regs_(regs),
         pwm_a_(regs.sTimerxRegs[ch_a].CMP1xR), 
         pwm_b_(regs.sTimerxRegs[ch_b].CMP1xR), 
         pwm_c_(regs.sTimerxRegs[ch_c].CMP1xR), 
         ch_a_(ch_a), ch_b_(ch_b), ch_c_(ch_c),
         pwm3_mode_(pwm3_mode) {
      set_frequency_hz(frequency_hz);
      set_vbus(12);
      if (pwm3_mode) {
         // nothing to be done
      } else {
         uint32_t deadtime = 10;
         uint32_t deadprescale = 0;
         HRTIM1->sTimerxRegs[ch_a].OUTxR |= HRTIM_OUTR_DTEN;
         HRTIM1->sTimerxRegs[ch_a].DTxR |= (deadtime << HRTIM_DTR_DTF_Pos) | (deadtime << HRTIM_DTR_DTR_Pos) | (deadprescale << HRTIM_DTR_DTPRSC_Pos);
         HRTIM1->sTimerxRegs[ch_b].OUTxR |= HRTIM_OUTR_DTEN;
         HRTIM1->sTimerxRegs[ch_b].DTxR |= (deadtime << HRTIM_DTR_DTF_Pos) | (deadtime << HRTIM_DTR_DTR_Pos) | (deadprescale << HRTIM_DTR_DTPRSC_Pos);
         HRTIM1->sTimerxRegs[ch_c].OUTxR |= HRTIM_OUTR_DTEN;
         HRTIM1->sTimerxRegs[ch_c].DTxR |= (deadtime << HRTIM_DTR_DTF_Pos) | (deadtime << HRTIM_DTR_DTR_Pos) | (deadprescale << HRTIM_DTR_DTPRSC_Pos);
      }
      regs.sTimerxRegs[ch_a].TIMxCR |= HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU;
      regs.sTimerxRegs[ch_b].TIMxCR |= HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU;
      regs.sTimerxRegs[ch_c].TIMxCR |= HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU;
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
};

#endif
