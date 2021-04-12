#ifndef QEP_ENCODER_H
#define QEP_ENCODER_H

#include <cstdint>
#include "encoder.h"
#include "../st_device.h"
#include "control_fun.h"

class QEPEncoder final : public EncoderBase {
 public:
   QEPEncoder(TIM_TypeDef &regs) : EncoderBase(), regs_(regs) { 
     regs_.SMCR = 3;  // quadrature mode
     regs_.CCMR1 = (1 << TIM_CCMR1_CC1S_Pos) | (1 << TIM_CCMR1_CC2S_Pos) |  // input selection
        (1 << TIM_CCMR1_IC1F_Pos) | (1 << TIM_CCMR1_IC2F_Pos); // filters at 1
     regs_.CCMR2 = (1 << TIM_CCMR2_CC3S_Pos) | (2 << TIM_CCMR2_IC3F_Pos); // index filter at 2
     regs_.CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;  // enable
     regs_.CR1 |= TIM_CR1_CEN;
   }
   int32_t read() { value_ = regs_.CNT; return value_; } __attribute__((section (".ccmram")));
   int32_t get_value() const { return value_; } __attribute__((section (".ccmram")));
   void trigger() {} __attribute__((section (".ccmram")));
   int32_t get_index_pos() { check_index(); return regs_.CCR3; }
   bool index_received() { check_index(); return index_received_; }
   int32_t first_index() const { return first_index_; }
   void check_index() {
     if(!index_received_ && regs_.SR & TIM_SR_CC3IF) {
       index_received_ = true;
       first_index_ = regs_.CCR3;
     }
   }
   // TODO what happens in rollover?
   int32_t index_error(int32_t cpr) {
     int32_t diff = get_index_pos() - first_index();
     int32_t error = (abs(diff) + cpr/2) % cpr - cpr/2;
     return sign(diff) * error;
   }
   bool init() { return true; }
 private:
   TIM_TypeDef &regs_;
   int32_t value_ = 0;
   bool index_received_ = false;
   int32_t first_index_ = 0;
};

#endif
