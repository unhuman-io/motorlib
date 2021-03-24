#ifndef QEP_ENCODER_H
#define QEP_ENCODER_H

#include <cstdint>
#include "encoder.h"
#include "../st_device.h"

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
   int32_t get_index_pos() { return regs_.CCR3; }
   bool index_received() { return regs_.SR & TIM_SR_CC3IF; }
 private:
   TIM_TypeDef &regs_;
   int32_t value_ = 0;
};

#endif
