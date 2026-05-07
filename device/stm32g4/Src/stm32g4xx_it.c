
#include "system.h"
#include "messages.h"
#include <stdatomic.h>
#include <string.h>

void ADC5_IRQHandler(void) __attribute__((section (".ccmram")));
void HRTIM1_Master_IRQHandler(void) __attribute__((section (".ccmram")));
void TIM1_UP_TIM16_IRQHandler(void) __attribute__((section (".ccmram")));
void USB_LP_IRQHandler(void) __attribute__((section (".ccmram")));

void update_stats(CycleStats* s, uint32_t current_cycles) {
  if (current_cycles > s->max) s->max = current_cycles;
  if (current_cycles < s->min) s->min = current_cycles;

  s->total_sum += current_cycles;
  s->count++;
}

#define GET_EXEC_STATS(loop)                                                        \
  CycleStats get_##loop##_stats() {                                                 \
    static CycleStats* spare_buffer = &loop##_stats_buffer_B;                       \
    CycleStats* filled_stats = atomic_exchange(&loop##_active_stats, spare_buffer); \
    CycleStats c = *filled_stats;                                                   \
    memset(filled_stats, 0, sizeof(CycleStats));                                    \
    filled_stats->min = 0xFFFFFFFF;                                                 \
    spare_buffer = filled_stats;                                                    \
    return c;                                                                       \
  }

#define INTERRUPT_PROFILE_GLOBALS(loop)                                     \
  uint32_t t_exec_##loop __attribute__((used));                             \
  uint32_t t_period_##loop __attribute__((used));                           \
  uint32_t loop##_count __attribute__((used)) = 0;                          \
  static CycleStats loop##_stats_buffer_A = {.min=0xFFFFFFFF};              \
  static CycleStats loop##_stats_buffer_B = {.min=0xFFFFFFFF};              \
  static _Atomic(CycleStats*) loop##_active_stats = &loop##_stats_buffer_A; \
  GET_EXEC_STATS(loop)
#define INTERRUPT_PROFILE_START   \
  static uint32_t last_start = 0; \
  uint32_t t_start = get_clock();
#define INTERRUPT_PROFILE_END(loop)                                                 \
  t_exec_##loop = get_clock() - t_start;                                            \
  t_period_##loop = t_start - last_start;                                           \
  loop##_count += t_exec_##loop;                                                    \
  last_start = t_start;                                                             \
  CycleStats* s = atomic_load_explicit(&loop##_active_stats, memory_order_relaxed); \
  update_stats(s, t_exec_##loop);


#ifdef SCOPE_DEBUG
#define SET_SCOPE_PIN(X,x) GPIO##X->BSRR = 1 << x
#define CLEAR_SCOPE_PIN(X,x) GPIO##X->BSRR = 1 << (16 + x)
#else
#define SET_SCOPE_PIN(X,x)
#define CLEAR_SCOPE_PIN(X,x)
#endif
                                    

#include "util.h"
INTERRUPT_PROFILE_GLOBALS(fastloop);
INTERRUPT_PROFILE_GLOBALS(mainloop);
INTERRUPT_PROFILE_GLOBALS(systemloop);
INTERRUPT_PROFILE_GLOBALS(comint);

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{

  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

// void PendSV_Handler(void)
// {
// }

void SysTick_Handler(void)
{
}

void USB_LP_IRQHandler(void)
{
  asm("":::"memory");
  SET_SCOPE_PIN(C,2);
  INTERRUPT_PROFILE_START;
  usb_interrupt();
  INTERRUPT_PROFILE_END(comint);
  CLEAR_SCOPE_PIN(C,2); 
}

void TIM1_CC_IRQHandler(void)
{
  asm("":::"memory");
  SET_SCOPE_PIN(A,0);
  INTERRUPT_PROFILE_START;
  system_loop_interrupt();
  TIM1->SR = 0;
  INTERRUPT_PROFILE_END(systemloop);
  CLEAR_SCOPE_PIN(A,0); 
}

void ADC5_IRQHandler(void)
{
  asm("":::"memory");
  SET_SCOPE_PIN(C,1);
  INTERRUPT_PROFILE_START;
  fast_loop_interrupt();
  ADC5->ISR = ADC_ISR_JEOS;
  INTERRUPT_PROFILE_END(fastloop)
  CLEAR_SCOPE_PIN(C,1);
}

void HRTIM1_Master_IRQHandler(void)
{
  asm("":::"memory");
  SET_SCOPE_PIN(C,0);
  INTERRUPT_PROFILE_START;
  main_loop_interrupt();
  HRTIM1->sMasterRegs.MICR = HRTIM_MICR_MCMP1;
  INTERRUPT_PROFILE_END(mainloop);
  CLEAR_SCOPE_PIN(C,0); 
}
