#include "messages.h"
#include "../../interrupt_profiler.h"

extern "C" {
void main_loop_interrupt();
void fast_loop_interrupt();
void system_loop_interrupt();
void usb_interrupt();

void ADC5_IRQHandler(void) __attribute__((section (".ccmram")));
void HRTIM1_Master_IRQHandler(void) __attribute__((section (".ccmram")));
void TIM1_UP_TIM16_IRQHandler(void) __attribute__((section (".ccmram")));
void USB_LP_IRQHandler(void) __attribute__((section (".ccmram")));



#ifdef SCOPE_DEBUG
#define SET_SCOPE_PIN(X,x) GPIO##X->BSRR = 1 << x
#define CLEAR_SCOPE_PIN(X,x) GPIO##X->BSRR = 1 << (16 + x)
#else
#define SET_SCOPE_PIN(X,x)
#define CLEAR_SCOPE_PIN(X,x)
#endif
                                    

#define INTERRUPT_PROFILE_GLOBALS(loop)                                     \
  uint32_t t_exec_##loop __attribute__((used));                             \
  uint32_t t_period_##loop __attribute__((used));                           \
  uint32_t loop##_count __attribute__((used)) = 0; \
  uint32_t loop##_last_start = 0

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
  Profiler<false> p;
  p.start();
  usb_interrupt();
  p.end(active_stats.load()->comint_stats, 
          t_exec_comint, t_period_comint, comint_count, comint_last_start);
  CLEAR_SCOPE_PIN(C,2); 
}

void TIM1_CC_IRQHandler(void)
{
  asm("":::"memory");
  SET_SCOPE_PIN(A,0);
  Profiler<false> p;
  p.start();
  system_loop_interrupt();
  TIM1->SR = 0;
  p.end(active_stats.load()->systemloop_stats, 
          t_exec_systemloop, t_period_systemloop, systemloop_count, systemloop_last_start);
  CLEAR_SCOPE_PIN(A,0); 
}

void ADC5_IRQHandler(void)
{
  asm("":::"memory");
  SET_SCOPE_PIN(C,1);
  Profiler<false> p;
  p.start();
  fast_loop_interrupt();
  ADC5->ISR = ADC_ISR_JEOS;
  p.end(active_stats.load()->fastloop_stats, 
          t_exec_fastloop, t_period_fastloop, fastloop_count, fastloop_last_start);
  CLEAR_SCOPE_PIN(C,1);
}

void HRTIM1_Master_IRQHandler(void)
{
  asm("":::"memory");
  SET_SCOPE_PIN(C,0);
  Profiler<false> p;
  p.start();
  main_loop_interrupt();
  HRTIM1->sMasterRegs.MICR = HRTIM_MICR_MCMP1;
  p.end(active_stats.load()->mainloop_stats, 
        t_exec_mainloop, t_period_mainloop, mainloop_count, mainloop_last_start);
  CLEAR_SCOPE_PIN(C,0); 
}
} // extern "C"
