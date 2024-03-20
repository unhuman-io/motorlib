
#include "../../motorlib/system.h"

void ADC5_IRQHandler(void) __attribute__((section (".ccmram")));

#define INTERRUPT_PROFILE_GLOBALS(loop) uint32_t t_exec_##loop __attribute__((used));\
                                        uint32_t t_period_##loop __attribute__((used));\
                                        uint32_t loop##_count __attribute__((used)) = 0;
#define INTERRUPT_PROFILE_START static uint32_t last_start = 0; \
                                      uint32_t t_start = get_clock();
#define INTERRUPT_PROFILE_END(loop) t_exec_##loop = get_clock()-t_start; \
                                      t_period_##loop = t_start - last_start; \
                                      loop##_count += t_exec_##loop; \
                                      last_start = t_start;

#ifdef SCOPE_DEBUG
#define SET_SCOPE_PIN(X,x) GPIO##X->BSRR = 1 << x
#define CLEAR_SCOPE_PIN(X,x) GPIO##X->BSRR = 1 << (16 + x)
#else
#define SET_SCOPE_PIN(X,x)
#define CLEAR_SCOPE_PIN(X,x)
#endif
                                    

#include "../../motorlib/util.h"
INTERRUPT_PROFILE_GLOBALS(fastloop);
INTERRUPT_PROFILE_GLOBALS(mainloop);
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
  SET_SCOPE_PIN(C,2);
  INTERRUPT_PROFILE_START;
  usb_interrupt();
  INTERRUPT_PROFILE_END(comint);
  CLEAR_SCOPE_PIN(C,2); 
}

void TIM1_UP_TIM16_IRQHandler(void)
{
  SET_SCOPE_PIN(C,0);
  INTERRUPT_PROFILE_START;
  main_loop_interrupt();
  TIM1->SR = 0;
  INTERRUPT_PROFILE_END(mainloop);
  CLEAR_SCOPE_PIN(C,0); 
}

void ADC5_IRQHandler(void)
{
  SET_SCOPE_PIN(C,1);
  INTERRUPT_PROFILE_START;
  fast_loop_interrupt();
  ADC5->ISR = ADC_ISR_JEOS;
  INTERRUPT_PROFILE_END(fastloop)
  CLEAR_SCOPE_PIN(C,1);
}

void HRTIM1_Master_IRQHandler(void)
{
  SET_SCOPE_PIN(C,0);
  INTERRUPT_PROFILE_START;
  main_loop_interrupt();
  HRTIM1->sMasterRegs.MICR = HRTIM_MICR_MCMP1;
  INTERRUPT_PROFILE_END(mainloop);
  CLEAR_SCOPE_PIN(C,0); 
}
