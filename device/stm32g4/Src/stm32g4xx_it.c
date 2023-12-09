#include "../../stm_common/stm_profile.h"
#include "../../motorlib/system.h"


#define SPI_DEBUG_PIN2_SET()   do{GPIOC->ODR |= GPIO_ODR_OD12;}while(0)
#define SPI_DEBUG_PIN2_CLEAR() do{GPIOC->ODR &= ~GPIO_ODR_OD12;}while(0)

#define SPI_DEBUG_PIN3_SET()   do{GPIOC->ODR |= GPIO_ODR_OD11;}while(0)
#define SPI_DEBUG_PIN3_CLEAR() do{GPIOC->ODR &= ~GPIO_ODR_OD11;}while(0)


void ADC5_IRQHandler(void) __attribute__((section (".ccmram")));



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

void PendSV_Handler(void)
{
}

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
  SPI_DEBUG_PIN3_SET();
  SET_SCOPE_PIN(C,0);
  INTERRUPT_PROFILE_START;
  main_loop_interrupt();
  TIM1->SR = 0;
  INTERRUPT_PROFILE_END(mainloop);
  CLEAR_SCOPE_PIN(C,0); 
  SPI_DEBUG_PIN3_CLEAR();
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
