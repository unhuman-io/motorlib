#include "bootloader.h"
#include "../st_device.h"
#include "stm32f4xx_hal.h" // for HSE_VALUE only
#include <cmath>

// Find the best value of hsi_trim
// input hse_freq in MHz
uint8_t calibrate_hsi(uint8_t hse_freq) {
    // hsi frequency is supposed to be 16 MHz
    // Switch to hsi for sysclk, connect hse_rtc to tim11 for counting
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    RCC->CFGR &= ~(RCC_CFGR_SW | RCC_CFGR_PPRE2);      // HSI at 16MHz for sysclk, APB2 at 16MHz for TIM11
    RCC->CFGR |= (uint32_t) hse_freq << RCC_CFGR_RTCPRE_Pos; // HSE_RTC at 1 MHz

    TIM11->OR = 2;
    TIM11->CR1 |= TIM_CR1_CEN;
    TIM11->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1PSC; // input capture IC1, max prescaler, 8
    TIM11->CCER |= TIM_CCER_CC1E;     // enable input capture

    int16_t min_error = INT16_MAX;
    uint8_t min_hsi_trim = 0x10;
    for (uint32_t i=0; i<=0x1F; i++) {
      uint32_t hsi_trim = i << RCC_CR_HSITRIM_Pos;
      RCC->CR &= ~RCC_CR_HSITRIM;
      RCC->CR |= hsi_trim;

      uint16_t start_count = TIM11->CCR1;
      for (int j=0; j<10; j++) {
        while(!(TIM11->SR & TIM_SR_CC1IF)); // wait for capture
        TIM11->CCR1; // read resets the interrupt flag
      } 
      // The ideal total count is 10*8*16
      uint16_t total_count = TIM11->CCR1 - start_count;
      int16_t error = 10*8*16 - (int16_t) total_count;
      if (std::abs(error) < std::abs(min_error)) {
        min_hsi_trim = i;
        min_error = error;
      }
    }
    return min_hsi_trim;
}

// Call bootloader, trigger is go_to_bootloader==1 on reboot + software reset
uint8_t go_to_bootloader = false;
uint32_t hsi_trim = 0x10l << RCC_CR_HSITRIM_Pos;
void reboot_to_bootloader() {
  __disable_irq();
  go_to_bootloader = true;
  hsi_trim = (uint32_t) calibrate_hsi(HSE_VALUE/1e6) << RCC_CR_HSITRIM_Pos;
  NVIC_SystemReset();
}