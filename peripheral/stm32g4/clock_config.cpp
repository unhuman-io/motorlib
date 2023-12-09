#include "stm32g474xx.h"
#include "../../util.h"

volatile uint32_t * const cpu_clock = &DWT->CYCCNT;

extern "C" void SystemClock_Config(void)
{
    PWR->CR5 &= ~PWR_CR5_R1MODE; // R1MODE -> 0 for > 150 MHz operation

    // ensure cpu clock is started for us_delay
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // PWR_CR5_R1MODE change recommends 1 us startup
    us_delay(1);
#ifdef USE_HSI
    RCC->PLLCFGR = 2 << RCC_PLLCFGR_PLLSRC_Pos | // (2) HSI is pll source (16 MHz)
      3 << RCC_PLLCFGR_PLLM_Pos | // (3) div4 
      85 << RCC_PLLCFGR_PLLN_Pos | // (85) x85
      2 << RCC_PLLCFGR_PLLPDIV_Pos | // (2) div2
      //RCC_PLLCFGR_PLLPEN |
      0 << RCC_PLLCFGR_PLLQ_Pos | // (0) div2
      //RCC_PLLCFGR_PLLQEN | 
      0 << RCC_PLLCFGR_PLLR_Pos | // (0) div2
      RCC_PLLCFGR_PLLREN;
    RCC->CR = RCC_CR_HSION | RCC_CR_PLLON;
#else
  // P, Q, R all 170 MHz
  RCC->PLLCFGR = 3 << RCC_PLLCFGR_PLLSRC_Pos | // (3) HSE is pll source (24 MHz)
    5 << RCC_PLLCFGR_PLLM_Pos | // (5) div6 
    85 << RCC_PLLCFGR_PLLN_Pos | // (85) x85
    2 << RCC_PLLCFGR_PLLPDIV_Pos | // (2) div2
    //RCC_PLLCFGR_PLLPEN |
    0 << RCC_PLLCFGR_PLLQ_Pos | // (0) div2
    //RCC_PLLCFGR_PLLQEN | 
    0 << RCC_PLLCFGR_PLLR_Pos | // (0) div2
    RCC_PLLCFGR_PLLREN;  
    RCC->CR = RCC_CR_HSEON | RCC_CR_HSION | RCC_CR_PLLON;
#endif

  RCC->CRRCR = RCC_CRRCR_HSI48ON;  
  while(!(RCC->CR & RCC_CR_PLLRDY));

  FLASH->ACR |= 4 << FLASH_ACR_LATENCY_Pos; // flash latency at least 4
  

  RCC->CFGR = 3 << RCC_CFGR_SW_Pos; // (3) // PLL clock

  RCC->CCIPR = 0 << RCC_CCIPR_CLK48SEL_Pos | // HSI48 (0) for usb
    2 << RCC_CCIPR_ADC12SEL_Pos | 2 << RCC_CCIPR_ADC345SEL_Pos | // (2) sysclk
    0 << RCC_CCIPR_I2C1SEL_Pos | 0 << RCC_CCIPR_I2C2SEL_Pos; // (0) pclk


  RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN;
  RCC->APB1SMENR1 |= RCC_APB1SMENR1_CRSSMEN;
  CRS->CFGR = 2 << CRS_CFGR_SYNCSRC_Pos | 34 << CRS_CFGR_FELIM_Pos |
    (48000000/1000 - 1) << CRS_CFGR_RELOAD_Pos; // DIV1, source usb sof (2), polarity rising, 34 felim was specificed by cubemx, reload (48000000/1000 - 1)
  CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
}
