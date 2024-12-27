#include <stdint.h>
#include <stm32g4xx.h>
#include <stm32g4/pin_config.h>
#include <stm32g4/flash.h>
#include <stm32g4/can.h>

uint32_t go_to_bootloader = 0;

extern "C" {
void _lseek() {}
void _read() {}
void _write() {}
void _close() {}
void _fstat() {}
void _getpid() {}
void _isatty() {}
void _kill() {}
}

volatile uint32_t * const cpu_clock = &DWT->CYCCNT;
uint32_t rcc_csr_copy __attribute__((section (".noload")));

static_assert((uint32_t) CPU_FREQUENCY_HZ % 2000000 == 0, "CPU_FREQUENCY_HZ must be a multiple of 2000000");

extern "C" void SystemClock_Config(void)
{
    PWR->CR5 &= ~PWR_CR5_R1MODE; // R1MODE -> 0 for > 150 MHz operation

    // ensure cpu clock is started for us_delay
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // PWR_CR5_R1MODE change recommends 1 us startup
    //us_delay(1);

    FLASH->ACR |= FLASH_ACR_PRFTEN;
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | FLASH_ACR_LATENCY_4WS; // 4 flash wait states for 170 MHz
#ifdef USE_HSI
    RCC->PLLCFGR = 2 << RCC_PLLCFGR_PLLSRC_Pos | // (2) HSI is pll source (16 MHz)
      3 << RCC_PLLCFGR_PLLM_Pos | // (3) div4 
      (uint32_t) CPU_FREQUENCY_HZ/2000000 << RCC_PLLCFGR_PLLN_Pos | // (85) x85
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
    (uint32_t) CPU_FREQUENCY_HZ/2000000 << RCC_PLLCFGR_PLLN_Pos | // (85) x85
    2 << RCC_PLLCFGR_PLLPDIV_Pos | // (2) div2
    //RCC_PLLCFGR_PLLPEN |
    0 << RCC_PLLCFGR_PLLQ_Pos | // (0) div2
    //RCC_PLLCFGR_PLLQEN | 
    0 << RCC_PLLCFGR_PLLR_Pos | // (0) div2
    RCC_PLLCFGR_PLLREN;  
    RCC->CR = RCC_CR_HSEON | RCC_CR_HSION | RCC_CR_PLLON;
#endif

  RCC->CRRCR = RCC_CRRCR_HSI48ON;
  while(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY));
  while(!(RCC->CR & RCC_CR_PLLRDY));

  RCC->CFGR = 3 << RCC_CFGR_SW_Pos; // (3) // PLL clock

  RCC->CCIPR = 0 << RCC_CCIPR_CLK48SEL_Pos | // HSI48 (0) for usb
    2 << RCC_CCIPR_ADC12SEL_Pos | 2 << RCC_CCIPR_ADC345SEL_Pos | // (2) sysclk
    0 << RCC_CCIPR_I2C1SEL_Pos | 0 << RCC_CCIPR_I2C2SEL_Pos | // (0) pclk
    2 << RCC_CCIPR_FDCANSEL_Pos; // (2) pclk fdcan


  RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN;
  RCC->APB1SMENR1 |= RCC_APB1SMENR1_CRSSMEN;
  CRS->CFGR = 2 << CRS_CFGR_SYNCSRC_Pos | 34 << CRS_CFGR_FELIM_Pos |
    (48000000/1000 - 1) << CRS_CFGR_RELOAD_Pos; // DIV1, source usb sof (2), polarity rising, 34 felim was specificed by cubemx, reload (48000000/1000 - 1)
  CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
}

int main() {
    SystemClock_Config();
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    GPIO_SETH(B, 8, GPIO_MODE::OUTPUT, GPIO_SPEED::MEDIUM, 0);
    GPIO_SETH(A, 8, GPIO_MODE::ALT_FUN, GPIO_SPEED::VERY_HIGH, 11); // can3 rx
    GPIO_SETL(B, 4, GPIO_MODE::ALT_FUN, GPIO_SPEED::VERY_HIGH, 11); // can3 tx

    Flash flash(*FLASH);
    CAN can(CAN::CAN3, CAN::ARB_2M, CAN::DATA_5M);

    while(1) {
        IWDG->KR = 0xAAAA;

        for (int i = 0; i < 10000000; i++) {
            __asm__("nop");
        }
        GPIOB->BSRR = GPIO_BSRR_BS8;
        for (int i = 0; i < 10000000; i++) {
            __asm__("nop");
        }
        GPIOB->BSRR = GPIO_BSRR_BR8;
        can.write(0x123, 0, 0);

    }
    return 0;
}