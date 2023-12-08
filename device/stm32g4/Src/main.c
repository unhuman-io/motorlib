
#include "../../motorlib/system.h"
#include "st_device.h"


void _close_r() {}
void _fstat_r() {}
void _getpid_r() {}
void _isatty_r() {}
void _kill_r() {}
void _lseek_r() {}
void _read_r() {}
void _write_r() {}

uint32_t go_to_bootloader = 0;


int main(void)
{
  system_init();
  while (1)
  {
    system_run();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // HAL_Init();

  // RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  // RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  // RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  // RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage 
  */
//  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
#ifdef USE_HSI
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  // RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  // RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  // RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  // RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  // RCC_OscInitStruct.PLL.PLLN = 85;
  // RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  // RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  // RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;


  // calibration

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
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;
  // RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  // RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  // RCC_OscInitStruct.PLL.PLLN = 85;
  // RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  // RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  // RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

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
  
  // if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  // RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
  //                             |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  // RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  // RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  // RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  RCC->CFGR = 3 << RCC_CFGR_SW_Pos; // (3) // PLL clock


  // if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  /** Initializes the peripherals clocks 
  */
  // PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
  //                             |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC12
  //                             |RCC_PERIPHCLK_ADC345;
  // PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  // PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  // PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  // PeriphClkInit.Adc1obot_g474/st_device.hInit);
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1);
  /* USER CODE END Error_Handler_Debug */
}
