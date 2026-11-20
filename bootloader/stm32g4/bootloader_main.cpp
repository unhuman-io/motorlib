#include <stdint.h>
#include <stm32g4xx.h>
#include <stm32g4/pin_config.h>
#include <stm32g4/flash.h>
#include <stm32g4/can.h>
#include <communication/can_communication.h>

uint32_t go_to_bootloader __attribute__((section(".bootloader_flag"))) = 0;

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

#define         DEVICE_ID1          (UID_BASE) //(0x1FFF7A10)
#define         DEVICE_ID2          (UID_BASE + 4) 
#define         DEVICE_ID3          (UID_BASE + 8)

char to_hex[] = "0123456789abcdef";

char * get_serial_number() {
    static char serial_number[13];
    uint32_t deviceserial0, deviceserial1, deviceserial2;

    deviceserial0 = *(uint32_t *)DEVICE_ID1;
    deviceserial1 = *(uint32_t *)DEVICE_ID2;
    deviceserial2 = *(uint32_t *)DEVICE_ID3;

    deviceserial0 += deviceserial2;

    //std::sprintf(serial_number,"%lX%X",deviceserial0, (uint16_t) (deviceserial1>>16));
  

    serial_number[0] = to_hex[(deviceserial0 >> 28) & 0xf];
    serial_number[1] = to_hex[(deviceserial0 >> 24) & 0xf];
    serial_number[2] = to_hex[(deviceserial0 >> 20) & 0xf];
    serial_number[3] = to_hex[(deviceserial0 >> 16) & 0xf];
    serial_number[4] = to_hex[(deviceserial0 >> 12) & 0xf];
    serial_number[5] = to_hex[(deviceserial0 >> 8) & 0xf];
    serial_number[6] = to_hex[(deviceserial0 >> 4) & 0xf];
    serial_number[7] = to_hex[(deviceserial0 >> 0) & 0xf];
    serial_number[8] = to_hex[(deviceserial1 >> 28) & 0xf];
    serial_number[9] = to_hex[(deviceserial1 >> 24) & 0xf];
    serial_number[10] = to_hex[(deviceserial1 >> 20) & 0xf];
    serial_number[11] = to_hex[(deviceserial1 >> 16) & 0xf];
    return serial_number;
}


extern volatile uint32_t * const cpu_clock = &DWT->CYCCNT;
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

uint8_t can_id = *(uint8_t *) (0x8060000);

int main() {
    SystemClock_Config();
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    GPIO_SETH(B, 8, GPIO_MODE::OUTPUT, GPIO_SPEED::MEDIUM, 0); // led b
    GPIO_SETL(B, 6, GPIO_MODE::OUTPUT, GPIO_SPEED::MEDIUM, 0); // led r
    GPIO_SETH(A, 8, GPIO_MODE::ALT_FUN, GPIO_SPEED::VERY_HIGH, 11); // can3 rx
    GPIO_SETL(B, 4, GPIO_MODE::ALT_FUN, GPIO_SPEED::VERY_HIGH, 11); // can3 tx

    Flash flash(*FLASH);
    CAN can(CAN::CAN3, CAN::ARB_1M, CAN::DATA_8M);
    CANCommunication can_communication(can, can_id);

    int loop_count = 0;
    while(1) {
        IWDG->KR = 0xAAAA;
        loop_count++;
        if (loop_count % 1000000 == 0) {
            GPIOB->BSRR = GPIO_BSRR_BS8;
        } else if (loop_count % 500000 == 0) {
            GPIOB->BSRR = GPIO_BSRR_BR8;
            GPIOB->BSRR = GPIO_BSRR_BR6;
        }
        enum {READ=0, WRITE=1};
        union {
            ReceiveData receive_data;
            struct {
                uint8_t command;
                uint8_t length;
                uint32_t address;
                uint8_t data[56];
            };
        } data;
        union {
            SendData send_data;
            struct {
                uint8_t command;
                uint8_t length;
                uint8_t data[62];
            };
        } send_data;
        int retval = can_communication.receive_data(&data.receive_data);
        if (retval > 0) {
            GPIOB->BSRR = GPIO_BSRR_BS6;
            if (data.command == READ) {
                // example cansend can0 101##00080000000200008
                if (data.length > 62) {
                    data.length = 62;
                }
                for (int i = 0; i < data.length; i++) {
                    send_data.data[i] = *(uint8_t *)data.address;
                    data.address++;
                }
                send_data.command = READ;
                send_data.length = data.length;
                can_communication.send_data(send_data.send_data);
            } else if (data.command == WRITE) {
                // example cansend can0 081##001800000002000080102030405060708
                if (data.length > 56) {
                    data.length = 56;
                }
                flash.write(data.address, data.data, data.length, Flash::ERASE_ONCE);
            }
        }
        char s[65];
        can_communication.receive_string(s);
        if (s[0] != 0) {
            if (strcmp(s, "name") == 0) {
                can_communication.send_string("bootloader", 10);
            } else if (strcmp(s, "serial") == 0) {
                can_communication.send_string(get_serial_number(), 12);
            } else if (strcmp(s, "messages_version") == 0) {
                can_communication.send_string(MOTOR_MESSAGES_VERSION, 3);
            } else if (strcmp(s, "version") == 0) {
                can_communication.send_string("1.0", 3);
            } else if (strcmp(s, "reset") == 0) {
                NVIC_SystemReset();
            } else {
                can_communication.send_string("", 0);
            }
        }

    }
    return 0;
}