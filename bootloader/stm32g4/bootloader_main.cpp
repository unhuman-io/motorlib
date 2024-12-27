#include <stdint.h>
#include <stm32g4xx.h>
#include <stm32g4/pin_config.h>

uint32_t go_to_bootloader = 0;
uint32_t rcc_csr_copy __attribute__((section (".noload")));

extern "C" {
void _lseek() {}
void _read() {}
void _write() {}
void _close() {}
}


int main() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    GPIO_SETH(B, 8, GPIO_MODE::OUTPUT, GPIO_SPEED::MEDIUM, 0);

    while(1) {
        IWDG->KR = 0xAAAA;

        for (int i = 0; i < 1000000; i++) {
            __asm__("nop");
        }
        GPIOB->BSRR = GPIO_BSRR_BS8;
        for (int i = 0; i < 1000000; i++) {
            __asm__("nop");
        }
        GPIOB->BSRR = GPIO_BSRR_BR8;

    }
    return 0;
}