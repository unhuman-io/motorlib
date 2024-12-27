#include <stdint.h>

uint32_t go_to_bootloader = 0;
uint32_t rcc_csr_copy __attribute__((section (".noload")));

extern "C" {
void _lseek() {}
void _read() {}
void _write() {}
void _close() {}
}


int main() {
    while(1) {

    }
    return 0;
}