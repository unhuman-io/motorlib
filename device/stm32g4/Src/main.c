
#include "../../motorlib/system.h"
#include "st_device.h"


void _close_r() {}
void _getpid_r() {}
void _kill_r() {}
void _lseek_r() {}
void _read_r() {}

uint32_t go_to_bootloader = 0;


int main(void)
{
  system_init();
  while (1)
  {
    system_run();
  }
}
