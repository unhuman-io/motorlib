#include "../stm32_serial.h"
#include <stm32f446xx.h>

#define         DEVICE_ID1          (UID_BASE) //(0x1FFF7A10)
#define         DEVICE_ID2          (UID_BASE + 4) 
#define         DEVICE_ID3          (UID_BASE + 8) 

// This is the serial number used by the bootloader, 12 bytes
const char * get_serial_number() {
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = *(uint32_t *)DEVICE_ID1;
  deviceserial1 = *(uint32_t *)DEVICE_ID2;
  deviceserial2 = *(uint32_t *)DEVICE_ID3;

  deviceserial0 += deviceserial2;
  char buffer[13];
  std::sprintf(buffer,"%lX%X",deviceserial0, (uint16_t) (deviceserial1>>16));
  return buffer;
}

void init_serial_number() {}
