#ifndef ST_DEVICE_H
#define ST_DEVICE_H

// Not set by the SVDConv header; the core is built with DSP (-mcpu=cortex-m33)
#define __DSP_PRESENT 1
#include "STM32G564.h"
#ifndef CPU_FREQUENCY_HZ
#define CPU_FREQUENCY_HZ    ((float) 64e6)
#endif

#endif
