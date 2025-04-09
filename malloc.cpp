#include <stdlib.h>
#include "st_device.h"
#include "peripheral/macro.h"

using namespace std;

// Test if in interrupt mode
inline bool isInterrupt()
{
  return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

extern "C" void * T32_malloc(size_t size);
extern "C" void * T32_realloc(void * ptr, size_t size);
extern "C" void T32_free(void * ptr);
void * operator new(size_t size)
{
  FIGURE_ASSERT(!isInterrupt(), "Malloc cannot be called from inside of an interrupt");

  void * p = T32_malloc(size);

  //FIGURE_ASSERT(p != NULL, "Malloc failed");
  return p;
}

void operator delete(void * p)
{
  FIGURE_ASSERT(!isInterrupt(), "Free cannot be called from inside of an interrupt");

  T32_free(p);
}
