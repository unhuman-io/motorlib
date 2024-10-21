#include <stdlib.h>
#include "st_device.h"
#include "peripheral/macro.h"

using namespace std;

// Test if in interrupt mode
inline bool isInterrupt()
{
  return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

void * operator new(size_t size)
{
  FIGURE_ASSERT(!isInterrupt(), "Malloc cannot be called from inside of an interrupt");

  void * p = malloc(size);

  //FIGURE_ASSERT(p != NULL, "Malloc failed");
  return p;
}

void operator delete(void * p)
{
  FIGURE_ASSERT(!isInterrupt(), "Free cannot be called from inside of an interrupt");

  free(p);
}
