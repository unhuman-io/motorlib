#pragma once

#ifndef FIGURE_ASSERT
#define FIGURE_ASSERT(x, ...) \
do {                          \
  if (!(x)) {                 \
    while(1);                 \
  }                           \
} while(0)
#endif // #ifndef FIGURE_ASSERT

#ifndef FIGURE_COUNTOF
#define FIGURE_COUNTOF(array) (sizeof(array)/sizeof((array)[0]))
#endif // #ifndef FIGURE_COUNTOF
