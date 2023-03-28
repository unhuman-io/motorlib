#ifndef UNHUMAN_MOTORLIB_SINCOS_H_
#define UNHUMAN_MOTORLIB_SINCOS_H_

struct Sincos {
  float sin, cos;
};

// input x in radians
Sincos sincos1(float x) __attribute__((section(".ccmram")));

#endif  // UNHUMAN_MOTORLIB_SINCOS_H_
