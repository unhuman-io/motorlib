#pragma once

#include <math.h>

#define TO_Q31(x) ((int32_t) (x * (float) 0x80000000))

inline int32_t atan2f_q31(float y, float x) {
    float max = fmaxf(fabsf(y), fabsf(x));
    float one_max = 1/max;
    CORDIC->CSR = 5 << CORDIC_CSR_PRECISION_Pos | 2 << CORDIC_CSR_FUNC_Pos | CORDIC_CSR_NARGS; // phase calculation
    int32_t y_q31 = TO_Q31(y*one_max);
    int32_t x_q31 = TO_Q31(x*one_max);
    CORDIC->WDATA = x_q31;
    CORDIC->WDATA = y_q31;
    return CORDIC->RDATA;
}