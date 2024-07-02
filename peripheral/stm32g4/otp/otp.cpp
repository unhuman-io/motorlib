#include "otp.h"

#if !defined(NAME) || !defined(REV) || !defined(NUM)
#error Must define NAME, REV, and NUM to use this file
#endif

static_assert(NAME == "motor_molex" ? 
    REV == "MR0" || REV == "MR0P" || REV == "MR1" || REV == "MR2" : 
    NAME == "motor" ?
    REV == "R0" || REV == "R1" || REV == "R3" || REV == "R4" :
    NAME == "trace" ?
    REV == "R0" :
    false);




const volatile OTP __attribute__ ((section ("otp"), used)) otp1 = {
    .version = 1,
    .name = NAME,
    .rev = REV,
    .num = NUM,
};

