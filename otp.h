#ifndef UNHUMAN_MOTORLIB_OTP_H_
#define UNHUMAN_MOTORLIB_OTP_H_

#include <stdint.h>

typedef struct {
    uint8_t version;
    char name[20];
    char rev[5];
    char reserved[2];
    int32_t num;
} OTP;

static_assert(sizeof(OTP) == 32);

extern OTP otp;

#define otp ((OTP *) FLASH_OTP_BASE)

#endif // UNHUMAN_MOTORLIB_OTP_H_
