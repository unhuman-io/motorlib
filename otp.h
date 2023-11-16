#ifndef UNHUMAN_MOTORLIB_OTP_H_
#define UNHUMAN_MOTORLIB_OTP_H_

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t version;
    char name[20];
    char rev[5];
} OTP;

extern OTP otp;

#define otp ((OTP *) FLASH_OTP_BASE)

#endif // UNHUMAN_MOTORLIB_OTP_H_
