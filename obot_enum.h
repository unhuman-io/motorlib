#pragma once

#include <cstring>
#include "motor_messages/motor_messages.h"
#include "peripheral/stm32_serial.h"
#include "otp.h"

// Helper to safely grab a character or return '\0' if out of bounds
#define CHAR_OR_NULL(s, i) ((i) < sizeof(s) ? (s)[i] : '\0')

// Expands a string literal into a 10-character braced initializer list
#define TRUNC_STR_10(s) { \
    CHAR_OR_NULL(s, 0), CHAR_OR_NULL(s, 1), CHAR_OR_NULL(s, 2), CHAR_OR_NULL(s, 3), \
    CHAR_OR_NULL(s, 4), CHAR_OR_NULL(s, 5), CHAR_OR_NULL(s, 6), CHAR_OR_NULL(s, 7), \
    CHAR_OR_NULL(s, 8), CHAR_OR_NULL(s, 9) \
}

EnumResponse get_enum_response() {
    EnumResponse response {
        .messages_version = {MOTOR_MESSAGES_VERSION_MAJOR, MOTOR_MESSAGES_VERSION_MINOR},
        .version = 1,
        .config = TRUNC_STR_10(CONFIG),
    };
    if (otp->version == 1) {
        std::memcpy(response.board_name, otp->name, sizeof(response.board_name));
        response.board_rev = otp->rev[0];
        response.board_num = otp->num;
    }
    std::memcpy(response.name, param->name, sizeof(response.name));
    std::memcpy(response.serial_number, get_serial_number(), sizeof(response.serial_number));
    return response;
}