#pragma once

#include <map>
#include <string>
#include "../otp.h"
#include "st_device.h"
#include "../peripheral/stm32_serial.h"

struct BoardRev {
    enum Rev {kR0, kR1, kR2, kR3, kR4, kMR0, kMR0P, kMR1, kMR2} rev;
    bool has_max31875;
    bool has_max31889;
    bool has_bmi270;
    bool has_bridge_thermistors;
    bool has_5V_sense;
    bool has_I5V_sense;
    bool has_I48V_sense;
    bool has_mb85rc64;
};

BoardRev get_board_rev() {
    init_serial_number();
    BoardRev b = {};
    enum BoardRev::Rev& rev = b.rev;

    std::map<std::string, BoardRev::Rev> rev_map = {{"R0", BoardRev::Rev::kR0},
                                                    {"R1", BoardRev::Rev::kR1},
                                                    {"R2", BoardRev::Rev::kR2},
                                                    {"R3", BoardRev::Rev::kR3},
                                                    {"R4", BoardRev::Rev::kR4},
                                                    {"MR0", BoardRev::Rev::kMR0},
                                                    {"MR0P", BoardRev::Rev::kMR0P},
                                                    {"MR1", BoardRev::Rev::kMR1},
                                                    {"MR2", BoardRev::Rev::kMR2},};
    std::string otp_rev(otp->rev);
    b.rev = rev_map[otp_rev];

// allowing #define logic to override OTP
#ifdef R0
    rev = BoardRev::Rev::kR0;
#elif defined(R1)
    rev = BoardRev::Rev::kR1;
#elif defined(R2)
    rev = BoardRev::Rev::kR2;
#elif defined(R3)
    rev = BoardRev::Rev::kR3;
#elif defined(R4)
    rev = BoardRev::Rev::kR4;
#elif defined(MR0)
    rev = BoardRev::Rev::kMR0;
#elif defined(MR0P)
    rev = BoardRev::Rev::kMR0P;
#elif defined(MR1)
    rev = BoardRev::Rev::kMR1;
#elif defined(MR2)
    rev = BoardRev::Rev::kMR2;
#endif

    if (rev == BoardRev::Rev::kR3 || rev == BoardRev::Rev::kR4 || 
        rev == BoardRev::Rev::kMR0 || rev == BoardRev::Rev::kMR0P) {
        b.has_max31875 = true;
    }

    if (rev == BoardRev::Rev::kMR1 || rev == BoardRev::Rev::kMR2) {
        b.has_max31889 = true;
    }

    if (rev == BoardRev::Rev::kMR0 || rev == BoardRev::Rev::kMR0P ||
        rev == BoardRev::Rev::kMR1 || rev == BoardRev::Rev::kMR2) {
        b.has_bridge_thermistors = true;
    }

    if (rev == BoardRev::Rev::kR4 || 
        rev == BoardRev::Rev::kMR0 || rev == BoardRev::Rev::kMR0P ||
        rev == BoardRev::Rev::kMR1 || rev == BoardRev::Rev::kMR2) {
        b.has_bmi270 = true;
    }

    if (rev == BoardRev::Rev::kMR1 || rev == BoardRev::Rev::kMR2) {
        b.has_5V_sense = true;
        b.has_I5V_sense = true;
        b.has_I48V_sense = true;
    }

    if (rev == BoardRev::Rev::kMR2) {
        b.has_mb85rc64 = true;
    }
// keeping old #define logic
#ifdef BROKEN_MAX31875
    b.has_max31875 == false;
#endif

    return b;
}
