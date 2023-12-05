#pragma once

#include <map>
#include <string>
#include "../otp.h"
#include "../st_device.h"

struct BoardRev {
    enum Rev {R0, R1, R2, R3, R4, MR0, MR0P, MR1, MR2} rev;
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
    BoardRev b = {};
    enum BoardRev::Rev& rev = b.rev;

    std::map<std::string, BoardRev::Rev> rev_map = {{"R0", BoardRev::Rev::R0},
                                                    {"R1", BoardRev::Rev::R1},
                                                    {"R2", BoardRev::Rev::R2},
                                                    {"R3", BoardRev::Rev::R3},
                                                    {"R4", BoardRev::Rev::R4},
                                                    {"MR0", BoardRev::Rev::MR0},
                                                    {"MR0P", BoardRev::Rev::MR0P},
                                                    {"MR1", BoardRev::Rev::MR1},
                                                    {"MR2", BoardRev::Rev::MR2},};
    std::string otp_rev(otp->rev);
    b.rev = rev_map[otp_rev];

// allowing #define logic to override OTP
#ifdef R0
    rev = BoardRev::Rev::R0;
#elif defined(R1)
    rev = BoardRev::Rev::R1;
#elif defined(R2)
    rev = BoardRev::Rev::R2;
#elif defined(R3)
    rev = BoardRev::Rev::R3;
#elif defined(R4)
    rev = BoardRev::Rev::R4;
#elif defined(MR0)
    rev = BoardRev::Rev::MR0;
#elif defined(MR0P)
    rev = BoardRev::Rev::MR0P;
#elif defined(MR1)
    rev = BoardRev::Rev::MR1;
#elif defined(MR2)
    rev = BoardRev::Rev::MR2;
#endif

    if (rev == BoardRev::Rev::R3 || rev == BoardRev::Rev::R4 || 
        rev == BoardRev::Rev::MR0 || rev == BoardRev::Rev::MR0P) {
        b.has_max31875 = true;
    }

    if (rev == BoardRev::Rev::MR1 || rev == BoardRev::Rev::MR2) {
        b.has_max31889 = true;
    }

    if (rev == BoardRev::Rev::MR0 || rev == BoardRev::Rev::MR0P ||
        rev == BoardRev::Rev::MR1 || rev == BoardRev::Rev::MR2) {
        b.has_bridge_thermistors = true;
    }

    if (rev == BoardRev::Rev::R4 || 
        rev == BoardRev::Rev::MR0 || rev == BoardRev::Rev::MR0P ||
        rev == BoardRev::Rev::MR1 || rev == BoardRev::Rev::MR2) {
        b.has_bmi270 = true;
    }

    if (rev == BoardRev::Rev::MR1 || rev == BoardRev::Rev::MR2) {
        b.has_5V_sense = true;
        b.has_I5V_sense = true;
        b.has_I48V_sense = true;
    }

    if (rev == BoardRev::Rev::MR2) {
        b.has_mb85rc64 = true;
    }
// keeping old #define logic
#ifdef BROKEN_MAX31875
    b.has_max31875 == false;
#endif

    return b;
}
