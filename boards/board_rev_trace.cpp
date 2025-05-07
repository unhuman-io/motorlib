module;

#include <map>
#include <string>
#include "../otp.h"
#include "st_device.h"
#include "../peripheral/stm32_serial.h"

export module board_rev_trace;

export struct BoardRev {
    enum Rev {kR0, kR1} rev;
};

export BoardRev get_board_rev() {
    init_serial_number();
    BoardRev b = {};

    std::map<std::string, BoardRev::Rev> rev_map = {{"R0", BoardRev::Rev::kR0},
                                                    {"R1", BoardRev::Rev::kR1}};
    std::string otp_rev(otp->rev);
    b.rev = rev_map[otp_rev];

// allowing #define logic to override OTP
#ifdef R0
    rev = BoardRev::Rev::kR0;
#elif defined(R1)
    rev = BoardRev::Rev::kR1;
#endif

    return b;
}
