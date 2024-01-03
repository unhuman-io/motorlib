#ifndef UNHUMAN_MOTORLIB_LTC4332_H_
#define UNHUMAN_MOTORLIB_LTC4332_H_

#include "logger.h"

class LTC4332 {
 public:
    LTC4332(SPIDMA &spidma) {
        // set ss1 pha = 1 in reg 0 / config
        uint8_t data_out[] = {0, 1};
        uint8_t data_in[2];
        spidma.readwrite(data_out, data_in, 2, true);

        // Logger not currently functional in constructors
        // // read reg 0 / config
        // data_out[0] = 1;
        // spidma.readwrite(data_out, data_in, 2, true);
        // logger.log_printf("LTC4332 reg 0: 0x%02x", data_in[1]);
    }
};

#endif // UNHUMAN_MOTORLIB_LTC4332_H_
