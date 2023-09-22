#ifndef UNHUMAN_MOTORLIB_ICMU_H_
#define UNHUMAN_MOTORLIB_ICMU_H_

#include "icpz.h"

class ICMU : public ICPZ {
 public:
    ICMU(SPIDMA &spidma) : ICPZ (spidma) {
        read_register_opcode_ = 0x97;
        write_register_opcode_ = 0xD2;
        type_ = MU;
    }
    bool init() {
        bool success = true;
        success = set_register(0, 0xe, {4}) ? success : false; // filter 4
        return success;
    }
};

#endif  // UNHUMAN_MOTORLIB_ICMU_H_
