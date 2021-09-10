#pragma once

#include "icpz.h"

class ICMU : public ICPZ {
 public:
    ICMU(SPIDMA &spidma) : ICPZ (spidma) {}
    bool init() {
        return true;
    }
};