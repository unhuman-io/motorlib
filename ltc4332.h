#ifndef UNHUMAN_MOTORLIB_LTC4332_H_
#define UNHUMAN_MOTORLIB_LTC4332_H_

class LTC4332 {
 public:
    LTC4332(SPIDMA &spidma) {
        // set ss1 pha = 1
        uint8_t data_out[] = {2, 1};
        uint8_t data_in[2];
        spidma.readwrite(data_out, data_in, 2, true);
    }
};

#endif // UNHUMAN_MOTORLIB_LTC4332_H_
