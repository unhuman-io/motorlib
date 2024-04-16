#ifndef UNHUMAN_MOTORLIB_ADS1235_H_
#define UNHUMAN_MOTORLIB_ADS1235_H_

#include "torque_sensor.h"
#include "util.h"

extern "C" {
void system_init();
}

class ADS1235 : public TorqueSensorBase {
 public:
    enum PGAGain {GAIN_1=0, GAIN_64=6, GAIN_128=7};
    enum SPS {SPS_2_5=0, SPS_10=2, SPS_100=7, SPS_1200=9, SPS_2400=10, SPS_4800=11, SPS_7200=12};
    ADS1235(SPIDMA &spidma, PGAGain pga_gain = GAIN_128, SPS sps = SPS_1200, volatile int* register_operation = nullptr) : 
        spidma_(spidma), pga_gain_(pga_gain) {
            if (register_operation != nullptr) {
                register_operation_ = register_operation;
            }
      command_[0] = 0x12;
      constructor_init(sps);
    }
    void constructor_init(SPS sps) {
       init_val_ = 0;
       init_val_ = set_register(2, (sps << 3) + 3); // SPS, sinc4 filter
       init_val_ += set_register(3, 0x21) ? 10 : 0; // chop mode, 50 us start delay (default)
       init_val_ += set_register(0x10, gain_) ? 100 : 0; // pga gain 128
       init_val_ += set_register(0x11, 0x34) ? 1000 : 0; // inputs AIN0 AIN1
    }
    uint32_t init() {
      return init_val_;
    }
    void trigger() {
      if (!*register_operation_) {
        spidma_.start_readwrite(command_, data_, 5);
      }
    }
    float read() {
      if (!*register_operation_) {
        spidma_.finish_readwrite();
        int32_t torque_int = signextend<int32_t, 24>((data_[2]) << 16 | (data_[3] << 8) | data_[4]);
        torque_ = gain_*torque_int;
      }
      return torque_;
    }
    volatile int *register_operation_ = &register_operation_local_;
 protected:

    uint8_t read_register(uint8_t address) {
        (*register_operation_)++;
        uint8_t command[3] = {(uint8_t) (0x20u+address)};
        uint8_t data_in[3];
        spidma_.readwrite(command, data_in, 3);
        (*register_operation_)--;
        return data_in[2];
    }

    // non interrupt context
    // returns true for success
    bool set_register(uint8_t address, uint8_t value) {
        (*register_operation_)++;
        bool retval = true;
        if (read_register(address) != value) {
            uint8_t command[2] = {(uint8_t) (0x40+address), value};
            uint8_t data_in[2];
            spidma_.readwrite(command, data_in, 2);
            retval = read_register(address) == value;
        }
        (*register_operation_)--;
        return retval;
    }

    SPIDMA &spidma_;
    uint8_t command_[5] = {};
    uint8_t data_[5] = {};
    volatile int register_operation_local_ = 0;
    PGAGain pga_gain_;
    uint32_t init_val_ = 0;

    friend class System;
    friend void system_init();
};

#endif  // UNHUMAN_MOTORLIB_ADS1235_H_
