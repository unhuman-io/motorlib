#pragma once
#include "torque_sensor.h"
#include "util.h"

class ADS1235 : public TorqueSensorBase {
 public:
    ADS1235(SPIDMA &spidma, volatile int* register_operation = nullptr) : 
        spidma_(spidma) {
            if (register_operation != nullptr) {
                register_operation_ = register_operation;
            }
      command_[0] = 0x12;
    }
    uint32_t init() {
       uint32_t retval = 0;
       uint32_t shift = 0;
       retval |= set_register(2, 0x4B) << shift++; // 1200 SPS, sinc4 filter
       retval |= set_register(0x10, 0x07) << shift++; // pga gain 128
       retval |= set_register(0x11, 0x34) << shift++; // inputs AIN0 AIN1
       return retval;
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
        torque_ = gain_*torque_int + bias_;
      }
      return torque_;
    }
    volatile int *register_operation_ = &register_operation_local_;
 private:

    uint8_t read_register(uint8_t address) {
        (*register_operation_)++;
        uint8_t command[3] = {(uint8_t) (0x20u+address)};
        uint8_t data_in[3];
        spidma_.readwrite(command, data_in, 3);
        (*register_operation_)--;
        return data_in[2];
    }

    // non interrupt context
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
    float torque_ = 0;

    friend void system_init();
};
