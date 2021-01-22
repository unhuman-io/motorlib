#pragma once
#include "encoder.h"
#include "util.h"

class ICPZ : public EncoderBase {
 public:
    ICPZ(SPIDMA &spidma) 
      : spidma_(spidma) {
      command_[0] = 0xa6; // read position
    }
    bool init() {
       uint8_t command[6] = {0xcf, 0x40, 0x07}; // set memory bank 7
       uint8_t data_in[3];
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x09;
       command[2] = 0x00;     // multiturn data length = 0
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x40;
       command[2] = 0x08;     // memory bank 8
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x00;
       command[2] = 0xbe;     // flexcode 446, 216
       command[3] = 0x01;
       command[4] = 216;
       command[5] = 0x00;
       spidma_.readwrite(command, data_in, 6);
       command[0] = 0xcf;
       command[1] = 0x40;
       command[2] = 0x00;     // memory bank 0
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x07;
       command[2] = 0x09;     // sys_ovr = 9 bit
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x0F;
       command[2] = 0x80;     // ran_fld = 0 -> never update position based on absolute track after initial
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x40;
       command[2] = 0x01;     // memory bank 1
       spidma_.readwrite(command, data_in, 3);
       command[0] = 0xcf;
       command[1] = 0x08;
       command[2] = 0xC0;     // ai_phase = -20
       command[3] = 0xF1;
       spidma_.readwrite(command, data_in, 4);
       command[0] = 0xcf;
       command[1] = 0x0a;
       command[2] = 0x00;     // ai_scale = .9558
       command[3] = 0xD8;
       spidma_.readwrite(command, data_in, 4);
       command[0] = 0xd9;
       command[1] = 0x41;     // write command conf write all    
       spidma_.readwrite(command, data_in, 2);
       ms_delay(10);
       command[0] = 0x81;
       command[1] = 0x76;
       command[2] = 0x00;     // cmd stat
       command[3] = 0x00;
       spidma_.readwrite(command, data_in, 4);
       if (data_in[3] == 0) { // cmd succeeded
         return true;
       }
       return false;
    }
    void trigger() {
      if (!*register_operation_) {
        spidma_.start_readwrite(command_, data_, 4);
      }
    }
    int32_t read() {
      if (!*register_operation_) {
        spidma_.finish_readwrite();
        //uint32_t data = (data_[1] << 16) | (data_[2] << 8) | data_[3];
        //uint16_t data = (data_[2] << 8) | data_[3];
        //pos_ += (int16_t) (data - last_data_); // rollover summing
        //pos_ = data;
        pos_ = data_[1];
        //last_data_ = data;
      }
      return get_value();
    }
    int32_t get_value() const {
      return pos_;
    }
    bool index_received() const { return true; }

    void set_register_operation() {
       (*register_operation_)++;
    }
    void clear_register_operation() {
       (*register_operation_)--;
    }

        // non interrupt context
    uint8_t read_register(uint8_t address) {
        (*register_operation_)++;
        uint8_t retval = 0;
        (*register_operation_)--;
        return retval;
    }

    // non interrupt context
    bool set_register(uint8_t bank, uint8_t address, uint8_t *value, uint8_t length) {
        (*register_operation_)++;

        bool retval = 0;
        (*register_operation_)--;
        return retval;
    }

 private:
    SPIDMA &spidma_;
    uint8_t command_[4] = {};
    uint8_t data_[4] = {};
    int32_t pos_ = 0;
    uint16_t last_data_ = 0;
    volatile int register_operation_local_ = 0;
    volatile int *register_operation_ = &register_operation_local_;
};
