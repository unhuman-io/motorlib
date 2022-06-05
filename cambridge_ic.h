#pragma once
#include "encoder.h"


class CambridgeIC : public EncoderBase {
 public:
    CambridgeIC(SPIDMA &spidma)
        : spidma_(spidma) {
            command_[0] = 0x1;
            command_[1] = 0x02;
        }

    void trigger() {
      if (!*register_operation_) {
        ongoing_read_ = true;
        spidma_.start_readwrite(command_, data_, 4);
      }
    }
    int32_t read() {
      if (ongoing_read_) {
        spidma_.finish_readwrite();
        uint32_t data = (data_[2] << 8) | data_[3];
        pos_ = data;
        ongoing_read_ = false;
      }
      return get_value();
    }
    int32_t get_value() const {
      return pos_;
    }
    bool index_received() const { return true; }
 private:
    SPIDMA &spidma_;
    uint8_t command_[4] = {};
    uint8_t data_[4] = {};
    int32_t pos_ = 0;
    uint32_t last_data_ = 0;
    volatile int register_operation_local_ = 0;
    volatile int *register_operation_ = &register_operation_local_;
    bool ongoing_read_ = false;
};
