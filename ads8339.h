#pragma once
#include "torque_sensor.h"
#include "logger.h"

class ADS8339 : public TorqueSensorBase {
 public:
    ADS8339(SPIDMA &spi_dma, uint8_t decimation=0) :
        TorqueSensorBase(), spi_dma_(spi_dma), decimation_(decimation) {
    }
    bool init() { 
        return true;
    }

    void trigger() {
        if (++count_ <= decimation_) {
            return;
        }
        count_ = 0;
        spi_dma_.start_readwrite(data_out_, data_in_, length_);
    }

    float read() {
        if (count_ == 0) {
            spi_dma_.finish_readwrite();
            raw_value_ = data_in_[0] << 8 | data_in_[1];
            signed_value_ = raw_value_ - 0x7FFF;
            torque_ = signed_value_ * gain_ + bias_;

            // stale values are currently the only fault mechanism
            if (last_new_signed_value_ != signed_value_) {
                last_new_signed_value_ = signed_value_;
                stale_value_count_ = 0;
            } else {
                stale_value_count_++;
                if (stale_value_count_ > 3) {
                    timeout_error_++;
                    stale_value_count_ = 0;
                }
            }
            if (raw_value_ == 0 || raw_value_ == 0xFFFF) {
                read_error_++;
            }

        }
        return torque_;
    }

    void clear_faults() {
        timeout_error_ = 0;
        stale_value_count_ = 0;
        read_error_ = 0;
    }
    uint8_t count_ = 0;

    int32_t signed_value_ = 0;
    int32_t last_new_signed_value_ = 0;
    uint8_t stale_value_count_ = 0;
    uint32_t timeout_error_ = 0;
    uint32_t read_error_ = 0;
    uint32_t raw_value_ = 0;
    SPIDMA &spi_dma_;
    static const uint8_t length_ = 2;
    uint8_t data_out_[length_] = {};
    uint8_t data_in_[length_] = {};
    uint8_t decimation_ = 0;
};
