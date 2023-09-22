#ifndef UNHUMAN_MOTORLIB_MAX11158_H_
#define UNHUMAN_MOTORLIB_MAX11158_H_
#include "torque_sensor.h"
#include "logger.h"
#include "parameter_api.h"

class MAX11158 : public TorqueSensorBase {
 public:
    MAX11158(SPIDMA &spi_dma, uint8_t decimation=0) :
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
            raw_value_ = ~(data_in_[0] << 16 | data_in_[1] << 8 | data_in_[2]) & 0xFFFFFF; // invert for mistake on board
            signed_value_ = (int32_t) (raw_value_ >> 6) - 0x20000;
            torque_ = signed_value_ * gain_ + bias_;

            // stale values are currently the only fault mechanism
            if (last_new_signed_value_ != signed_value_) {
                last_new_signed_value_ = signed_value_;
                stale_value_count_ = 0;
            } else {
                stale_value_count_++;
                if (stale_value_count_ > 100) {
                    timeout_error_++;
                    stale_value_count_ = 0;
                }
            }
            if (raw_value_ == 0 || raw_value_ == 0xFFFFFF) {
                read_error_++;
            }

        }
        return torque_;
    }

    float get_value() const { return torque_; }

    void clear_faults() {
        timeout_error_ = 0;
        stale_value_count_ = 0;
        read_error_ = 0;
    }

    void set_debug_variables(ParameterAPI &api) {
        api.add_api_variable("max_raw", new const APIUint32(&raw_value_));
        api.add_api_variable("max_int", new const APIInt32(&signed_value_));
        api.add_api_variable("max_timeout_error", new const APIUint32(&timeout_error_));
        api.add_api_variable("max_read_error", new const APIUint32(&read_error_));
        api.add_api_variable("max_read_error", new const APIUint32(&read_error_));
    }

    uint8_t count_ = 0;

    int32_t signed_value_ = 0;
    int32_t last_new_signed_value_ = 0;
    uint8_t stale_value_count_ = 0;
    uint32_t timeout_error_ = 0;
    uint32_t read_error_ = 0;
    uint32_t raw_value_ = 0;
    SPIDMA &spi_dma_;
    static const uint8_t length_ = 5;
    uint8_t data_out_[length_] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data_in_[length_] = {};
    uint8_t decimation_ = 0;
};


#endif // UNHUMAN_MOTORLIB_MAX11158_H_
