#pragma once

#include <cstdint>

class Sensor {
 public:
    virtual int32_t read() { return get_value(); }
    virtual int32_t get_value() const { return 0; }
    virtual void trigger() {}
    virtual int32_t get_index_pos() const { return 0; }
    virtual bool index_received() const { return false; }
};
