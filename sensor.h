#pragma once

#include <cstdint>

class SensorBase {
 public:
    int32_t read() { return get_value(); }
    int32_t get_value() const { return 0; }
    void trigger() {}
    int32_t get_index_pos() const { return 0; }
    bool index_received() const { return false; }
    bool error() { return false; }
};
