#ifndef UNHUMAN_MOTORLIB_SENSOR_H_
#define UNHUMAN_MOTORLIB_SENSOR_H_

#include <cstdint>

class SensorBase {
 public:
    int32_t read() { return get_value(); }
    int32_t get_value() const { return 0; }
    void trigger() {}
    int32_t get_index_pos() const { return 0; }
    bool index_received() const { return false; }
    void reinit() {}
    bool error() { return false; }
    void clear_faults() {}
};

#endif  // UNHUMAN_MOTORLIB_SENSOR_H_
