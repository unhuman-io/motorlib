#ifndef UNHUMAN_MOTORLIB_SENSOR_H_
#define UNHUMAN_MOTORLIB_SENSOR_H_

#include <cstdint>

class SensorBase {
 public:
  void reinit() {}
  void trigger() {}

  // TODO: Consider moving these into the EncoderBase.
  int32_t get_index_pos() const { return 0; }
  bool index_received() const { return false; }

  bool error() { return false; }
};

#endif  // UNHUMAN_MOTORLIB_SENSOR_H_
