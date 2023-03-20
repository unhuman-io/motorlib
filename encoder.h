#ifndef UNHUMAN_MOTORLIB_ENCODER_H_
#define UNHUMAN_MOTORLIB_ENCODER_H_

#include "sensor.h"

// Base class for encoders, which by default are used as position sensors for
// both a motor encoder and an output encoder. An encoder should be periodically
// polled by a control loop that will first call trigger() then read(). Calling
// the trigger function ahead of read is useful, for example, for starting a
// digital read and then allowing some time for processing and returning later
// to reap the results with read.
//
// The encoder should return a int32_t that incrementally counts and wraps at
// 2^31 to 2^-31. Additional code in the main and fast loops will ensure that
// this wrap does not cause a control discontinuity. It should never be updated
// in a way that causes a discontinuity after the first read. That is, the first
// read can return any integer value, such as in the case of an absolute
// encoder, but additional reads should be incrementally accurate. The
// index_received() function is used to indicate presence of an index pulse
// received for an incremental encoder. It should be set to always true for an
// absolute encoder.
class EncoderBase : public SensorBase {
 public:
  EncoderBase() {}

  // init is called by system_init()
  bool init() { return false; }

  // Always called before read. One trigger and one read per control loop cycle.
  // Trigger should be called at a fixed frequency, ideally with no jitter.
  void trigger() {}

  // Read the results of the encoder measurement after calling trigger. It will
  // be called somewhat after trigger, possibly with some jitter.
  int32_t read() { return get_value(); }

  // get_value() should return the last read result but not trigger an update of
  // the sensor of any type
  int32_t get_value() const { return 0; }

  // For an incremental encoder with an index pulse, this should return the
  // latched position of the encoder value at the instant of the index pulse.
  // For an absolute encoder or an incremental encoder without an index pulse,
  // zero is a good return value for this function.
  int32_t get_index_pos() const { return 0; }

  // Set true after an index pulse is received or always true for an absolute
  // encoder.
  bool index_received() const { return false; }

  // Returns an offset error measurement as determined by the encoder. For an
  // incremental encoder with an index pulse this could be the count error
  // between an expected index pulse and a received index pulse. For an absolute
  // encoder this could return any useful error check information.
  int32_t index_error(int32_t cpr) { return 0; }
};

#endif  // UNHUMAN_MOTORLIB_ENCODER_H_
