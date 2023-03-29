#ifndef UNHUMAN_MOTORLIB_DRIVER_H_
#define UNHUMAN_MOTORLIB_DRIVER_H_

class DriverBase {
 public:
  void enable() { enabled_ = true; }
  void disable() { enabled_ = false; }
  bool is_enabled() const { return enabled_; }
  bool is_faulted() const { return false; }

 private:
  bool enabled_ = false;
};

#endif  // UNHUMAN_MOTORLIB_DRIVER_H_
