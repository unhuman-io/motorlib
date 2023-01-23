#pragma once

class DriverBase {
 public:
    void enable() { enabled_ = true; }
    void disable() { enabled_ = false; }
    bool is_enabled() const { return enabled_; }
    bool is_faulted() const { return false; }
 private:
    bool enabled_ = false;
};
