#pragma once

// A hardware brake that is expected to be off in any mode other than HARDWARE_BRAKE

class HardwareBrakeBase {
 public:
    void on() {}
    void off() {}
};
