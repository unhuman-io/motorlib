#ifndef UNHUMAN_MOTORLIB_HARDWARE_BRAKE_H_
#define UNHUMAN_MOTORLIB_HARDWARE_BRAKE_H_

// A hardware brake that is expected to be off in any mode other than HARDWARE_BRAKE

class HardwareBrakeBase {
 public:
    void on() {}
    void off() {}
};

#endif  // UNHUMAN_MOTORLIB_HARDWARE_BRAKE_H_
