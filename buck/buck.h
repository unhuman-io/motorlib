#ifndef UNHUMAN_MOTORLIB_BUCK_BUCK_H_
#define UNHUMAN_MOTORLIB_BUCK_BUCK_H_
#include "messages_buck.h"

class PIController;

// control current to a load, feedforward voltage
class BuckController {
 public:
    BuckController(float dt);
    ~BuckController();
    // returns voltage desired
    float step(float current_desired, float current_measured, float voltage_feedforward=0);
 private:
    PIController *controller_;
};

#endif  // UNHUMAN_MOTORLIB_BUCK_BUCK_H_
