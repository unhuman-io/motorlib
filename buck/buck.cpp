#include "buck.h"
#include "../control_fun.h"

BuckController::BuckController(float dt) {
    controller_ = new PIController;
}

BuckController::~BuckController() {
    delete controller_;
}

float BuckController::step(float current_desired, float current_measured, float voltage_feedforward) {
    float v_desired = controller_->step(current_desired, current_measured) + voltage_feedforward;
}