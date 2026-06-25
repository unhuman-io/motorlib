#pragma once

#include "../control_fun_param.h"

struct PositionControllerParam {
    PIDParam position;
    float velocity_limit;
    float desired_filter_hz;
    float tracking_tolerance;
};
