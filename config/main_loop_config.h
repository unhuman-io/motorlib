#pragma once
#include "../controller/position_controller.h"
#include "../controller/torque_controller.h"
#include "../controller/impedance_controller.h"
#include "../controller/velocity_controller.h"
#include "../controller/state_controller.h"
#include "../controller/joint_position_controller.h"
#include "../controller/admittance_controller.h"

#include "fast_loop_config.h"

struct MainLoopConfigDefault {
    static constexpr int32_t frequency_hz = 10'000;
    using FastLoopType = FastLoop<FastLoopConfigDefault::frequency_hz>;
    using PositionControllerParamType = PositionControllerParam;
    template <float dt, auto p> using PositionControllerType = PositionController<dt, p>;
    template <float dt> using TorqueControllerType = TorqueController<dt>;
    template <float dt> using ImpedanceControllerType = ImpedanceController<dt>;
    template <float dt> using VelocityControllerType = VelocityController<dt>;
    template <float dt> using StateControllerType = StateController<dt>;
    template <float dt> using JointPositionControllerType = JointPositionController<dt>;
    template <float dt> using AdmittanceControllerType = AdmittanceController<dt>;
};
