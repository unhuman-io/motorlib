#pragma once

template<typename Config>
struct MainLoopParam2 {
    typename Config::PositionControllerParamType position_controller {
        .position {
            .kp = 0,
        },
    };
};
