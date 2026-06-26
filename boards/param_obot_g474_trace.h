#pragma once

#include "../config/main_loop_param.h"

template<typename Config>
struct TraceParam {
    MainLoopParam2<Config> main_loop {
        .position_controller {
            .position {
                .kp = 1,
            },
        },
    };
};
