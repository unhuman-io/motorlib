module;
#include "../peripheral/stm32g4/clock_config.h"
export module trace_board;

export class TraceBoard {
 public:
    static void board_init() {
        SystemClock_Config();
    }
};
