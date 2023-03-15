#ifndef UNHUMAN_MOTORLIB_BUCK_MAIN_LOOP_BUCK_H_
#define UNHUMAN_MOTORLIB_BUCK_MAIN_LOOP_BUCK_H_

#include "messages_buck.h"

class Communication;

template<typename FastLoopBuck>
class MainLoopBuck {
 public:
    MainLoopBuck(Communication &communication) : communication_(communication) {}
    void update() {
        float v_bus = fast_loop_.get_v_bus();
        float i_bus = fast_loop_.get_i_bus();
        float v_out = fast_loop_.get_v_out();
        float i_out = fast_loop_.get_i_out();

        switch (state_) {
            case SLEEP:
                if (v_bus > 13.6 && v_out > 10) {
                    count_start_++;
                } else {
                    count_start_ = 0;
                }
                if (count_start_ > 10) {
                    state_ = RUN;
                    count_start_ = 0;
                    fast_loop_.set_current_desired{10};
                    fast_loop_.set_voltage_feedforward(v_bus);
                    fast_loop_.current_mode();
                }
                break;
            case RUN:
                if (v_bus < 13.5 || i_bus < -1) {
                    count_end_++;
                } else {
                    count_end_ = 0;
                }
                if (count_end_ > 10 ||
                    i_out > 15 ||       // faults
                    v_bus < 10 ||
                    v_out < 10) {
                    state_ = SLEEP;
                    count_end_ = 0;
                    fast_loop_.set_current_desired(0);
                    fast_loop_.open_mode();
                }
                break;
        }

        buck_status_.i_bus = i_bus;
        buck_status_.i_out = i_out;
        buck_status_.v_bus = v_bus;
        buck_status_.v_out = v_out;
        communication_.send(buck_status_);
    }
 private:
    Communication &communication_;
    BuckStatus buck_status_ = {};
    enum {SLEEP, RUN} state_ = SLEEP;
    FastLoopBuck fast_loop_;
    int count_start_ = 0;
    int count_end_ = 0;
};

#endif  // UNHUMAN_MOTORLIB_BUCK_MAIN_LOOP_BUCK_H_
