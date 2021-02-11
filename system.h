#pragma once

#ifdef __cplusplus
#include "parameter_api.h"
#include <queue>
#include <string>

extern uint32_t t_exec_fastloop;
extern uint32_t t_exec_mainloop;
extern uint32_t t_period_fastloop;
extern uint32_t t_period_mainloop;

void system_maintenance();

class System {
 public:
    static void run() {
        actuator_.start();

        log("finished startup");

        uint32_t cpu_frequency = CPU_FREQUENCY_HZ;
        api.add_api_variable("mode", new APIUint32((uint32_t *) &actuator_.main_loop_.mode_));
        api.add_api_variable("kp", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.kp_));
        api.add_api_variable("kd", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.kd_));
        api.add_api_variable("ki", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.ki_));
        api.add_api_variable("ki_limit", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.ki_limit_));
        api.add_api_variable("max", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.command_max_));
        api.add_api_variable("vkp", new APIFloat(&actuator_.main_loop_.velocity_controller_.controller_.kp_));
        api.add_api_variable("vkd", new APIFloat(&actuator_.main_loop_.velocity_controller_.controller_.kd_));
        api.add_api_variable("cpu_frequency", new APIUint32(&cpu_frequency));
        api.add_api_variable("t_exec_fastloop", new APIUint32(&t_exec_fastloop));
        api.add_api_variable("t_exec_mainloop", new APIUint32(&t_exec_mainloop));
        api.add_api_variable("t_period_fastloop", new APIUint32(&t_period_fastloop));
        api.add_api_variable("t_period_mainloop", new APIUint32(&t_period_mainloop));
        api.add_api_variable("vbus", new APIFloat(&actuator_.main_loop_.status_.fast_loop.vbus));
        api.add_api_variable("va", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_a));
        api.add_api_variable("vb", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_b));
        api.add_api_variable("vc", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_c));
        api.add_api_variable("vq", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_q));
        api.add_api_variable("vd", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_d));
        api.add_api_variable("ia", new APIFloat(&actuator_.fast_loop_.foc_command_.measured.i_a));
        api.add_api_variable("ib", new APIFloat(&actuator_.fast_loop_.foc_command_.measured.i_b));
        api.add_api_variable("ic", new APIFloat(&actuator_.fast_loop_.foc_command_.measured.i_c));
        api.add_api_variable("id", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_d));
        api.add_api_variable("i0", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_0));
        api.add_api_variable("ikp", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_->kp_));
        api.add_api_variable("iki", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_->ki_));
        api.add_api_variable("iki_limit", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_->ki_limit_));
        api.add_api_variable("imax", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_->command_max_));
        api.add_api_variable("tkp", new APIFloat(&actuator_.main_loop_.torque_controller_.controller_.kp_));
        api.add_api_variable("tkd", new APIFloat(&actuator_.main_loop_.torque_controller_.controller_.kd_));
        api.add_api_variable("tmax", new APIFloat(&actuator_.main_loop_.torque_controller_.controller_.command_max_));

        std::function<void(float)> set_filt = std::bind(&SecondOrderLowPassFilter::set_frequency, &actuator_.main_loop_.position_controller_.controller_.error_dot_filter_, std::placeholders::_1);
        std::function<float(void)> get_filt = std::bind(&SecondOrderLowPassFilter::get_frequency, &actuator_.main_loop_.position_controller_.controller_.error_dot_filter_);
        api.add_api_variable("filt", new APICallbackFloat(get_filt, set_filt));

        while(1) {
            char *s = System::get_string();
            if (s[0] != 0) {
                System::log(api.parse_string(s));
            }
            send_log();
            system_maintenance();
            actuator_.maintenance();
        }
    }
    static void main_loop_interrupt() {
        actuator_.main_loop_.update();
    }
    static void fast_loop_interrupt() {
        actuator_.fast_loop_.update();
    }
    static void log(std::string str) {
        if (log_queue_.size() < 10) {
            log_queue_.push(str);
        }
    }
    static void send_log() {
        if (!log_queue_.empty()) {
            std::string str = log_queue_.front();
            bool sent = communication_.send_string(str.c_str(), str.length());
            if (sent) {
                log_queue_.pop();
            }
        }
    }
    static char *get_string() {
        static char buf[65];
        communication_.receive_string(buf);
        return buf;
    }

    static Communication communication_;
    static Actuator actuator_;
    static std::queue<std::string> log_queue_;
    static ParameterAPI api;
};

#include "../st_device.h"
extern "C" void SystemClock_Config();

struct SystemInitClass {
 public:
    SystemInitClass() {
        HAL_Init();
        SystemClock_Config();
    }
};

extern "C" {
#endif // __cplusplus

void system_init();
void system_run();
void main_loop_interrupt();
void fast_loop_interrupt();
void usb_interrupt();

#ifdef __cplusplus
}
#endif
