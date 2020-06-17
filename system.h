#pragma once

#ifdef __cplusplus
#include "parameter_api.h"
#include <queue>
#include <string>

extern uint32_t t_exec_fastloop;
extern uint32_t t_exec_mainloop;
extern uint32_t t_period_fastloop;
extern uint32_t t_period_mainloop;

template<typename Actuator, typename Communication>
class System {
 public:
    static void run() {
        actuator_.start();

        log("finished startup");

        ParameterAPI api;
        uint32_t cpu_frequency = CPU_FREQUENCY_HZ;
        api.add_api_variable("kp", new APIFloat(&actuator_.main_loop_.controller_.kp_));
        api.add_api_variable("kd", new APIFloat(&actuator_.main_loop_.controller_.kd_));
        api.add_api_variable("cpu_frequency", new APIUint32(&cpu_frequency));
        api.add_api_variable("t_exec_fastloop", new APIUint32(&t_exec_fastloop));
        api.add_api_variable("t_exec_mainloop", new APIUint32(&t_exec_mainloop));
        api.add_api_variable("t_period_fastloop", new APIUint32(&t_period_fastloop));
        api.add_api_variable("t_period_mainloop", new APIUint32(&t_period_mainloop));
        api.add_api_variable("vbus", new APIFloat(&actuator_.main_loop_.fast_loop_status_.vbus));
        api.add_api_variable("va", new APIFloat(&actuator_.main_loop_.fast_loop_status_.foc_status.command.v_a));
        api.add_api_variable("vb", new APIFloat(&actuator_.main_loop_.fast_loop_status_.foc_status.command.v_b));
        api.add_api_variable("vc", new APIFloat(&actuator_.main_loop_.fast_loop_status_.foc_status.command.v_c));
        api.add_api_variable("ia", new APIFloat(&actuator_.fast_loop_.foc_command_.measured.i_a));
        api.add_api_variable("ib", new APIFloat(&actuator_.fast_loop_.foc_command_.measured.i_b));
        api.add_api_variable("ic", new APIFloat(&actuator_.fast_loop_.foc_command_.measured.i_c));
        api.add_api_variable("id", new APIFloat(&actuator_.main_loop_.fast_loop_status_.foc_status.measured.i_d));
        api.add_api_variable("i0", new APIFloat(&actuator_.main_loop_.fast_loop_status_.foc_status.measured.i_0));
        api.add_api_variable("ikp", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_->kp_));
        api.add_api_variable("iki", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_->ki_));
        api.add_api_variable("iki_limit", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_->ki_limit_));
        while(1) {
            char *s = System::get_string();
            if (s != NULL) {
                System::log(api.parse_string(s));
            }
            send_log();
            actuator_.maintenance();
        }
    }
    static void main_loop_interrupt() {
        actuator_.main_loop_.update();
    }
    static void fast_loop_interrupt() {
        actuator_.fast_loop_.update();
    }
    static void usb_interrupt() {
        usb_.interrupt();
    }
    static void log(std::string str) {
        if (log_queue_.size() < 10) {
            log_queue_.push(str);
        }
    }
    static void send_log() {
        if (!usb_.tx_active(1) && !log_queue_.empty()) {
            std::string str = log_queue_.front();
            log_queue_.pop();
            usb_.send_data(1, (const uint8_t *) str.c_str(), str.size()+1, false);
        }
    }
    static char *get_string() {
        static char buf[64];
        int count = usb_.receive_data(1, (uint8_t *) buf, 64);
        buf[count] = 0;
        if (count) {
            return buf;
        } else {
            return NULL;
        }
    }
// private:
    static Communication usb_;
    static Actuator actuator_;
    static std::queue<std::string> log_queue_;
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
