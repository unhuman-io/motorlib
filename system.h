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

#define API_ADD_FILTER(name, type, location) \
    std::function<void(float)> set_filt_##name = std::bind(&type::set_frequency, &location, std::placeholders::_1); \
    std::function<float(void)> get_filt_##name = std::bind(&type::get_frequency, &location); \
    api.add_api_variable(#name, new APICallbackFloat(get_filt_##name, set_filt_##name))


class System {
 public:
    static void run() {
        actuator_.start();

        log("finished startup");

        uint32_t cpu_frequency = CPU_FREQUENCY_HZ;
        api.add_api_variable("system_count", new APIUint32((uint32_t *) &count_));
        api.add_api_variable("mode", new APIUint32((uint32_t *) &actuator_.main_loop_.mode_));
        api.add_api_variable("kp", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.kp_));
        api.add_api_variable("kd", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.kd_));
        api.add_api_variable("ki", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.ki_));
        api.add_api_variable("ki_limit", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.ki_limit_));
        api.add_api_variable("max", new APIFloat(&actuator_.main_loop_.position_controller_.controller_.command_max_));
        API_ADD_FILTER(velocity_filter, SecondOrderLowPassFilter, actuator_.main_loop_.position_controller_.controller_.error_dot_filter_);
        API_ADD_FILTER(output_filter, FirstOrderLowPassFilter, actuator_.main_loop_.position_controller_.controller_.output_filter_);
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
        api.add_api_variable("tki", new APIFloat(&actuator_.main_loop_.torque_controller_.controller_.ki_));
        api.add_api_variable("tki_limit", new APIFloat(&actuator_.main_loop_.torque_controller_.controller_.ki_limit_));
        API_ADD_FILTER(t_velocity_filter, SecondOrderLowPassFilter, actuator_.main_loop_.torque_controller_.controller_.error_dot_filter_);
        API_ADD_FILTER(t_output_filter, FirstOrderLowPassFilter, actuator_.main_loop_.torque_controller_.controller_.output_filter_);
        api.add_api_variable("tmax", new APIFloat(&actuator_.main_loop_.torque_controller_.controller_.command_max_));
        api.add_api_variable("tgain", new APIFloat(&actuator_.main_loop_.torque_sensor_.gain_));
        api.add_api_variable("tbias", new APIFloat(&actuator_.main_loop_.torque_sensor_.bias_));
        api.add_api_variable("t_i_correction", new APIFloat(&actuator_.main_loop_.param_.torque_correction));
        api.add_api_variable("log", new APICallback(get_log, log));
        api.add_api_variable("messages_version", new APICallback([](){ return MOTOR_MESSAGES_VERSION; }, [](std::string s) {} ));

        while(1) {
            count_++;
            char *s = System::get_string();
            if (s[0] != 0) {
                auto response = api.parse_string(s);
                communication_.send_string(response.c_str(), response.length());
            }
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
    static std::string get_log() {
        std::string str = "log end";
        if (!log_queue_.empty()) {
            str = log_queue_.front();
            log_queue_.pop();
        }
        return str;
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
    static uint32_t count_;
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
