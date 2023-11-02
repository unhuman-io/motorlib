#ifndef UNHUMAN_MOTORLIB_SYSTEM_H_
#define UNHUMAN_MOTORLIB_SYSTEM_H_

#ifdef __cplusplus
#include "parameter_api.h"
#include "logger.h"
#include "round_robin_logger.h"


extern uint32_t t_exec_fastloop;
extern uint32_t t_exec_mainloop;
extern uint32_t t_period_fastloop;
extern uint32_t t_period_mainloop;

void system_maintenance();


class System {
 public:
    static void run() {
        // check parameter version
        if (GIT_HASH != std::string(param->git_hash)) {
            logger.log_printf("param version error, firmware: %s, param: %s", GIT_HASH, param->git_hash);
            actuator_.main_loop_.led_.set_color(LED::RED);
            actuator_.main_loop_.led_.set_mode(LED::BLINKING);
            while(1) {
                go_to_bootloader = 0xB007;
                NVIC_SystemReset();
            }
        } else {
            logger.log_printf("param version match: %s", GIT_HASH);
        }
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
        api.add_api_variable("tracking_tol", new APIFloat(&actuator_.main_loop_.position_controller_.tracking_tolerance_));
        api.add_api_variable("vlimit", new APIFloat(&actuator_.main_loop_.position_controller_.velocity_limit_));
        API_ADD_FILTER(desired_filter, SecondOrderLowPassFilter, actuator_.main_loop_.position_controller_.desired_filter_);
        api.add_api_variable("error", new const APIFloat(&actuator_.main_loop_.position_controller_.controller_.error_));
        API_ADD_FILTER(velocity_filter, SecondOrderLowPassFilter, actuator_.main_loop_.position_controller_.controller_.velocity_filter_);
        API_ADD_FILTER(output_filter, FirstOrderLowPassFilter, actuator_.main_loop_.position_controller_.controller_.output_filter_);
        api.add_api_variable("vkp", new APIFloat(&actuator_.main_loop_.velocity_controller_.controller_.kp_));
        api.add_api_variable("vki", new APIFloat(&actuator_.main_loop_.velocity_controller_.controller_.ki_));
        api.add_api_variable("vki_limit", new APIFloat(&actuator_.main_loop_.velocity_controller_.controller_.ki_limit_));
        api.add_api_variable("vmax", new APIFloat(&actuator_.main_loop_.velocity_controller_.controller_.command_max_));
        api.add_api_variable("vacceleration_limit", new APIFloat(&actuator_.main_loop_.velocity_controller_.acceleration_limit_));
        api.add_api_variable("jmax", new APIFloat(&actuator_.main_loop_.joint_position_controller_.velocity_controller_.controller_.command_max_));
        api.add_api_variable("jki_limit", new APIFloat(&actuator_.main_loop_.joint_position_controller_.velocity_controller_.controller_.ki_limit_));
        API_ADD_FILTER(vfilt, FirstOrderLowPassFilter, actuator_.main_loop_.velocity_controller_.velocity_filter_);
        API_ADD_FILTER(voutput_filt, FirstOrderLowPassFilter, actuator_.main_loop_.velocity_controller_.controller_.output_filter_);
        api.add_api_variable("cpu_frequency", new APIUint32(&cpu_frequency));
        api.add_api_variable("t_exec_fastloop", new APIUint32(&t_exec_fastloop));
        api.add_api_variable("t_exec_mainloop", new APIUint32(&t_exec_mainloop));
        api.add_api_variable("t_period_fastloop", new APIUint32(&t_period_fastloop));
        api.add_api_variable("t_period_mainloop", new APIUint32(&t_period_mainloop));
        api.add_api_variable("vbus", new APIFloat(&actuator_.main_loop_.status_.fast_loop.vbus));
        api.add_api_variable("phase_mode", new APICallbackUint8([](){ return actuator_.fast_loop_.get_phase_mode(); }, [](uint8_t p){ actuator_.fast_loop_.set_phase_mode(p); }));
        api.add_api_variable("va", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_a));
        api.add_api_variable("vb", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_b));
        api.add_api_variable("vc", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_c));
        api.add_api_variable("vq", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_q));
        api.add_api_variable("vd", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_d));
        api.add_api_variable("ia", new APIFloat(&actuator_.fast_loop_.foc_command_.measured.i_a));
        api.add_api_variable("ib", new APIFloat(&actuator_.fast_loop_.foc_command_.measured.i_b));
        api.add_api_variable("ic", new APIFloat(&actuator_.fast_loop_.foc_command_.measured.i_c));
        api.add_api_variable("id", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_d));
        api.add_api_variable("iq", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_q));
        api.add_api_variable("i0", new APIFloat(&actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_0));
        api.add_api_variable("ikp", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_.kp_));
        api.add_api_variable("iki", new APICallbackFloat([](){ return actuator_.fast_loop_.foc_->pi_iq_.ki_; },
            [](float f){ if (f == 0) { actuator_.fast_loop_.foc_->pi_iq_.ki_sum_ = 0; } actuator_.fast_loop_.foc_->pi_iq_.ki_ = f; }));
        api.add_api_variable("iki_limit", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_.ki_limit_));
        api.add_api_variable("imax", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_.command_max_));
        api.add_api_variable("idkp", new APIFloat(&actuator_.fast_loop_.foc_->pi_id_.kp_));
        api.add_api_variable("idki", new APICallbackFloat([](){ return actuator_.fast_loop_.foc_->pi_id_.ki_; },
            [](float f){ if (f == 0) { actuator_.fast_loop_.foc_->pi_id_.ki_sum_ = 0; } actuator_.fast_loop_.foc_->pi_id_.ki_ = f; }));
        api.add_api_variable("idki_limit", new APIFloat(&actuator_.fast_loop_.foc_->pi_id_.ki_limit_));
        api.add_api_variable("idmax", new APIFloat(&actuator_.fast_loop_.foc_->pi_id_.command_max_));
        api.add_api_variable("idiq", new APICallbackFloat([](){return 0;}, 
            [](float f){actuator_.fast_loop_.foc_->pi_id_.kp_ = actuator_.fast_loop_.foc_->pi_iq_.kp_;
                actuator_.fast_loop_.foc_->pi_id_.ki_ = actuator_.fast_loop_.foc_->pi_iq_.ki_;
                actuator_.fast_loop_.foc_->pi_id_.kp2_ = actuator_.fast_loop_.foc_->pi_iq_.kp2_;
                actuator_.fast_loop_.foc_->pi_id_.ki2_ = actuator_.fast_loop_.foc_->pi_iq_.ki2_;
                actuator_.fast_loop_.foc_->pi_id_.ki_limit_ = actuator_.fast_loop_.foc_->pi_iq_.ki_limit_;
                actuator_.fast_loop_.foc_->pi_id_.command_max_ = actuator_.fast_loop_.foc_->pi_iq_.command_max_;
                actuator_.fast_loop_.foc_->set_id_limit(actuator_.fast_loop_.foc_->get_iq_limit()); }));
        actuator_.main_loop_.torque_controller_.set_debug_variables(api);
        actuator_.main_loop_.state_controller_.set_debug_variables(api);
        api.add_api_variable("tgain", new APIFloat(&actuator_.main_loop_.torque_sensor_.gain_));
        api.add_api_variable("tbias", new APIFloat(&actuator_.main_loop_.torque_sensor_.bias_));
        api.add_api_variable("torque", new const APIFloat(&actuator_.main_loop_.torque_sensor_.torque_));
        api.add_api_variable("t_i_correction", new const APIFloat(&actuator_.main_loop_.param_.torque_correction));
        api.add_api_variable("log", new APICallback(get_log, log));
        api.add_api_variable("messages_version", new APICallback([](){ return MOTOR_MESSAGES_VERSION; }, [](std::string s) {} ));
        api.add_api_variable("index_pos", new APICallback([](){ return std::to_string(actuator_.fast_loop_.encoder_.get_index_pos()); }, [](std::string s) {}));
        api.add_api_variable("index_received", new APICallbackUint32([](){return actuator_.fast_loop_.encoder_.index_received();}, [](uint32_t u) {}));
        api.add_api_variable("index_offset_measured", new const APIFloat(&actuator_.fast_loop_.motor_index_electrical_offset_measured_));
        api.add_api_variable("electrical_zero_pos", new APIInt32(&actuator_.fast_loop_.motor_electrical_zero_pos_));
        api.add_api_variable("mcpr", new const APIUint32(&param->fast_loop_param.motor_encoder.cpr));
        api.add_api_variable("ocpr", new const APIFloat(&param->main_loop_param.output_encoder.cpr));
        api.add_api_variable("irange", new const APICallbackFloat([](){ return 2048*param->fast_loop_param.adc1_gain; }));
        api.add_api_variable("stack_free", new const APICallbackUint32(get_stack_free));
        api.add_api_variable("stack_used", new const APICallbackUint32(get_stack_used));
        api.add_api_variable("heap_free", new const APICallbackUint32(get_heap_free));
        api.add_api_variable("heap_used", new const APICallbackUint32(get_heap_used));

        api.add_api_variable("vbus_min", new APIFloat(&actuator_.main_loop_.vbus_min_));
        api.add_api_variable("vbus_max", new APIFloat(&actuator_.main_loop_.vbus_max_));
        api.add_api_variable("ia_bias", new APIFloat(&actuator_.fast_loop_.ia_bias_));
        api.add_api_variable("ib_bias", new APIFloat(&actuator_.fast_loop_.ib_bias_));
        api.add_api_variable("ic_bias", new APIFloat(&actuator_.fast_loop_.ic_bias_));
        api.add_api_variable("power", new const APIFloat(&actuator_.main_loop_.status_.fast_loop.power));
        api.add_api_variable("power_avg", new const APIFloat(&actuator_.main_loop_.status_.power));
        api.add_api_variable("energy", new const APIUint32(&actuator_.main_loop_.status_.fast_loop.energy_uJ));
        api.add_api_variable("fast_log", new const APICallback([](){
            static uint8_t fast_state = 0;
            actuator_.main_loop_.lock_status_log();
            FastLog log;
            std::string out;
            for(int i=0; i<10; i++) {
                FastLoopStatus &status = actuator_.fast_loop_.status_log_.next();
                log.timestamp = status.timestamp;
                log.measured_motor_position = status.foc_command.measured.motor_encoder;
                log.command_iq = status.foc_status.command.i_q;
                log.measured_iq = status.foc_status.measured.i_q;
                log.measured_ia = status.foc_command.measured.i_a;
                log.measured_ib = status.foc_command.measured.i_b;
                log.measured_ic = status.foc_command.measured.i_c;
                log.command_va = status.foc_status.command.v_a;
                log.command_vb = status.foc_status.command.v_b;
                log.command_vc = status.foc_status.command.v_c;
                log.vbus = status.vbus;
                std::string s((char *) &log, sizeof(log));
                actuator_.fast_loop_.status_log_.finish();
                out += s;
                
            }
            fast_state++;
            if (fast_state > 9) {
                fast_state = 0;
                actuator_.main_loop_.unlock_status_log();
            }
            return out; }));
        api.add_api_variable("beep", new APICallbackFloat([](){ return 0; }, [](float f){ actuator_.fast_loop_.beep_on(f); }));
        api.add_api_variable("beep_frequency", new APIFloat(&actuator_.fast_loop_.param_.beep_frequency));
        api.add_api_variable("beep_amplitude", new APIFloat(&actuator_.fast_loop_.param_.beep_amplitude));
        api.add_api_variable("zero_current_sensors", new APICallbackFloat([](){ return 0; }, [](float f){ actuator_.fast_loop_.zero_current_sensors_on(f); }));
        api.add_api_variable("disable_safe_mode", new const APICallback([](){ actuator_.main_loop_.error_mask_.all = ERROR_MASK_NONE; return "ok"; }));
        api.add_api_variable("error_mask", new APICallback([](){ return u32_to_hex(actuator_.main_loop_.error_mask_.all); },
                [](std::string s){ try {
                        actuator_.main_loop_.error_mask_.all = std::stoul(s, nullptr, 16) & ERROR_MASK_ALL;}
                    catch(...) {} }));
        api.add_api_variable("help", new const APICallback([](){ return api.get_all_api_variables(); }));
        api.add_api_variable("api_length", new const APICallbackUint16([](){ return api.get_api_length(); }));
        api.add_api_variable("disable_position_limits", new APIBool(&actuator_.main_loop_.position_limits_disable_));
        api.add_api_variable("jkpj", new APIFloat(&actuator_.main_loop_.joint_position_controller_.param_.kpj));
        api.add_api_variable("motor_position_raw", new const APIFloat(&actuator_.fast_loop_.motor_position_));
        api.add_api_variable("obias", new APIFloat(&actuator_.main_loop_.output_encoder_bias_));
        api.add_api_variable("mbias", new APIFloat(&actuator_.main_loop_.motor_encoder_bias_));
        api.add_api_variable("ttgain", new const APIFloat(&actuator_.main_loop_.param_.torque_sensor.table_gain));
        API_ADD_FILTER(id_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.foc_->id_filter_);
        API_ADD_FILTER(iq_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.foc_->iq_filter_);
        API_ADD_FILTER(output_iq_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.iq_filter_);
        API_ADD_FILTER(output_motor_velocity_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.motor_velocity_filter_);
        API_ADD_FILTER(output_motor_position_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.motor_position_filter_);
        api.add_api_variable("startup_phase_lock_current", new const APIFloat(&param->startup_param.phase_lock_current));
        api.add_api_variable("startup_mbias", new APIFloat(&actuator_.startup_motor_bias_));
        api.add_api_variable("set_startup_bias", new const APICallback([](){ actuator_.set_bias(); return "ok"; }));
        api.add_api_variable("odir", new APIFloat(&actuator_.main_loop_.output_encoder_dir_));
        api.add_api_variable("tdir", new APIFloat(&actuator_.main_loop_.torque_sensor_dir_));
        api.add_api_variable("mdir", new APIFloat(&actuator_.fast_loop_.motor_encoder_dir_));
        //API_ADD_FILTER(output_motor_velocity_filter2, FirstOrderLowPassFilter, actuator_.main_loop_.motor_velocity_filter_);
        API_ADD_FILTER(output_motor_position_filter2, FirstOrderLowPassFilter, actuator_.main_loop_.motor_position_filter_);
        //API_ADD_FILTER(output_output_velocity_filter, FirstOrderLowPassFilter, actuator_.main_loop_.output_velocity_filter_);
        API_ADD_FILTER(output_output_position_filter, FirstOrderLowPassFilter, actuator_.main_loop_.output_position_filter_);
        API_ADD_FILTER(output_torque_filter, FirstOrderLowPassFilter, actuator_.main_loop_.torque_filter_);
        api.add_api_variable("idir", new APIFloat(&actuator_.fast_loop_.current_direction_));
        api.add_api_variable("uptime", new const APICallbackUint32(get_uptime));
        api.add_api_variable("menc", new const APIInt32(&actuator_.fast_loop_.motor_enc));
        api.add_api_variable("oenc", new const APICallbackInt32([](){ return actuator_.main_loop_.output_encoder_.get_value(); }));
        api.add_api_variable("amax", new APIFloat(&actuator_.main_loop_.admittance_controller_.torque_controller_.command_max_));
        api.add_api_variable("akp", new APIFloat(&actuator_.main_loop_.admittance_controller_.torque_controller_.kp_));
        api.add_api_variable("Tmotor_est", new const APIFloat(&actuator_.main_loop_.status_.motor_temperature_estimate));
        API_ADD_FILTER(a_output_filter, FirstOrderLowPassFilter, actuator_.main_loop_.admittance_controller_.torque_controller_.output_filter_);
        api.add_api_variable("fast_loop_status", new const APICallback([](){ 
            FastLoopStatus status = actuator_.fast_loop_.status_.top();
            uint8_t len = 192;
            char c[len];
            std::snprintf(c, len, "%ld, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
                    status.timestamp,
                    status.foc_command.measured.motor_encoder,
                    status.foc_command.desired.i_q,
                    status.foc_status.measured.i_q,
                    status.foc_command.measured.i_a,
                    status.foc_command.measured.i_b,
                    status.foc_command.measured.i_c,
                    status.foc_status.command.v_a,
                    status.foc_status.command.v_b,
                    status.foc_status.command.v_c,
                    status.vbus);
            std::string s(c);
            return s;
        }));
        api.add_api_variable("ikp2", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_.kp2_));
        api.add_api_variable("iki2", new APIFloat(&actuator_.fast_loop_.foc_->pi_iq_.ki2_));
        api.add_api_variable("idkp2", new APIFloat(&actuator_.fast_loop_.foc_->pi_id_.kp2_));
        api.add_api_variable("idki2", new APIFloat(&actuator_.fast_loop_.foc_->pi_id_.ki2_));
        api.add_api_variable("ivalue2", new APICallbackFloat([](){ return actuator_.fast_loop_.foc_->pi_iq_.value2_; },
             [](float f){ 
                PI2Param param = actuator_.fast_loop_.foc_->pi_iq_.get_param();
                param.value2 = f;
                actuator_.fast_loop_.foc_->pi_iq_.set_param(param);
        }));
        api.add_api_variable("idvalue2", new APICallbackFloat([](){ return actuator_.fast_loop_.foc_->pi_id_.value2_; },
             [](float f){ 
                PI2Param param = actuator_.fast_loop_.foc_->pi_id_.get_param();
                param.value2 = f;
                actuator_.fast_loop_.foc_->pi_id_.set_param(param);
        }));
        api.add_api_variable("id_des", new APIFloat(&actuator_.fast_loop_.foc_command_.desired.i_d));
        api.add_api_variable("trigger_fast_log", new const APICallback([](){ actuator_.fast_loop_.trigger_status_log(); return "triggered"; }));
        api.add_api_variable("ilimit", new APICallbackFloat([](){ return actuator_.fast_loop_.foc_->get_iq_limit(); },
            [](float f){ actuator_.fast_loop_.foc_->set_iq_limit(f); }));
        api.add_api_variable("idlimit", new APICallbackFloat([](){ return actuator_.fast_loop_.foc_->get_id_limit(); },
            [](float f){ actuator_.fast_loop_.foc_->set_id_limit(f); }));
        api.add_api_variable("num_poles", new APIFloat(&actuator_.fast_loop_.foc_->num_poles_));
        api.add_api_variable("timestamp", new const APICallbackUint32(get_clock));
        api.add_api_variable("mrollover", new const APICallbackFloat([](){ return actuator_.fast_loop_.get_rollover(); }));
        api.add_api_variable("gear_ratio", new const APIFloat(&param->startup_param.gear_ratio));
        api.add_api_variable("version", new const APICallback([](){ return GIT_VERSION; }));
        api.add_api_variable("git_sha", new const APICallback([](){ return GIT_HASH; }));
        api.add_api_variable("motorlib_sha", new const APICallback([](){ return MOTORLIB_HASH; }));
        api.add_api_variable("name", new const APICallback([](){ return param->name; }));
        uint32_t api_timeout_us = 10000;
        api.add_api_variable("api_timeout", new APIUint32(&api_timeout_us));


        uint32_t t_start = get_clock();
        while(1) {
            count_++;
            if (communication_.send_string_active() && get_clock() - t_start > US_TO_CPU(api_timeout_us)) {
                communication_.cancel_send_string();
            }
            char *s = System::get_string();
            if (s[0] != 0) {
                auto response = api.parse_string(s);
                communication_.send_string(response.c_str(), response.length());
                t_start = get_clock();
            }
            system_maintenance();
            actuator_.maintenance();
            round_robin_logger.log_data(UPTIME_INDEX, get_uptime());
        }
    }
    static void main_loop_interrupt() {
        actuator_.main_loop_.update();
    }
    static void fast_loop_interrupt() {
        actuator_.fast_loop_.update();
    }
    static void log(std::string str) {
        logger.log(str);
    }
    static std::string get_log() {
        return logger.get_log();
    }

    static char *get_string() {
        static char buf[65];
        communication_.receive_string(buf);
        return buf;
    }

    static Communication communication_;
    static Actuator actuator_;
    static ParameterAPI api;
    static uint32_t count_;
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

#endif  // UNHUMAN_MOTORLIB_SYSTEM_H_
