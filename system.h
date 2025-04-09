#ifndef UNHUMAN_MOTORLIB_SYSTEM_H_
#define UNHUMAN_MOTORLIB_SYSTEM_H_

#ifdef __cplusplus
#include "parameter_api.h"
#include "logger.h"
#include "round_robin_logger.h"
#include "otp.h"
#include "peripheral/stm32_serial.h"


extern uint32_t t_exec_fastloop;
extern uint32_t t_exec_mainloop;
extern uint32_t t_period_fastloop;
extern uint32_t t_period_mainloop;

void system_maintenance();
void main_maintenance();

#ifndef TOGGLE_SCOPE_PIN
#define TOGGLE_SCOPE_PIN(X,x)
#endif


class System {
 public:
    static void run() {
        {
            char t[18];
            RTClock::power_on_date_time(t);
            logger.log(t);
        }
        // check parameter version
        if (OBOT_HASH != std::string(param->obot_hash)) {
            logger.log_printf("param version error, firmware: %s, param: %s", OBOT_HASH, param->obot_hash);
            actuator_.main_loop_.led_.set_color(LED::RED);
            actuator_.main_loop_.led_.set_mode(LED::BLINKING);
            while(1) {
                go_to_bootloader = 0xB007;
                NVIC_SystemReset();
            }
        } else {
            logger.log_printf("param version match: %s", OBOT_HASH);
        }

        actuator_.start();

        log("finished startup");

        uint32_t cpu_frequency = CPU_FREQUENCY_HZ;
        static APIUint32 api_system_count((uint32_t *) &count_);
        api.add_api_variable("system_count", &api_system_count);
        static APIUint32 api_mode((uint32_t *) &actuator_.main_loop_.mode_);
        api.add_api_variable("mode", &api_mode);
        static APIFloat api_kp(&actuator_.main_loop_.position_controller_.controller_.kp_);
        api.add_api_variable("kp", &api_kp);
        static APIFloat api_kd(&actuator_.main_loop_.position_controller_.controller_.kd_);
        api.add_api_variable("kd", &api_kd);
        static APIFloat api_ki(&actuator_.main_loop_.position_controller_.controller_.ki_);
        api.add_api_variable("ki", &api_ki);
        static APIFloat api_ki_limit(&actuator_.main_loop_.position_controller_.controller_.ki_limit_);
        api.add_api_variable("ki_limit", &api_ki_limit);
        static APIFloat api_max(&actuator_.main_loop_.position_controller_.controller_.command_max_);
        api.add_api_variable("max", &api_max);
        static APIFloat api_tracking_tol(&actuator_.main_loop_.position_controller_.tracking_tolerance_);
        api.add_api_variable("tracking_tol", &api_tracking_tol);
        static APIFloat api_vlimit(&actuator_.main_loop_.position_controller_.velocity_limit_);
        api.add_api_variable("vlimit", &api_vlimit);
        API_ADD_FILTER(desired_filter, SecondOrderLowPassFilter, actuator_.main_loop_.position_controller_.desired_filter_);
        static const APIFloat api_error(&actuator_.main_loop_.position_controller_.controller_.error_);
        api.add_api_variable("error", &api_error);
        API_ADD_FILTER(velocity_filter, SecondOrderLowPassFilter, actuator_.main_loop_.position_controller_.controller_.velocity_filter_);
        API_ADD_FILTER(output_filter, FirstOrderLowPassFilter, actuator_.main_loop_.position_controller_.controller_.output_filter_);
        static APIFloat api_vkp(&actuator_.main_loop_.velocity_controller_.controller_.kp_);
        api.add_api_variable("vkp", &api_vkp);
        static APIFloat api_vki(&actuator_.main_loop_.velocity_controller_.controller_.ki_);
        api.add_api_variable("vki", &api_vki);
        static APIFloat api_vki_limit(&actuator_.main_loop_.velocity_controller_.controller_.ki_limit_);
        api.add_api_variable("vki_limit", &api_vki_limit);
        static APIFloat api_vmax(&actuator_.main_loop_.velocity_controller_.controller_.command_max_);
        api.add_api_variable("vmax", &api_vmax);
        static APIFloat api_vacceleration_limit(&actuator_.main_loop_.velocity_controller_.acceleration_limit_);
        api.add_api_variable("vacceleration_limit", &api_vacceleration_limit);
        static APIFloat api_jmax(&actuator_.main_loop_.joint_position_controller_.velocity_controller_.controller_.command_max_);
        api.add_api_variable("jmax", &api_jmax);
        static APIFloat api_jki_limit(&actuator_.main_loop_.joint_position_controller_.velocity_controller_.controller_.ki_limit_);
        api.add_api_variable("jki_limit", &api_jki_limit);
        API_ADD_FILTER(vfilt, FirstOrderLowPassFilter, actuator_.main_loop_.velocity_controller_.velocity_filter_);
        API_ADD_FILTER(voutput_filt, FirstOrderLowPassFilter, actuator_.main_loop_.velocity_controller_.controller_.output_filter_);
        static APIUint32 api_cpu_frequency(&cpu_frequency);
        api.add_api_variable("cpu_frequency", &api_cpu_frequency);
        static APIUint32 api_t_exec_fastloop(&t_exec_fastloop);
        api.add_api_variable("t_exec_fastloop", &api_t_exec_fastloop);
        static APIUint32 api_t_exec_mainloop(&t_exec_mainloop);
        api.add_api_variable("t_exec_mainloop", &api_t_exec_mainloop);
        static APIUint32 api_t_period_fastloop(&t_period_fastloop);
        api.add_api_variable("t_period_fastloop", &api_t_period_fastloop);
        static APIUint32 api_t_period_mainloop(&t_period_mainloop);
        api.add_api_variable("t_period_mainloop", &api_t_period_mainloop);
        static APIFloat api_vbus(&actuator_.main_loop_.status_.fast_loop.vbus);
        api.add_api_variable("vbus", &api_vbus);
        static APICallbackUint8 api_phase_mode([](){ return actuator_.fast_loop_.get_phase_mode(); }, [](uint8_t p){ actuator_.fast_loop_.set_phase_mode(p); });
        api.add_api_variable("phase_mode", &api_phase_mode);
        static APIFloat api_va(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_a);
        api.add_api_variable("va", &api_va);
        static APIFloat api_vb(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_b);
        api.add_api_variable("vb", &api_vb);
        static APIFloat api_vc(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_c);
        api.add_api_variable("vc", &api_vc);
        static APIFloat api_vq(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_q);
        api.add_api_variable("vq", &api_vq);
        static APIFloat api_vd(&actuator_.main_loop_.status_.fast_loop.foc_status.command.v_d);
        api.add_api_variable("vd", &api_vd);
        static APIFloat api_ia(&actuator_.fast_loop_.foc_command_.measured.i_a);
        api.add_api_variable("ia", &api_ia);
        static APIFloat api_ib(&actuator_.fast_loop_.foc_command_.measured.i_b);
        api.add_api_variable("ib", &api_ib);
        static APIFloat api_ic(&actuator_.fast_loop_.foc_command_.measured.i_c);
        api.add_api_variable("ic", &api_ic);
        static APIFloat api_id(&actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_d);
        api.add_api_variable("id", &api_id);
        static APIFloat api_iq(&actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_q);
        api.add_api_variable("iq", &api_iq);
        static APIFloat api_i0(&actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_0);
        api.add_api_variable("i0", &api_i0);
        static APIFloat api_ikp(&actuator_.fast_loop_.foc_->pi_iq_.kp_);
        api.add_api_variable("ikp", &api_ikp);
        static APICallbackFloat api_iki([](){ return actuator_.fast_loop_.foc_->pi_iq_.ki_; },
            [](float f){ if (f == 0) { actuator_.fast_loop_.foc_->pi_iq_.ki_sum_ = 0; } actuator_.fast_loop_.foc_->pi_iq_.ki_ = f; });
        api.add_api_variable("iki", &api_iki);
        static APIFloat api_iki_limit(&actuator_.fast_loop_.foc_->pi_iq_.ki_limit_);
        api.add_api_variable("iki_limit", &api_iki_limit);
        static APIFloat api_imax(&actuator_.fast_loop_.foc_->pi_iq_.command_max_);
        api.add_api_variable("imax", &api_imax);
        static APIFloat api_idkp(&actuator_.fast_loop_.foc_->pi_id_.kp_);
        api.add_api_variable("idkp", &api_idkp);
        static APICallbackFloat api_idki([](){ return actuator_.fast_loop_.foc_->pi_id_.ki_; },
            [](float f){ if (f == 0) { actuator_.fast_loop_.foc_->pi_id_.ki_sum_ = 0; } actuator_.fast_loop_.foc_->pi_id_.ki_ = f; });
        api.add_api_variable("idki", &api_idki);
        static APIFloat api_idki_limit(&actuator_.fast_loop_.foc_->pi_id_.ki_limit_);
        api.add_api_variable("idki_limit", &api_idki_limit);
        static APIFloat api_idmax(&actuator_.fast_loop_.foc_->pi_id_.command_max_);
        api.add_api_variable("idmax", &api_idmax);
        static APIFloat api_icmax(&actuator_.fast_loop_.foc_->param_.voltage_limit);
        api.add_api_variable("icmax", &api_icmax);
        static const APICallback api_idiq([]{
                actuator_.fast_loop_.foc_->pi_id_.kp_ = actuator_.fast_loop_.foc_->pi_iq_.kp_;
                actuator_.fast_loop_.foc_->pi_id_.ki_ = actuator_.fast_loop_.foc_->pi_iq_.ki_;
                actuator_.fast_loop_.foc_->pi_id_.ki_limit_ = actuator_.fast_loop_.foc_->pi_iq_.ki_limit_;
                actuator_.fast_loop_.foc_->pi_id_.command_max_ = actuator_.fast_loop_.foc_->pi_iq_.command_max_;
                actuator_.fast_loop_.foc_->set_id_limit(actuator_.fast_loop_.foc_->get_iq_limit());
                return std::string("ok"); });
        api.add_api_variable("idiq", &api_idiq);
        TORQUE_CONTROLLER_DEBUG_VARIABLES(api, actuator_.main_loop_.torque_controller_);
        STATE_CONTROLLER_DEBUG_VARIABLES(api, actuator_.main_loop_.state_controller_);
        static APIFloat api_tgain(&actuator_.main_loop_.torque_sensor_.gain_);
        api.add_api_variable("tgain", &api_tgain);
        static APIFloat api_tbias(&actuator_.main_loop_.torque_sensor_bias_);
        api.add_api_variable("tbias", &api_tbias);
        static const APIFloat api_torque(&actuator_.main_loop_.status_.torque);
        api.add_api_variable("torque", &api_torque);
        static const APIFloat api_t_i_correction(&actuator_.main_loop_.param_.torque_correction);
        api.add_api_variable("t_i_correction", &api_t_i_correction);
        static APICallback api_log(get_log, log);
        api.add_api_variable("log", &api_log);
        static const APICallback api_old_log([]{ return logger.get_old_log(); });
        api.add_api_variable("old_log", &api_old_log);
        static const APICallback api_log_reset([]()->std::string{ logger.reset_read_front(); return "ok"; });
        api.add_api_variable("log_reset", &api_log_reset);
        static const APICallbackUint32 api_log_num([]{ return logger.num_elements(); });
        api.add_api_variable("log_num", &api_log_num);
        static const APIStringView api_messages_version(MOTOR_MESSAGES_VERSION);
        api.add_api_variable("messages_version", &api_messages_version);
        static const APICallbackInt32 api_index_pos([]{ return actuator_.fast_loop_.encoder_.get_index_pos(); });
        api.add_api_variable("index_pos", &api_index_pos);
        static const APICallbackUint8 api_index_received([]()->uint8_t{return actuator_.fast_loop_.encoder_.index_received();});
        api.add_api_variable("index_received", &api_index_received);
        static const APIFloat api_index_offset_measured(&actuator_.fast_loop_.motor_index_electrical_offset_measured_);
        api.add_api_variable("index_offset_measured", &api_index_offset_measured);
        static APIInt32 api_electrical_zero_pos(&actuator_.fast_loop_.motor_electrical_zero_pos_);
        api.add_api_variable("electrical_zero_pos", &api_electrical_zero_pos);
        static const APIUint32 api_mcpr(&param->fast_loop_param.motor_encoder.cpr);
        api.add_api_variable("mcpr", &api_mcpr);
        static const APIFloat api_ocpr(&param->main_loop_param.output_encoder.cpr);
        api.add_api_variable("ocpr", &api_ocpr);
        static const APICallbackFloat api_irange([](){ return 2048*param->fast_loop_param.adc1_gain; });
        api.add_api_variable("irange", &api_irange);
        static const APICallbackUint32 api_stack_free(get_stack_free);
        api.add_api_variable("stack_free", &api_stack_free);
        static const APICallbackUint32 api_stack_used(get_stack_used);
        api.add_api_variable("stack_used", &api_stack_used);
        static const APICallbackUint32 api_heap_free(get_heap_free);
        api.add_api_variable("heap_free", &api_heap_free);
        static const APICallbackUint32 api_heap_used(get_heap_used);
        api.add_api_variable("heap_used", &api_heap_used);
        static const APICallbackUint32 api_heap_current_free(get_current_heap_free);
        api.add_api_variable("heap_current_free", &api_heap_current_free);
        static const APICallbackUint32 api_heap_current_used(get_current_heap_used);
        api.add_api_variable("heap_current_used", &api_heap_current_used);
        static APICallbackUint32 api_malloc([](){ return (uint32_t) get_heap_free() + get_heap_used(); }, 
            [](uint32_t u) {
                try { char* volatile c = new char[u]; delete c; }
                catch(...) { logger.log_printf("couldn't allocate %d", u); } });
        api.add_api_variable("malloc", &api_malloc);
        static APIFloat api_vbus_min(&actuator_.main_loop_.vbus_min_);
        api.add_api_variable("vbus_min", &api_vbus_min);
        static APIFloat api_vbus_max(&actuator_.main_loop_.vbus_max_);
        api.add_api_variable("vbus_max", &api_vbus_max);
        static APIFloat api_ia_bias(&actuator_.fast_loop_.ia_bias_);
        api.add_api_variable("ia_bias", &api_ia_bias);
        static APIFloat api_ib_bias(&actuator_.fast_loop_.ib_bias_);
        api.add_api_variable("ib_bias", &api_ib_bias);
        static APIFloat api_ic_bias(&actuator_.fast_loop_.ic_bias_);
        api.add_api_variable("ic_bias", &api_ic_bias);
        static const APIFloat api_power(&actuator_.main_loop_.status_.fast_loop.power);
        api.add_api_variable("power", &api_power);
        static const APIFloat api_power_avg(&actuator_.main_loop_.status_.power);
        api.add_api_variable("power_avg", &api_power_avg);
        static const APIUint32 api_energy(&actuator_.main_loop_.status_.fast_loop.energy_uJ);
        api.add_api_variable("energy", &api_energy);
        static const APICallback api_fast_log([](){
            actuator_.main_loop_.lock_status_log();
            FastLog log;
            std::string out;
            for(int i=0; i<FAST_LOG_LENGTH; i++) {
                FastLoopStatus &status = actuator_.fast_loop_.status_log_.next();
                log.timestamp = status.timestamp;
                log.electrical_position = status.foc_command.measured.motor_encoder / actuator_.fast_loop_.foc_->num_poles_;
                log.command_iq = status.foc_status.command.i_q;
                log.command_id = status.foc_status.command.i_d;
                log.measured_iq = status.foc_status.measured.i_q;
                log.measured_id = status.foc_status.measured.i_d;
                log.command_vq = status.foc_status.command.v_q;
                log.command_vd = status.foc_status.command.v_d;
                log.vbus = status.vbus;
                log.ibus = status.ibus;
                std::string s((char *) &log, sizeof(log));
                actuator_.fast_loop_.status_log_.finish();
                out += s;
            }
            actuator_.main_loop_.unlock_status_log();
            return out;
        });
        api.add_api_variable("fast_log", &api_fast_log);
        static const APICallback api_fast_log2([](){
            actuator_.main_loop_.lock_status_log();
            FastLog2 log;
            std::string out;
            for(int i=0; i<FAST_LOG_LENGTH; i++) {
                FastLoopStatus &status = actuator_.fast_loop_.status_log_.next();
                log.timestamp = status.timestamp;
                log.electrical_position = status.foc_command.measured.motor_encoder / actuator_.fast_loop_.foc_->num_poles_;
                log.measured_ia = status.foc_command.measured.i_a;
                log.measured_ib = status.foc_command.measured.i_b;
                log.measured_ic = status.foc_command.measured.i_c;
                log.command_va = status.foc_status.command.v_a;
                log.command_vb = status.foc_status.command.v_b;
                log.command_vc = status.foc_status.command.v_c;
                log.motor_encoder_flags = status.foc_command.motor_encoder_flags;
                log.mode = status.mode;
                std::string s((char *) &log, sizeof(log));
                actuator_.fast_loop_.status_log_.finish();
                out += s;
            }
            actuator_.main_loop_.unlock_status_log();
            return out;
        });
        api.add_api_variable("fast_log2", &api_fast_log2);
        static const APICallbackFloat api_beep([](){ return 0.0; }, [](float f){ actuator_.fast_loop_.beep_on(f); });
        api.add_api_variable("beep", &api_beep);
        static APIFloat api_beep_frequency(&actuator_.fast_loop_.param_.beep_frequency);
        api.add_api_variable("beep_frequency", &api_beep_frequency);
        static APIFloat api_beep_amplitude(&actuator_.fast_loop_.param_.beep_amplitude);
        api.add_api_variable("beep_amplitude", &api_beep_amplitude);
        static APICallbackFloat api_zero_current_sensors([](){ return 0.0; }, [](float f){ actuator_.fast_loop_.zero_current_sensors_on(f); });
        api.add_api_variable("zero_current_sensors", &api_zero_current_sensors);
        static const APICallback api_disable_safe_mode([]()->std::string{ actuator_.main_loop_.error_mask_.all = ERROR_MASK_NONE; return "ok"; });
        api.add_api_variable("disable_safe_mode", &api_disable_safe_mode);
        static APICallback api_error_mask([](){ return u32_to_hex(actuator_.main_loop_.error_mask_.all); },
                [](std::string s){ try {
                        actuator_.main_loop_.error_mask_.all = std::stoul(s, nullptr, 16) & ERROR_MASK_ALL;}
                    catch(...) {} });
        api.add_api_variable("error_mask", &api_error_mask);
        static const APICallback api_help([](){ return api.get_all_api_variables(); });
        api.add_api_variable("help", &api_help);
        static const APICallbackUint16 api_api_length([](){ return api.get_api_length(); });
        api.add_api_variable("api_length", &api_api_length);
        static APIBool api_disable_position_limits(&actuator_.main_loop_.position_limits_disable_);
        api.add_api_variable("disable_position_limits", &api_disable_position_limits);
        static APIFloat api_jkpj(&actuator_.main_loop_.joint_position_controller_.param_.kpj);
        api.add_api_variable("jkpj", &api_jkpj);
        static const APIFloat api_motor_position_raw(&actuator_.fast_loop_.motor_position_);
        api.add_api_variable("motor_position_raw", &api_motor_position_raw);
        static APIFloat api_obias(&actuator_.main_loop_.output_encoder_bias_);
        api.add_api_variable("obias", &api_obias);
        static APIFloat api_mbias(&actuator_.main_loop_.motor_encoder_bias_);
        api.add_api_variable("mbias", &api_mbias);
        static const APIFloat api_ttgain(&actuator_.main_loop_.calibration_.torque_sensor.table_gain);
        api.add_api_variable("ttgain", &api_ttgain);
        API_ADD_FILTER(id_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.foc_->id_filter_);
        API_ADD_FILTER(iq_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.foc_->iq_filter_);
        API_ADD_FILTER(output_iq_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.iq_filter_);
        API_ADD_FILTER(output_motor_velocity_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.motor_velocity_filter_);
        API_ADD_FILTER(output_motor_position_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.motor_position_filter_);
        static const APIFloat api_startup_phase_lock_current(&param->startup_param.phase_lock_current);
        api.add_api_variable("startup_phase_lock_current", &api_startup_phase_lock_current);
        static APIFloat api_startup_mbias(&actuator_.startup_motor_bias_);
        api.add_api_variable("startup_mbias", &api_startup_mbias);
        static const APICallback api_set_startup_bias([]()->std::string{ actuator_.set_bias(); return "ok"; });
        api.add_api_variable("set_startup_bias", &api_set_startup_bias);
        static APIFloat api_odir(&actuator_.main_loop_.output_encoder_dir_);
        api.add_api_variable("odir", &api_odir);
        static APIFloat api_tdir(&actuator_.main_loop_.torque_sensor_dir_);
        api.add_api_variable("tdir", &api_tdir);
        static APIFloat api_mdir(&actuator_.fast_loop_.motor_encoder_dir_);
        api.add_api_variable("mdir", &api_mdir);
        //API_ADD_FILTER(output_motor_velocity_filter2, FirstOrderLowPassFilter, actuator_.main_loop_.motor_velocity_filter_);
        API_ADD_FILTER(output_motor_position_filter2, FirstOrderLowPassFilter, actuator_.main_loop_.motor_position_filter_);
        //API_ADD_FILTER(output_output_velocity_filter, FirstOrderLowPassFilter, actuator_.main_loop_.output_velocity_filter_);
        API_ADD_FILTER(output_output_position_filter, FirstOrderLowPassFilter, actuator_.main_loop_.output_position_filter_);
        API_ADD_FILTER(output_torque_filter, FirstOrderLowPassFilter, actuator_.main_loop_.torque_filter_);
        static APIFloat api_idir(&actuator_.fast_loop_.current_direction_);
        api.add_api_variable("idir", &api_idir);
        static const APICallbackUint32 api_uptime(get_uptime);
        api.add_api_variable("uptime", &api_uptime);
        static const APICallback api_power_on_time([]{
            char t[9];
            RTClock::power_on_time(t);
            return std::string(t);
        });
        api.add_api_variable("power_on_time", &api_power_on_time);
        static const APIInt32 api_menc(&actuator_.fast_loop_.motor_enc);
        api.add_api_variable("menc", &api_menc);
        static const APICallbackInt32 api_oenc([](){ return actuator_.main_loop_.output_encoder_.get_value(); });
        api.add_api_variable("oenc", &api_oenc);
        static APIFloat api_amax(&actuator_.main_loop_.admittance_controller_.torque_controller_.command_max_);
        api.add_api_variable("amax", &api_amax);
        static APIFloat api_akp(&actuator_.main_loop_.admittance_controller_.torque_controller_.kp_);
        api.add_api_variable("akp", &api_akp);
        static const APIFloat api_Tmotor_est(&actuator_.main_loop_.status_.motor_temperature_estimate);
        api.add_api_variable("Tmotor_est", &api_Tmotor_est);
        API_ADD_FILTER(a_output_filter, FirstOrderLowPassFilter, actuator_.main_loop_.admittance_controller_.torque_controller_.output_filter_);
        static const APICallback api_fast_loop_status([](){ 
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
        });
        api.add_api_variable("fast_loop_status", &api_fast_loop_status);
        static APIFloat api_id_des(&actuator_.fast_loop_.foc_command_.desired.i_d);
        api.add_api_variable("id_des", &api_id_des);
        static const APICallback api_trigger_fast_log([]()->std::string{ actuator_.fast_loop_.trigger_status_log(); return "triggered"; });
        api.add_api_variable("trigger_fast_log", &api_trigger_fast_log);
        static APICallbackFloat api_ilimit([](){ return actuator_.fast_loop_.foc_->get_iq_limit(); },
            [](float f){ actuator_.fast_loop_.foc_->set_iq_limit(f); });
        api.add_api_variable("ilimit", &api_ilimit);
        static APICallbackFloat api_idlimit([](){ return actuator_.fast_loop_.foc_->get_id_limit(); },
            [](float f){ actuator_.fast_loop_.foc_->set_id_limit(f); });
        api.add_api_variable("idlimit", &api_idlimit);
        static APIFloat api_num_poles(&actuator_.fast_loop_.foc_->num_poles_);
        api.add_api_variable("num_poles", &api_num_poles);
        static const APICallbackUint32 api_timestamp(get_clock);
        api.add_api_variable("timestamp", &api_timestamp);
        static const APICallbackFloat api_mrollover([](){ return actuator_.fast_loop_.get_rollover(); });
        api.add_api_variable("mrollover", &api_mrollover);
        static const APIFloat api_gear_ratio(&param->startup_param.gear_ratio);
        api.add_api_variable("gear_ratio", &api_gear_ratio);
        static const APIStringView api_version(OBOT_VERSION);
        api.add_api_variable("version", &api_version);
        static const APIStringView api_obot_hash(OBOT_HASH);
        api.add_api_variable("obot_hash", &api_obot_hash);
        static const APIStringView api_motorlib_hash(MOTORLIB_HASH);
        api.add_api_variable("motorlib_hash", &api_motorlib_hash);
        static const APIStringView api_name(param->name);
        api.add_api_variable("name", &api_name);
        uint32_t api_timeout_us = 10000;
        static APIUint32 api_api_timeout(&api_timeout_us);
        api.add_api_variable("api_timeout", &api_api_timeout);
        static const APIStringView api_notes(NOTES);
        api.add_api_variable("notes", &api_notes);
        static const APIFloat api_tuning_desired(&actuator_.main_loop_.tuning_trajectory_generator_.trajectory_value_.value );
        api.add_api_variable("tuning_desired", &api_tuning_desired);
        static const APIFloat api_dft_frequency(&actuator_.main_loop_.dft_.desired_.frequency_last_);
        api.add_api_variable("dft_frequency", &api_dft_frequency);
        static const APIFloat api_dft_desired_magnitude(&actuator_.main_loop_.dft_.desired_.magnitude_last_);
        api.add_api_variable("dft_desired_magnitude", &api_dft_desired_magnitude);
        static const APIFloat api_dft_phase(&actuator_.main_loop_.dft_.phase_);
        api.add_api_variable("dft_phase", &api_dft_phase);
        static const APIFloat api_dft_magnitude(&actuator_.main_loop_.dft_.magnitude_);
        api.add_api_variable("dft_magnitude", &api_dft_magnitude);
        static APICallbackHex<uint32_t> api_gpioa([](){ return GPIOA->IDR; }, [](uint32_t u){ GPIOA->ODR = u; });
        api.add_api_variable("gpioa", &api_gpioa);
        static APICallbackHex<uint32_t> api_gpiob([](){ return GPIOB->IDR; }, [](uint32_t u){ GPIOB->ODR = u; });
        api.add_api_variable("gpiob", &api_gpiob);
        static APICallbackHex<uint32_t> api_gpioc([](){ return GPIOC->IDR; }, [](uint32_t u){ GPIOC->ODR = u; });
        api.add_api_variable("gpioc", &api_gpioc);
        static APICallbackHex<uint32_t> api_gpiod([](){ return GPIOD->IDR; }, [](uint32_t u){ GPIOD->ODR = u; });
        api.add_api_variable("gpiod", &api_gpiod);
        static APICallbackHex<uint32_t> api_gpioe([](){ return GPIOE->IDR; }, [](uint32_t u){ GPIOE->ODR = u; });
        api.add_api_variable("gpioe", &api_gpioe);
        static const APICallback api_board_name([]()->std::string{ return otp->version == 1 ? otp->name : ""; });
        api.add_api_variable("board_name", &api_board_name);
        static const APICallback api_board_rev([]()->std::string{ return otp->version == 1 ? otp->rev : ""; });
        api.add_api_variable("board_rev", &api_board_rev);
        static const APIInt32 api_board_num(&otp->num);
        api.add_api_variable("board_num", &api_board_num);
        static const APICallback api_long_packet([]{ 
          char long_packet[MAX_API_DATA_SIZE+1] = "This is a long packet test\n";
          int len = std::strlen(long_packet);
          for (int i=0; i<MAX_API_DATA_SIZE-len; i++) {
            long_packet[i+len] = '0' + (i % 10);
          }
          return std::string((char *) &long_packet, sizeof(long_packet));
        });
        api.add_api_variable("long_packet", &api_long_packet);
        static const APICallback api_really_long_packet([]{
          char long_packet[MAX_API_LONG_DATA_SIZE];
          for (int i=0; i<MAX_API_LONG_DATA_SIZE; i++) {
            long_packet[i] = '0' + (i % 10);
          }
          return std::string((char *) &long_packet, sizeof(long_packet));
        });
        api.add_api_variable("really_long_packet", &api_really_long_packet);
        static const APIStringView api_config(CONFIG);
        api.add_api_variable("config", &api_config);
        static const APIStringView api_serial(get_serial_number());
        api.add_api_variable("serial", &api_serial);
        static APIFloat api_olimit_max(&actuator_.main_loop_.encoder_limits_.output_hard_max);
        api.add_api_variable("olimit_max", &api_olimit_max);
        static APIFloat api_olimit_min(&actuator_.main_loop_.encoder_limits_.output_hard_min);
        api.add_api_variable("olimit_min", &api_olimit_min);
        static APIFloat api_mlimit_max(&actuator_.main_loop_.encoder_limits_.motor_hard_max);
        api.add_api_variable("mlimit_max", &api_mlimit_max);
        static APIFloat api_mlimit_min(&actuator_.main_loop_.encoder_limits_.motor_hard_min);
        api.add_api_variable("mlimit_min", &api_mlimit_min);
        static APIFloat api_msoftlimit_max(&actuator_.main_loop_.encoder_limits_.motor_controlled_max);
        api.add_api_variable("msoftlimit_max", &api_msoftlimit_max);
        static APIFloat api_msoftlimit_min(&actuator_.main_loop_.encoder_limits_.motor_controlled_min);
        api.add_api_variable("msoftlimit_min", &api_msoftlimit_min);
        static const APICallbackUint8 api_is_sbank([]()->uint8_t{ return (*((uint8_t *) 0x1fff7802) & 0x40) == 0; });
        api.add_api_variable("is_sbank", &api_is_sbank);
        static APICallbackFloat api_invalid_command_leak_rate_s([]{
            return actuator_.main_loop_.invalid_command_fault_.get_leak_period_s(actuator_.main_loop_.dt_); },
            [](float f){ actuator_.main_loop_.invalid_command_fault_.set_leak_period(f, actuator_.main_loop_.dt_); });
        api.add_api_variable("invalid_command_leak_rate_s", &api_invalid_command_leak_rate_s);
        static APIUint32 api_invalid_command_limit(&actuator_.main_loop_.invalid_command_limit_);
        api.add_api_variable("invalid_command_limit", &api_invalid_command_limit);
        static APIUint32 api_invalid_command_count(&actuator_.main_loop_.invalid_command_fault_.count_);
        api.add_api_variable("invalid_command_count", &api_invalid_command_count);
        static const APICallbackHex<uint32_t> api_fault([](){ return actuator_.main_loop_.status_.error.all; });
        api.add_api_variable("fault", &api_fault);
        static const APICallback api_fault_str([](){
            char c[600];
            actuator_.main_loop_.get_fault_str(c, 600);
            return std::string(c);
        });
        api.add_api_variable("fault_str", &api_fault_str);
        static const APICallbackUint8 api_reset([]()->uint8_t{ NVIC_SystemReset(); return 0; });
        api.add_api_variable("reset", &api_reset);
        uint32_t t_start = get_clock();
        current_api_timeout_us_ = api_timeout_us;
        while(1) {
            TOGGLE_SCOPE_PIN(C,4);
            count_++;
            if (communication_.send_string_active() && get_clock() - t_start > US_TO_CPU(current_api_timeout_us_)) {
                communication_.cancel_send_string();
                current_api_timeout_us_ = api_timeout_us;
            }
            char *s = System::get_string();
            if (s[0] != 0) {
                auto response = api.parse_string(s);
                current_api_timeout_us_ = api_timeout_us;
                communication_.send_string(response.c_str(), response.length());
                t_start = get_clock();
            }
            main_maintenance();
        }
    }
    static void set_one_time_api_timeout_us(uint32_t us) {
        communication_.send_one_time_api_timeout_request(us);
        current_api_timeout_us_ = US_TO_CPU(us);
    }
    static void main_loop_interrupt() {
        actuator_.main_loop_.update();
    }
    static void fast_loop_interrupt() {
        actuator_.fast_loop_.update();
    }
    static void system_loop() {
        system_maintenance();
        actuator_.maintenance();
        round_robin_logger.log_data(UPTIME_INDEX, get_uptime());
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
    static uint32_t current_api_timeout_us_;
};

extern "C" {
#endif // __cplusplus

void system_init();
void system_run();
void main_loop_interrupt();
void fast_loop_interrupt();
void system_loop_interrupt();
void usb_interrupt();

#ifdef __cplusplus
}
#endif

#endif  // UNHUMAN_MOTORLIB_SYSTEM_H_

