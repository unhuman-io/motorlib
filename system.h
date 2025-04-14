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
        //api.add_api_variable("system_count", new APIUint32((uint32_t *) &count_));
        api.add_api_variable<APIUint32>("system_count", &count_);
        api.add_api_variable<const APIUint32>("system_count_const", &count_);
        api.add_api_variable<const APIUint32>("api_memory_used", &ParameterAPI::AllocatorBase::index_);
        //api.add_api_variable<const APIUint32>("mode", &actuator_.main_loop_.mode_);
        api.add_api_variable<APIUint32>("mode", (uint32_t *) &actuator_.main_loop_.mode_);
        api.add_api_variable<APIFloat>("kp", &actuator_.main_loop_.position_controller_.controller_.kp_);
        api.add_api_variable<APIFloat>("kd", &actuator_.main_loop_.position_controller_.controller_.kd_);
        api.add_api_variable<APIFloat>("ki", &actuator_.main_loop_.position_controller_.controller_.ki_);
        api.add_api_variable<APIFloat>("ki_limit", &actuator_.main_loop_.position_controller_.controller_.ki_limit_);
        api.add_api_variable<APIFloat>("max", &actuator_.main_loop_.position_controller_.controller_.command_max_);
        api.add_api_variable<APIFloat>("tracking_tol", &actuator_.main_loop_.position_controller_.tracking_tolerance_);
        api.add_api_variable<APIFloat>("vlimit", &actuator_.main_loop_.position_controller_.velocity_limit_);
        API_ADD_FILTER(desired_filter, SecondOrderLowPassFilter, actuator_.main_loop_.position_controller_.desired_filter_);
        api.add_api_variable<const APIFloat>("error", &actuator_.main_loop_.position_controller_.controller_.error_);
        API_ADD_FILTER(velocity_filter, SecondOrderLowPassFilter, actuator_.main_loop_.position_controller_.controller_.velocity_filter_);
        API_ADD_FILTER(output_filter, FirstOrderLowPassFilter, actuator_.main_loop_.position_controller_.controller_.output_filter_);
        api.add_api_variable<APIFloat>("vkp", &actuator_.main_loop_.velocity_controller_.controller_.kp_);
        api.add_api_variable<APIFloat>("vki", &actuator_.main_loop_.velocity_controller_.controller_.ki_);
        api.add_api_variable<APIFloat>("vki_limit", &actuator_.main_loop_.velocity_controller_.controller_.ki_limit_);
        api.add_api_variable<APIFloat>("vmax", &actuator_.main_loop_.velocity_controller_.controller_.command_max_);
        api.add_api_variable<APIFloat>("vacceleration_limit", &actuator_.main_loop_.velocity_controller_.acceleration_limit_);
        api.add_api_variable<APIFloat>("jmax", &actuator_.main_loop_.joint_position_controller_.velocity_controller_.controller_.command_max_);
        api.add_api_variable<APIFloat>("jki_limit", &actuator_.main_loop_.joint_position_controller_.velocity_controller_.controller_.ki_limit_);
        API_ADD_FILTER(vfilt, FirstOrderLowPassFilter, actuator_.main_loop_.velocity_controller_.velocity_filter_);
        API_ADD_FILTER(voutput_filt, FirstOrderLowPassFilter, actuator_.main_loop_.velocity_controller_.controller_.output_filter_);
        api.add_api_variable<APIUint32>("cpu_frequency", &cpu_frequency);
        api.add_api_variable<APIUint32>("t_exec_fastloop", &t_exec_fastloop);
        api.add_api_variable<APIUint32>("t_exec_mainloop", &t_exec_mainloop);
        api.add_api_variable<APIUint32>("t_period_fastloop", &t_period_fastloop);
        api.add_api_variable<APIUint32>("t_period_mainloop", &t_period_mainloop);
        api.add_api_variable<APIFloat>("vbus", &actuator_.main_loop_.status_.fast_loop.vbus);
        api.add_api_variable<APICallbackUint8>("phase_mode", [](){ return actuator_.fast_loop_.get_phase_mode(); }, [](uint8_t p){ actuator_.fast_loop_.set_phase_mode(p); });
        api.add_api_variable<APIFloat>("va", &actuator_.main_loop_.status_.fast_loop.foc_status.command.v_a);
        api.add_api_variable<APIFloat>("vb", &actuator_.main_loop_.status_.fast_loop.foc_status.command.v_b);
        api.add_api_variable<APIFloat>("vc", &actuator_.main_loop_.status_.fast_loop.foc_status.command.v_c);
        api.add_api_variable<APIFloat>("vq", &actuator_.main_loop_.status_.fast_loop.foc_status.command.v_q);
        api.add_api_variable<APIFloat>("vd", &actuator_.main_loop_.status_.fast_loop.foc_status.command.v_d);
        api.add_api_variable<APIFloat>("ia", &actuator_.fast_loop_.foc_command_.measured.i_a);
        api.add_api_variable<APIFloat>("ib", &actuator_.fast_loop_.foc_command_.measured.i_b);
        api.add_api_variable<APIFloat>("ic", &actuator_.fast_loop_.foc_command_.measured.i_c);
        api.add_api_variable<APIFloat>("id", &actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_d);
        api.add_api_variable<APIFloat>("iq", &actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_q);
        api.add_api_variable<APIFloat>("i0", &actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_0);
        api.add_api_variable<APIFloat>("ikp", &actuator_.fast_loop_.foc_->pi_iq_.kp_);
        api.add_api_variable<APICallbackFloat>("iki", [](){ return actuator_.fast_loop_.foc_->pi_iq_.ki_; },
            [](float f){ if (f == 0) { actuator_.fast_loop_.foc_->pi_iq_.ki_sum_ = 0; } actuator_.fast_loop_.foc_->pi_iq_.ki_ = f; });
        api.add_api_variable<APIFloat>("iki_limit", &actuator_.fast_loop_.foc_->pi_iq_.ki_limit_);
        api.add_api_variable<APIFloat>("imax", &actuator_.fast_loop_.foc_->pi_iq_.command_max_);
        api.add_api_variable<APIFloat>("idkp", &actuator_.fast_loop_.foc_->pi_id_.kp_);
        api.add_api_variable<APICallbackFloat>("idki", [](){ return actuator_.fast_loop_.foc_->pi_id_.ki_; },
            [](float f){ if (f == 0) { actuator_.fast_loop_.foc_->pi_id_.ki_sum_ = 0; } actuator_.fast_loop_.foc_->pi_id_.ki_ = f; });
        api.add_api_variable<APIFloat>("idki_limit", &actuator_.fast_loop_.foc_->pi_id_.ki_limit_);
        api.add_api_variable<APIFloat>("idmax", &actuator_.fast_loop_.foc_->pi_id_.command_max_);
        api.add_api_variable<APIFloat>("icmax", &actuator_.fast_loop_.foc_->param_.voltage_limit);
        api.add_api_variable<const APICallback>("idiq", []{
                actuator_.fast_loop_.foc_->pi_id_.kp_ = actuator_.fast_loop_.foc_->pi_iq_.kp_;
                actuator_.fast_loop_.foc_->pi_id_.ki_ = actuator_.fast_loop_.foc_->pi_iq_.ki_;
                actuator_.fast_loop_.foc_->pi_id_.ki_limit_ = actuator_.fast_loop_.foc_->pi_iq_.ki_limit_;
                actuator_.fast_loop_.foc_->pi_id_.command_max_ = actuator_.fast_loop_.foc_->pi_iq_.command_max_;
                actuator_.fast_loop_.foc_->set_id_limit(actuator_.fast_loop_.foc_->get_iq_limit());
                return std::string("ok"); });
        TORQUE_CONTROLLER_DEBUG_VARIABLES(api, actuator_.main_loop_.torque_controller_);
        STATE_CONTROLLER_DEBUG_VARIABLES(api, actuator_.main_loop_.state_controller_);
        api.add_api_variable<APIFloat>("tgain", &actuator_.main_loop_.torque_sensor_.gain_);
        api.add_api_variable<APIFloat>("tbias", &actuator_.main_loop_.torque_sensor_bias_);
        api.add_api_variable<const APIFloat>("torque", &actuator_.main_loop_.status_.torque);
        api.add_api_variable<const APIFloat>("t_i_correction", &actuator_.main_loop_.param_.torque_correction);
        api.add_api_variable<APICallback>("log", get_log, log);
        api.add_api_variable<const APICallback>("old_log", []{ return logger.get_old_log(); });
        api.add_api_variable<const APICallback>("log_reset", []()->std::string{ logger.reset_read_front(); return "ok"; });
        api.add_api_variable<const APICallbackUint32>("log_num", []{ return logger.num_elements(); });
        api.add_api_variable<const APIStringView>("messages_version", MOTOR_MESSAGES_VERSION);
        api.add_api_variable<const APICallbackInt32>("index_pos", []{ return actuator_.fast_loop_.encoder_.get_index_pos(); });
        api.add_api_variable<const APICallbackUint8>("index_received", []()->uint8_t{return actuator_.fast_loop_.encoder_.index_received();});
        api.add_api_variable<const APIFloat>("index_offset_measured", &actuator_.fast_loop_.motor_index_electrical_offset_measured_);
        api.add_api_variable<APIInt32>("electrical_zero_pos", &actuator_.fast_loop_.motor_electrical_zero_pos_);
        api.add_api_variable<const APIUint32>("mcpr", &param->fast_loop_param.motor_encoder.cpr);
        api.add_api_variable<const APIFloat>("ocpr", &param->main_loop_param.output_encoder.cpr);
        api.add_api_variable<const APICallbackFloat>("irange", [](){ return 2048*param->fast_loop_param.adc1_gain; });
        api.add_api_variable<const APICallbackUint32>("stack_free", get_stack_free);
        api.add_api_variable<const APICallbackUint32>("stack_used", get_stack_used);
        api.add_api_variable<const APICallbackUint32>("heap_free", get_heap_free);
        api.add_api_variable<const APICallbackUint32>("heap_used", get_heap_used);
        api.add_api_variable<const APICallbackUint32>("heap_current_free", get_current_heap_free);
        api.add_api_variable<const APICallbackUint32>("heap_current_used", get_current_heap_used);
        api.add_api_variable<APICallbackUint32>("malloc", [](){ return (uint32_t) get_heap_free() + get_heap_used(); }, 
            [](uint32_t u) {
                try { char* volatile c = new char[u]; delete c; }
                catch(...) { logger.log_printf("couldn't allocate %d", u); } });
        api.add_api_variable<APIFloat>("vbus_min", &actuator_.main_loop_.vbus_min_);
        api.add_api_variable<APIFloat>("vbus_max", &actuator_.main_loop_.vbus_max_);
        api.add_api_variable<APIFloat>("ia_bias", &actuator_.fast_loop_.ia_bias_);
        api.add_api_variable<APIFloat>("ib_bias", &actuator_.fast_loop_.ib_bias_);
        api.add_api_variable<APIFloat>("ic_bias", &actuator_.fast_loop_.ic_bias_);
        api.add_api_variable<const APIFloat>("power", &actuator_.main_loop_.status_.fast_loop.power);
        api.add_api_variable<const APIFloat>("power_avg", &actuator_.main_loop_.status_.power);
        api.add_api_variable<const APIUint32>("energy", &actuator_.main_loop_.status_.fast_loop.energy_uJ);
        api.add_api_variable<const APICallback>("fast_log", [](){
            actuator_.main_loop_.lock_status_log();
            FastLog log;
            std::string out;
            out.reserve(FAST_LOG_LENGTH * sizeof(log));
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
            return out; });
        api.add_api_variable<const APICallback>("fast_log2", [](){
            actuator_.main_loop_.lock_status_log();
            FastLog2 log;
            std::string out;
            out.reserve(FAST_LOG_LENGTH * sizeof(log));
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
            return out; });
        api.add_api_variable<const APICallbackFloat>("beep", [](){ return 0.0; }, [](float f){ actuator_.fast_loop_.beep_on(f); });
        api.add_api_variable<APIFloat>("beep_frequency", &actuator_.fast_loop_.param_.beep_frequency);
        api.add_api_variable<APIFloat>("beep_amplitude", &actuator_.fast_loop_.param_.beep_amplitude);
        api.add_api_variable<APICallbackFloat>("zero_current_sensors", [](){ return 0.0; }, [](float f){ actuator_.fast_loop_.zero_current_sensors_on(f); });
        api.add_api_variable<const APICallback>("disable_safe_mode", []()->std::string{ actuator_.main_loop_.error_mask_.all = ERROR_MASK_NONE; return "ok"; });
        api.add_api_variable<APICallback>("error_mask", [](){ return u32_to_hex(actuator_.main_loop_.error_mask_.all); },
                [](std::string s){ try {
                        actuator_.main_loop_.error_mask_.all = std::stoul(s, nullptr, 16) & ERROR_MASK_ALL;}
                    catch(...) {} });
        api.add_api_variable<const APICallback>("help", [](){ return api.get_all_api_variables(); });
        api.add_api_variable<const APICallbackUint16>("api_length", [](){ return api.get_api_length(); });
        api.add_api_variable<APIBool>("disable_position_limits", &actuator_.main_loop_.position_limits_disable_);
        api.add_api_variable<APIFloat>("jkpj", &actuator_.main_loop_.joint_position_controller_.param_.kpj);
        api.add_api_variable<const APIFloat>("motor_position_raw", &actuator_.fast_loop_.motor_position_);
        api.add_api_variable<APIFloat>("obias", &actuator_.main_loop_.output_encoder_bias_);
        api.add_api_variable<APIFloat>("mbias", &actuator_.main_loop_.motor_encoder_bias_);
        api.add_api_variable<const APIFloat>("ttgain", &actuator_.main_loop_.calibration_.torque_sensor.table_gain);
        API_ADD_FILTER(id_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.foc_->id_filter_);
        API_ADD_FILTER(iq_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.foc_->iq_filter_);
        API_ADD_FILTER(output_iq_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.iq_filter_);
        API_ADD_FILTER(output_motor_velocity_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.motor_velocity_filter_);
        API_ADD_FILTER(output_motor_position_filter, FirstOrderLowPassFilter, actuator_.fast_loop_.motor_position_filter_);
        api.add_api_variable<const APIFloat>("startup_phase_lock_current", &param->startup_param.phase_lock_current);
        api.add_api_variable<APIFloat>("startup_mbias", &actuator_.startup_motor_bias_);
        api.add_api_variable<const APICallback>("set_startup_bias", []()->std::string{ actuator_.set_bias(); return "ok"; });
        api.add_api_variable<APIFloat>("odir", &actuator_.main_loop_.output_encoder_dir_);
        api.add_api_variable<APIFloat>("tdir", &actuator_.main_loop_.torque_sensor_dir_);
        api.add_api_variable<APIFloat>("mdir", &actuator_.fast_loop_.motor_encoder_dir_);
        //API_ADD_FILTER(output_motor_velocity_filter2, FirstOrderLowPassFilter, actuator_.main_loop_.motor_velocity_filter_);
        API_ADD_FILTER(output_motor_position_filter2, FirstOrderLowPassFilter, actuator_.main_loop_.motor_position_filter_);
        //API_ADD_FILTER(output_output_velocity_filter, FirstOrderLowPassFilter, actuator_.main_loop_.output_velocity_filter_);
        API_ADD_FILTER(output_output_position_filter, FirstOrderLowPassFilter, actuator_.main_loop_.output_position_filter_);
        API_ADD_FILTER(output_torque_filter, FirstOrderLowPassFilter, actuator_.main_loop_.torque_filter_);
        api.add_api_variable<APIFloat>("idir", &actuator_.fast_loop_.current_direction_);
        api.add_api_variable<const APICallbackUint32>("uptime", get_uptime);
        api.add_api_variable<const APICallback>("power_on_time", []{
            char t[9];
            RTClock::power_on_time(t);
            return std::string(t);
        });
        api.add_api_variable<const APIInt32>("menc", &actuator_.fast_loop_.motor_enc);
        api.add_api_variable<const APICallbackInt32>("oenc", [](){ return actuator_.main_loop_.output_encoder_.get_value(); });
        api.add_api_variable<APIFloat>("amax", &actuator_.main_loop_.admittance_controller_.torque_controller_.command_max_);
        api.add_api_variable<APIFloat>("akp", &actuator_.main_loop_.admittance_controller_.torque_controller_.kp_);
        api.add_api_variable<const APIFloat>("Tmotor_est", &actuator_.main_loop_.status_.motor_temperature_estimate);
        API_ADD_FILTER(a_output_filter, FirstOrderLowPassFilter, actuator_.main_loop_.admittance_controller_.torque_controller_.output_filter_);
        api.add_api_variable<const APICallback>("fast_loop_status", [](){ 
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
        api.add_api_variable<APIFloat>("id_des", &actuator_.fast_loop_.foc_command_.desired.i_d);
        api.add_api_variable<const APICallback>("trigger_fast_log", []()->std::string{ actuator_.fast_loop_.trigger_status_log(); return "triggered"; });
        api.add_api_variable<APICallbackFloat>("ilimit", [](){ return actuator_.fast_loop_.foc_->get_iq_limit(); },
            [](float f){ actuator_.fast_loop_.foc_->set_iq_limit(f); });
        api.add_api_variable<APICallbackFloat>("idlimit", [](){ return actuator_.fast_loop_.foc_->get_id_limit(); },
            [](float f){ actuator_.fast_loop_.foc_->set_id_limit(f); });
        api.add_api_variable<APIFloat>("num_poles", &actuator_.fast_loop_.foc_->num_poles_);
        api.add_api_variable<const APICallbackUint32>("timestamp", get_clock);
        api.add_api_variable<const APICallbackFloat>("mrollover", [](){ return actuator_.fast_loop_.get_rollover(); });
        api.add_api_variable<const APIFloat>("gear_ratio", &param->startup_param.gear_ratio);
        api.add_api_variable<const APIStringView>("version", OBOT_VERSION);
        api.add_api_variable<const APIStringView>("obot_hash", OBOT_HASH);
        api.add_api_variable<const APIStringView>("motorlib_hash", MOTORLIB_HASH);
        api.add_api_variable<const APIStringView>("name", param->name);
        uint32_t api_timeout_us = 10000;
        api.add_api_variable<APIUint32>("api_timeout", &api_timeout_us);
        api.add_api_variable<const APIStringView>("notes", NOTES);
        api.add_api_variable<const APIFloat>("tuning_desired", &actuator_.main_loop_.tuning_trajectory_generator_.trajectory_value_.value );
        api.add_api_variable<const APIFloat>("dft_frequency", &actuator_.main_loop_.dft_.desired_.frequency_last_);
        api.add_api_variable<const APIFloat>("dft_desired_magnitude", &actuator_.main_loop_.dft_.desired_.magnitude_last_);
        api.add_api_variable<const APIFloat>("dft_phase", &actuator_.main_loop_.dft_.phase_);
        api.add_api_variable<const APIFloat>("dft_magnitude", &actuator_.main_loop_.dft_.magnitude_);
        api.add_api_variable<APICallbackHex<uint32_t>>("gpioa", [](){ return GPIOA->IDR; }, [](uint32_t u){ GPIOA->ODR = u; });
        api.add_api_variable<APICallbackHex<uint32_t>>("gpiob", [](){ return GPIOB->IDR; }, [](uint32_t u){ GPIOB->ODR = u; });
        api.add_api_variable<APICallbackHex<uint32_t>>("gpioc", [](){ return GPIOC->IDR; }, [](uint32_t u){ GPIOC->ODR = u; });
        api.add_api_variable<APICallbackHex<uint32_t>>("gpiod", [](){ return GPIOD->IDR; }, [](uint32_t u){ GPIOD->ODR = u; });
        api.add_api_variable<APICallbackHex<uint32_t>>("gpioe", [](){ return GPIOE->IDR; }, [](uint32_t u){ GPIOE->ODR = u; });
        api.add_api_variable<const APICallback>("board_name", []()->std::string{ return otp->version == 1 ? otp->name : ""; });
        api.add_api_variable<const APICallback>("board_rev", []()->std::string{ return otp->version == 1 ? otp->rev : ""; });
        api.add_api_variable<const APIInt32>("board_num", &otp->num);
        api.add_api_variable<const APICallback>("long_packet", []{ 
          char long_packet[MAX_API_DATA_SIZE+1] = "This is a long packet test\n";
          int len = std::strlen(long_packet);
          for (int i=0; i<MAX_API_DATA_SIZE-len; i++) {
            long_packet[i+len] = '0' + (i % 10);
          }
          return std::string((char *) &long_packet, sizeof(long_packet));
        });
        api.add_api_variable<const APICallback>("really_long_packet", []{
          char long_packet[MAX_API_LONG_DATA_SIZE];
          for (int i=0; i<MAX_API_LONG_DATA_SIZE; i++) {
            long_packet[i] = '0' + (i % 10);
          }
          return std::string((char *) &long_packet, sizeof(long_packet));
        });
        api.add_api_variable<const APIStringView>("config", CONFIG);
        api.add_api_variable<const APIStringView>("serial", get_serial_number());
        api.add_api_variable<APIFloat>("olimit_max", &actuator_.main_loop_.encoder_limits_.output_hard_max);
        api.add_api_variable<APIFloat>("olimit_min", &actuator_.main_loop_.encoder_limits_.output_hard_min);
        api.add_api_variable<APIFloat>("mlimit_max", &actuator_.main_loop_.encoder_limits_.motor_hard_max);
        api.add_api_variable<APIFloat>("mlimit_min", &actuator_.main_loop_.encoder_limits_.motor_hard_min);
        api.add_api_variable<APIFloat>("msoftlimit_max", &actuator_.main_loop_.encoder_limits_.motor_controlled_max);
        api.add_api_variable<APIFloat>("msoftlimit_min", &actuator_.main_loop_.encoder_limits_.motor_controlled_min);
        api.add_api_variable<const APICallbackUint8>("is_sbank", []()->uint8_t{ return (*((uint8_t *) 0x1fff7802) & 0x40) == 0; });
        api.add_api_variable<APICallbackFloat>("invalid_command_leak_rate_s", []{
            return actuator_.main_loop_.invalid_command_fault_.get_leak_period_s(actuator_.main_loop_.dt_); },
            [](float f){ actuator_.main_loop_.invalid_command_fault_.set_leak_period(f, actuator_.main_loop_.dt_); });
        api.add_api_variable<APIUint32>("invalid_command_limit", &actuator_.main_loop_.invalid_command_limit_);
        api.add_api_variable<APIUint32>("invalid_command_count", &actuator_.main_loop_.invalid_command_fault_.count_);
        api.add_api_variable<const APICallbackHex<uint32_t>>("fault", [](){ return actuator_.main_loop_.status_.error.all; });
        api.add_api_variable<const APICallback>("fault_str", [](){
            char c[600];
            actuator_.main_loop_.get_fault_str(c, 600);
            return std::string(c);
        });
        api.add_api_variable<const APICallbackUint8>("reset", []()->uint8_t{ NVIC_SystemReset(); return 0; });
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

