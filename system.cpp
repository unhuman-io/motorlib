#include <string>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>

void system_run() {
    System::run();
}

void main_loop_interrupt() {
    System::main_loop_interrupt();
}

void fast_loop_interrupt() {
    System::fast_loop_interrupt();
}

void system_log(std::string s) {
    System::log(s);
}

void system_loop_interrupt() {
    System::system_loop();
}

Logger::CIndex log_index __attribute__((section(".noload")));
char log_queue[LOGGING_MAX_SIZE] __attribute__((section(".noload")));
__attribute__ ((init_priority(LOGGER_INIT_PRIORITY))) Logger logger(log_index, log_queue);
RoundRobinLogger round_robin_logger;
uint32_t System::count_ = 0;
ParameterAPI System::api = {};
uint32_t System::current_api_timeout_us_ = 0;

// send printf and other stdout/err to the logger
extern "C" void _write(int fd, const char *buf, size_t count) {
    logger.log(std::string_view(buf, count));
}


// Necessary for _write maybe?
extern "C" int _fstat (int fd, struct stat * st) 
{
  memset (st, 0, sizeof (* st));
  st->st_mode = S_IFCHR;
  setbuf(stdout, NULL); // Disable buffering for stdout
  setbuf(stderr, NULL);
  return (0);
}

extern "C" int _isatty (int fd) 
{
  return (1);
}

extern "C" caddr_t _sbrk (int incr) 
{
  static char * heap;
         char * prev_heap;

  if (heap == NULL) {
    heap = (char *)&_end;
  }
  
  prev_heap = heap;

  if ((heap + incr) > (char *)(&_estack - (uint32_t) &_Min_Stack_Size)) {
    errno = ENOMEM;
    logger.log("Heap overflow");
    return (caddr_t) -1;
  }
  
  heap += incr;

  return (caddr_t) prev_heap;
}

extern "C" void _exit(int status) {
    logger.log("Exiting with status: " + std::to_string(status));
    while (1);
}



// api.add_api_variable("api_memory_used", new const APIUint32(&ParameterAPI::AllocatorBase::index_));

// API_ADD_VARIABLE(api_memory_used, APIInt<uint32_t, ROVariable>, &ParameterAPI::AllocatorBase::index_);
// API_ADD_VARIABLE(kp, APIFloat, &System::actuator_.main_loop_.position_controller_.controller_.kp_);
// API_ADD_VARIABLE(kd, APIFloat, &System::actuator_.main_loop_.position_controller_.controller_.kd_);
// API_ADD_VARIABLE(ki, APIFloat, &System::actuator_.main_loop_.position_controller_.controller_.ki_);
// API_ADD_VARIABLE(ki_limit, APIFloat, &System::actuator_.main_loop_.position_controller_.controller_.ki_limit_);
// API_ADD_VARIABLE(max, APIFloat, &System::actuator_.main_loop_.position_controller_.controller_.command_max_);
// API_ADD_VARIABLE(tracking_tol, APIFloat, &System::actuator_.main_loop_.position_controller_.tracking_tolerance_);
// API_ADD_VARIABLE(vlimit, APIFloat, &System::actuator_.main_loop_.position_controller_.velocity_limit_);


const APIFloat kp1(&System::actuator_.main_loop_.position_controller_.controller_.kp_);
int32_t i;
constinit const APIInt32 iii(&i);
__attribute__((used, section("api_list"))) constinit auto api_kp1 = create_api_variable("kp1", kp1);
APIFloat ff(&System::actuator_.main_loop_.status_.fast_loop.foc_status.command.v_d);
APIInt32 ii(&System::actuator_.fast_loop_.motor_electrical_zero_pos_);



       uint32_t cpu_frequency = CPU_FREQUENCY_HZ;
// const APIUint32 var_system_count((uint32_t *) &count_);
// __attribute__((used, section("api_list"))) constinit auto api_system_count = create_api_variable("system_count", var_system_count);
// const APIUint32 var_mode((uint32_t *) &System::actuator_.main_loop_.mode_);
// __attribute__((used, section("api_list"))) constinit auto api_mode = create_api_variable("mode", var_mode);
const APIUint32 var_api_memory_used(&ParameterAPI::AllocatorBase::index_);
__attribute__((used, section("api_list"))) constinit auto api_api_memory_used = create_api_variable("api_memory_used", var_api_memory_used);
const APIFloat var_kp(&System::actuator_.main_loop_.position_controller_.controller_.kp_);
__attribute__((used, section("api_list"))) constinit auto api_kp = create_api_variable("kp", var_kp);
const APIFloat var_kd(&System::actuator_.main_loop_.position_controller_.controller_.kd_);
__attribute__((used, section("api_list"))) constinit auto api_kd = create_api_variable("kd", var_kd);
const APIFloat var_ki(&System::actuator_.main_loop_.position_controller_.controller_.ki_);
__attribute__((used, section("api_list"))) constinit auto api_ki = create_api_variable("ki", var_ki);
const APIFloat var_ki_limit(&System::actuator_.main_loop_.position_controller_.controller_.ki_limit_);
__attribute__((used, section("api_list"))) constinit auto api_ki_limit = create_api_variable("ki_limit", var_ki_limit);
const APIFloat var_max(&System::actuator_.main_loop_.position_controller_.controller_.command_max_);
__attribute__((used, section("api_list"))) constinit auto api_max = create_api_variable("max", var_max);
const APIFloat var_tracking_tol(&System::actuator_.main_loop_.position_controller_.tracking_tolerance_);
__attribute__((used, section("api_list"))) constinit auto api_tracking_tol = create_api_variable("tracking_tol", var_tracking_tol);
const APIFloat var_vlimit(&System::actuator_.main_loop_.position_controller_.velocity_limit_);
__attribute__((used, section("api_list"))) constinit auto api_vlimit = create_api_variable("vlimit", var_vlimit);
        API_ADD_FILTER(desired_filter, SecondOrderLowPassFilter, System::actuator_.main_loop_.position_controller_.desired_filter_);
const APIFloat var_error(&System::actuator_.main_loop_.position_controller_.controller_.error_);
__attribute__((used, section("api_list"))) constinit auto api_error = create_api_variable("error", var_error);
        API_ADD_FILTER(velocity_filter, SecondOrderLowPassFilter, System::actuator_.main_loop_.position_controller_.controller_.velocity_filter_);
        API_ADD_FILTER(output_filter, FirstOrderLowPassFilter, System::actuator_.main_loop_.position_controller_.controller_.output_filter_);
const APIFloat var_vkp(&System::actuator_.main_loop_.velocity_controller_.controller_.kp_);
__attribute__((used, section("api_list"))) constinit auto api_vkp = create_api_variable("vkp", var_vkp);
const APIFloat var_vki(&System::actuator_.main_loop_.velocity_controller_.controller_.ki_);
__attribute__((used, section("api_list"))) constinit auto api_vki = create_api_variable("vki", var_vki);
const APIFloat var_vki_limit(&System::actuator_.main_loop_.velocity_controller_.controller_.ki_limit_);
__attribute__((used, section("api_list"))) constinit auto api_vki_limit = create_api_variable("vki_limit", var_vki_limit);
const APIFloat var_vmax(&System::actuator_.main_loop_.velocity_controller_.controller_.command_max_);
__attribute__((used, section("api_list"))) constinit auto api_vmax = create_api_variable("vmax", var_vmax);
const APIFloat var_vacceleration_limit(&System::actuator_.main_loop_.velocity_controller_.acceleration_limit_);
__attribute__((used, section("api_list"))) constinit auto api_vacceleration_limit = create_api_variable("vacceleration_limit", var_vacceleration_limit);
const APIFloat var_jmax(&System::actuator_.main_loop_.joint_position_controller_.velocity_controller_.controller_.command_max_);
__attribute__((used, section("api_list"))) constinit auto api_jmax = create_api_variable("jmax", var_jmax);
const APIFloat var_jki_limit(&System::actuator_.main_loop_.joint_position_controller_.velocity_controller_.controller_.ki_limit_);
__attribute__((used, section("api_list"))) constinit auto api_jki_limit = create_api_variable("jki_limit", var_jki_limit);
        API_ADD_FILTER(vfilt, FirstOrderLowPassFilter, System::actuator_.main_loop_.velocity_controller_.velocity_filter_);
        API_ADD_FILTER(voutput_filt, FirstOrderLowPassFilter, System::actuator_.main_loop_.velocity_controller_.controller_.output_filter_);
const APIUint32 var_cpu_frequency(&cpu_frequency);
__attribute__((used, section("api_list"))) constinit auto api_cpu_frequency = create_api_variable("cpu_frequency", var_cpu_frequency);
const APIUint32 var_t_exec_fastloop(&t_exec_fastloop);
__attribute__((used, section("api_list"))) constinit auto api_t_exec_fastloop = create_api_variable("t_exec_fastloop", var_t_exec_fastloop);
const APIUint32 var_t_exec_mainloop(&t_exec_mainloop);
__attribute__((used, section("api_list"))) constinit auto api_t_exec_mainloop = create_api_variable("t_exec_mainloop", var_t_exec_mainloop);
const APIUint32 var_t_period_fastloop(&t_period_fastloop);
__attribute__((used, section("api_list"))) constinit auto api_t_period_fastloop = create_api_variable("t_period_fastloop", var_t_period_fastloop);
const APIUint32 var_t_period_mainloop(&t_period_mainloop);
__attribute__((used, section("api_list"))) constinit auto api_t_period_mainloop = create_api_variable("t_period_mainloop", var_t_period_mainloop);
const APIFloat var_vbus(&System::actuator_.main_loop_.status_.fast_loop.vbus);
__attribute__((used, section("api_list"))) constinit auto api_vbus = create_api_variable("vbus", var_vbus);
const APICallbackUint8 var_phase_mode([](){ return System::actuator_.fast_loop_.get_phase_mode(); }, [](uint8_t p){ System::actuator_.fast_loop_.set_phase_mode(p); });
__attribute__((used, section("api_list"))) constinit auto api_phase_mode = create_api_variable("phase_mode", var_phase_mode);
const APIFloat var_va(&System::actuator_.main_loop_.status_.fast_loop.foc_status.command.v_a);
__attribute__((used, section("api_list"))) constinit auto api_va = create_api_variable("va", var_va);
const APIFloat var_vb(&System::actuator_.main_loop_.status_.fast_loop.foc_status.command.v_b);
__attribute__((used, section("api_list"))) constinit auto api_vb = create_api_variable("vb", var_vb);
const APIFloat var_vc(&System::actuator_.main_loop_.status_.fast_loop.foc_status.command.v_c);
__attribute__((used, section("api_list"))) constinit auto api_vc = create_api_variable("vc", var_vc);
const APIFloat var_vq(&System::actuator_.main_loop_.status_.fast_loop.foc_status.command.v_q);
__attribute__((used, section("api_list"))) constinit auto api_vq = create_api_variable("vq", var_vq);
const APIFloat var_vd(&System::actuator_.main_loop_.status_.fast_loop.foc_status.command.v_d);
__attribute__((used, section("api_list"))) constinit auto api_vd = create_api_variable("vd", var_vd);
const APIFloat var_ia(&System::actuator_.fast_loop_.foc_command_.measured.i_a);
__attribute__((used, section("api_list"))) constinit auto api_ia = create_api_variable("ia", var_ia);
const APIFloat var_ib(&System::actuator_.fast_loop_.foc_command_.measured.i_b);
__attribute__((used, section("api_list"))) constinit auto api_ib = create_api_variable("ib", var_ib);
const APIFloat var_ic(&System::actuator_.fast_loop_.foc_command_.measured.i_c);
__attribute__((used, section("api_list"))) constinit auto api_ic = create_api_variable("ic", var_ic);
const APIFloat var_id(&System::actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_d);
__attribute__((used, section("api_list"))) constinit auto api_id = create_api_variable("id", var_id);
const APIFloat var_iq(&System::actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_q);
__attribute__((used, section("api_list"))) constinit auto api_iq = create_api_variable("iq", var_iq);
const APIFloat var_i0(&System::actuator_.main_loop_.status_.fast_loop.foc_status.measured.i_0);
__attribute__((used, section("api_list"))) constinit auto api_i0 = create_api_variable("i0", var_i0);
const APIFloat var_ikp(&System::actuator_.fast_loop_.foc_->pi_iq_.kp_);
__attribute__((used, section("api_list"))) constinit auto api_ikp = create_api_variable("ikp", var_ikp);
const APICallbackFloat var_iki([](){ return System::actuator_.fast_loop_.foc_->pi_iq_.ki_; },
            [](float f){ if (f == 0) { System::actuator_.fast_loop_.foc_->pi_iq_.ki_sum_ = 0; } System::actuator_.fast_loop_.foc_->pi_iq_.ki_ = f; });
__attribute__((used, section("api_list"))) constinit auto api_iki = create_api_variable("iki", var_iki);
const APIFloat var_iki_limit(&System::actuator_.fast_loop_.foc_->pi_iq_.ki_limit_);
__attribute__((used, section("api_list"))) constinit auto api_iki_limit = create_api_variable("iki_limit", var_iki_limit);
const APIFloat var_imax(&System::actuator_.fast_loop_.foc_->pi_iq_.command_max_);
__attribute__((used, section("api_list"))) constinit auto api_imax = create_api_variable("imax", var_imax);
const APIFloat var_idkp(&System::actuator_.fast_loop_.foc_->pi_id_.kp_);
__attribute__((used, section("api_list"))) constinit auto api_idkp = create_api_variable("idkp", var_idkp);
const APICallbackFloat var_idki([](){ return System::actuator_.fast_loop_.foc_->pi_id_.ki_; },
            [](float f){ if (f == 0) { System::actuator_.fast_loop_.foc_->pi_id_.ki_sum_ = 0; } System::actuator_.fast_loop_.foc_->pi_id_.ki_ = f; });
__attribute__((used, section("api_list"))) constinit auto api_idki = create_api_variable("idki", var_idki);
const APIFloat var_idki_limit(&System::actuator_.fast_loop_.foc_->pi_id_.ki_limit_);
__attribute__((used, section("api_list"))) constinit auto api_idki_limit = create_api_variable("idki_limit", var_idki_limit);
const APIFloat var_idmax(&System::actuator_.fast_loop_.foc_->pi_id_.command_max_);
__attribute__((used, section("api_list"))) constinit auto api_idmax = create_api_variable("idmax", var_idmax);
const APIFloat var_icmax(&System::actuator_.fast_loop_.foc_->param_.voltage_limit);
__attribute__((used, section("api_list"))) constinit auto api_icmax = create_api_variable("icmax", var_icmax);
const APICallback var_idiq([]{
                System::actuator_.fast_loop_.foc_->pi_id_.kp_ = System::actuator_.fast_loop_.foc_->pi_iq_.kp_;
                System::actuator_.fast_loop_.foc_->pi_id_.ki_ = System::actuator_.fast_loop_.foc_->pi_iq_.ki_;
                System::actuator_.fast_loop_.foc_->pi_id_.ki_limit_ = System::actuator_.fast_loop_.foc_->pi_iq_.ki_limit_;
                System::actuator_.fast_loop_.foc_->pi_id_.command_max_ = System::actuator_.fast_loop_.foc_->pi_iq_.command_max_;
                System::actuator_.fast_loop_.foc_->set_id_limit(System::actuator_.fast_loop_.foc_->get_iq_limit());
                return std::string("ok"); });
__attribute__((used, section("api_list"))) constinit auto api_idiq = create_api_variable("idiq", var_idiq);
       // TORQUE_CONTROLLER_DEBUG_VARIABLES(api, System::actuator_.main_loop_.torque_controller_);
      //  STATE_CONTROLLER_DEBUG_VARIABLES(api, System::actuator_.main_loop_.state_controller_);
const APIFloat var_tgain(&System::actuator_.main_loop_.torque_sensor_.gain_);
__attribute__((used, section("api_list"))) constinit auto api_tgain = create_api_variable("tgain", var_tgain);
const APIFloat var_tbias(&System::actuator_.main_loop_.torque_sensor_bias_);
__attribute__((used, section("api_list"))) constinit auto api_tbias = create_api_variable("tbias", var_tbias);
const APIFloat var_torque(&System::actuator_.main_loop_.status_.torque);
__attribute__((used, section("api_list"))) constinit auto api_torque = create_api_variable("torque", var_torque);
const APIFloat var_t_i_correction(&System::actuator_.main_loop_.param_.torque_correction);
__attribute__((used, section("api_list"))) constinit auto api_t_i_correction = create_api_variable("t_i_correction", var_t_i_correction);
const APICallback var_log(System::get_log, System::log);
__attribute__((used, section("api_list"))) constinit auto api_log = create_api_variable("log", var_log);
const APICallback var_old_log([]{ return logger.get_old_log(); });
__attribute__((used, section("api_list"))) constinit auto api_old_log = create_api_variable("old_log", var_old_log);
const APICallback var_log_reset([]()->std::string{ logger.reset_read_front(); return "ok"; });
__attribute__((used, section("api_list"))) constinit auto api_log_reset = create_api_variable("log_reset", var_log_reset);
const APICallbackUint32 var_log_num([]{ return logger.num_elements(); });
__attribute__((used, section("api_list"))) constinit auto api_log_num = create_api_variable("log_num", var_log_num);
const APIStringView var_messages_version(MOTOR_MESSAGES_VERSION);
__attribute__((used, section("api_list"))) constinit auto api_messages_version = create_api_variable("messages_version", var_messages_version);
const APICallbackInt32 var_index_pos([]{ return System::actuator_.fast_loop_.encoder_.get_index_pos(); });
__attribute__((used, section("api_list"))) constinit auto api_index_pos = create_api_variable("index_pos", var_index_pos);
const APICallbackUint8 var_index_received([]()->uint8_t{return System::actuator_.fast_loop_.encoder_.index_received();});
__attribute__((used, section("api_list"))) constinit auto api_index_received = create_api_variable("index_received", var_index_received);
const APIFloat var_index_offset_measured(&System::actuator_.fast_loop_.motor_index_electrical_offset_measured_);
__attribute__((used, section("api_list"))) constinit auto api_index_offset_measured = create_api_variable("index_offset_measured", var_index_offset_measured);
const APIInt32 var_electrical_zero_pos(&System::actuator_.fast_loop_.motor_electrical_zero_pos_);
__attribute__((used, section("api_list"))) constinit auto api_electrical_zero_pos = create_api_variable("electrical_zero_pos", var_electrical_zero_pos);
const APIUint32 var_mcpr(&param->fast_loop_param.motor_encoder.cpr);
__attribute__((used, section("api_list"))) constinit auto api_mcpr = create_api_variable("mcpr", var_mcpr);
const APIFloat var_ocpr(&param->main_loop_param.output_encoder.cpr);
__attribute__((used, section("api_list"))) constinit auto api_ocpr = create_api_variable("ocpr", var_ocpr);
const APICallbackFloat var_irange([](){ return 2048*param->fast_loop_param.adc1_gain; });
__attribute__((used, section("api_list"))) constinit auto api_irange = create_api_variable("irange", var_irange);
const APICallbackUint32 var_stack_free(get_stack_free);
__attribute__((used, section("api_list"))) constinit auto api_stack_free = create_api_variable("stack_free", var_stack_free);
const APICallbackUint32 var_stack_used(get_stack_used);
__attribute__((used, section("api_list"))) constinit auto api_stack_used = create_api_variable("stack_used", var_stack_used);
const APICallbackUint32 var_heap_free(get_heap_free);
__attribute__((used, section("api_list"))) constinit auto api_heap_free = create_api_variable("heap_free", var_heap_free);
const APICallbackUint32 var_heap_used(get_heap_used);
__attribute__((used, section("api_list"))) constinit auto api_heap_used = create_api_variable("heap_used", var_heap_used);
const APICallbackUint32 var_heap_current_free(get_current_heap_free);
__attribute__((used, section("api_list"))) constinit auto api_heap_current_free = create_api_variable("heap_current_free", var_heap_current_free);
const APICallbackUint32 var_heap_current_used(get_current_heap_used);
__attribute__((used, section("api_list"))) constinit auto api_heap_current_used = create_api_variable("heap_current_used", var_heap_current_used);
const APICallbackUint32 var_malloc([](){ return (uint32_t) get_heap_free() + get_heap_used(); }, 
            [](uint32_t u) {
                try { char* volatile c = new char[u]; delete [] c; }
                catch(...) { logger.log_printf("couldn't allocate %d", u); } });
__attribute__((used, section("api_list"))) constinit auto api_malloc = create_api_variable("malloc", var_malloc);
const APIFloat var_vbus_min(&System::actuator_.main_loop_.vbus_min_);
__attribute__((used, section("api_list"))) constinit auto api_vbus_min = create_api_variable("vbus_min", var_vbus_min);
const APIFloat var_vbus_max(&System::actuator_.main_loop_.vbus_max_);
__attribute__((used, section("api_list"))) constinit auto api_vbus_max = create_api_variable("vbus_max", var_vbus_max);
const APIFloat var_ia_bias(&System::actuator_.fast_loop_.ia_bias_);
__attribute__((used, section("api_list"))) constinit auto api_ia_bias = create_api_variable("ia_bias", var_ia_bias);
const APIFloat var_ib_bias(&System::actuator_.fast_loop_.ib_bias_);
__attribute__((used, section("api_list"))) constinit auto api_ib_bias = create_api_variable("ib_bias", var_ib_bias);
const APIFloat var_ic_bias(&System::actuator_.fast_loop_.ic_bias_);
__attribute__((used, section("api_list"))) constinit auto api_ic_bias = create_api_variable("ic_bias", var_ic_bias);
const APIFloat var_power(&System::actuator_.main_loop_.status_.fast_loop.power);
__attribute__((used, section("api_list"))) constinit auto api_power = create_api_variable("power", var_power);
const APIFloat var_power_avg(&System::actuator_.main_loop_.status_.power);
__attribute__((used, section("api_list"))) constinit auto api_power_avg = create_api_variable("power_avg", var_power_avg);
const APIUint32 var_energy(&System::actuator_.main_loop_.status_.fast_loop.energy_uJ);
__attribute__((used, section("api_list"))) constinit auto api_energy = create_api_variable("energy", var_energy);
const APICallback var_fast_log([](){
            System::actuator_.main_loop_.lock_status_log();
            FastLog log;
            std::string out;
            for(int i=0; i<FAST_LOG_LENGTH; i++) {
                FastLoopStatus &status = System::actuator_.fast_loop_.status_log_.next();
                log.timestamp = status.timestamp;
                log.electrical_position = status.foc_command.measured.motor_encoder / System::actuator_.fast_loop_.foc_->num_poles_;
                log.command_iq = status.foc_status.command.i_q;
                log.command_id = status.foc_status.command.i_d;
                log.measured_iq = status.foc_status.measured.i_q;
                log.measured_id = status.foc_status.measured.i_d;
                log.command_vq = status.foc_status.command.v_q;
                log.command_vd = status.foc_status.command.v_d;
                log.vbus = status.vbus;
                log.ibus = status.ibus;
                std::string s((char *) &log, sizeof(log));
                System::actuator_.fast_loop_.status_log_.finish();
                out += s;
            }
            System::actuator_.main_loop_.unlock_status_log();
            return out; });
__attribute__((used, section("api_list"))) constinit auto api_fast_log = create_api_variable("fast_log", var_fast_log);
const APICallback var_fast_log2([](){
            System::actuator_.main_loop_.lock_status_log();
            FastLog2 log;
            std::string out;
            for(int i=0; i<FAST_LOG_LENGTH; i++) {
                FastLoopStatus &status = System::actuator_.fast_loop_.status_log_.next();
                log.timestamp = status.timestamp;
                log.electrical_position = status.foc_command.measured.motor_encoder / System::actuator_.fast_loop_.foc_->num_poles_;
                log.measured_ia = status.foc_command.measured.i_a;
                log.measured_ib = status.foc_command.measured.i_b;
                log.measured_ic = status.foc_command.measured.i_c;
                log.command_va = status.foc_status.command.v_a;
                log.command_vb = status.foc_status.command.v_b;
                log.command_vc = status.foc_status.command.v_c;
                log.motor_encoder_flags = status.foc_command.motor_encoder_flags;
                log.mode = status.mode;
                std::string s((char *) &log, sizeof(log));
                System::actuator_.fast_loop_.status_log_.finish();
                out += s;
            }
            System::actuator_.main_loop_.unlock_status_log();
            return out; });
__attribute__((used, section("api_list"))) constinit auto api_fast_log2 = create_api_variable("fast_log2", var_fast_log2);
const APICallbackFloat var_beep([](){ return 0.0f; }, [](float f){ System::actuator_.fast_loop_.beep_on(f); });
__attribute__((used, section("api_list"))) constinit auto api_beep = create_api_variable("beep", var_beep);
const APIFloat var_beep_frequency(&System::actuator_.fast_loop_.param_.beep_frequency);
__attribute__((used, section("api_list"))) constinit auto api_beep_frequency = create_api_variable("beep_frequency", var_beep_frequency);
const APIFloat var_beep_amplitude(&System::actuator_.fast_loop_.param_.beep_amplitude);
__attribute__((used, section("api_list"))) constinit auto api_beep_amplitude = create_api_variable("beep_amplitude", var_beep_amplitude);
const APICallbackFloat var_zero_current_sensors([](){ return 0.0f; }, [](float f){ System::actuator_.fast_loop_.zero_current_sensors_on(f); });
__attribute__((used, section("api_list"))) constinit auto api_zero_current_sensors = create_api_variable("zero_current_sensors", var_zero_current_sensors);
const APICallback var_disable_safe_mode([]()->std::string{ System::actuator_.main_loop_.error_mask_.all = ERROR_MASK_NONE; return "ok"; });
__attribute__((used, section("api_list"))) constinit auto api_disable_safe_mode = create_api_variable("disable_safe_mode", var_disable_safe_mode);
const APICallback var_error_mask([](){ return u32_to_hex(System::actuator_.main_loop_.error_mask_.all); },
                [](std::string s){ try {
                        System::actuator_.main_loop_.error_mask_.all = std::stoul(s, nullptr, 16) & ERROR_MASK_ALL;}
                    catch(...) {} });
__attribute__((used, section("api_list"))) constinit auto api_error_mask = create_api_variable("error_mask", var_error_mask);
const APICallback var_help([](){ return System::api.get_all_api_variables(); });
__attribute__((used, section("api_list"))) constinit auto api_help = create_api_variable("help", var_help);
const APICallbackUint16 var_api_length([](){ return System::api.get_api_length(); });
__attribute__((used, section("api_list"))) constinit auto api_api_length = create_api_variable("api_length", var_api_length);
const APIBool var_disable_position_limits(&System::actuator_.main_loop_.position_limits_disable_);
__attribute__((used, section("api_list"))) constinit auto api_disable_position_limits = create_api_variable("disable_position_limits", var_disable_position_limits);
const APIFloat var_jkpj(&System::actuator_.main_loop_.joint_position_controller_.param_.kpj);
__attribute__((used, section("api_list"))) constinit auto api_jkpj = create_api_variable("jkpj", var_jkpj);
const APIFloat var_motor_position_raw(&System::actuator_.fast_loop_.motor_position_);
__attribute__((used, section("api_list"))) constinit auto api_motor_position_raw = create_api_variable("motor_position_raw", var_motor_position_raw);
const APIFloat var_obias(&System::actuator_.main_loop_.output_encoder_bias_);
__attribute__((used, section("api_list"))) constinit auto api_obias = create_api_variable("obias", var_obias);
const APIFloat var_mbias(&System::actuator_.main_loop_.motor_encoder_bias_);
__attribute__((used, section("api_list"))) constinit auto api_mbias = create_api_variable("mbias", var_mbias);
const APIFloat var_ttgain(&System::actuator_.main_loop_.calibration_.torque_sensor.table_gain);
__attribute__((used, section("api_list"))) constinit auto api_ttgain = create_api_variable("ttgain", var_ttgain);
        API_ADD_FILTER(id_filter, FirstOrderLowPassFilter, System::actuator_.fast_loop_.foc_->id_filter_);
        API_ADD_FILTER(iq_filter, FirstOrderLowPassFilter, System::actuator_.fast_loop_.foc_->iq_filter_);
        API_ADD_FILTER(output_iq_filter, FirstOrderLowPassFilter, System::actuator_.fast_loop_.iq_filter_);
        API_ADD_FILTER(output_motor_velocity_filter, FirstOrderLowPassFilter, System::actuator_.fast_loop_.motor_velocity_filter_);
        API_ADD_FILTER(output_motor_position_filter, FirstOrderLowPassFilter, System::actuator_.fast_loop_.motor_position_filter_);
const APIFloat var_startup_phase_lock_current(&param->startup_param.phase_lock_current);
__attribute__((used, section("api_list"))) constinit auto api_startup_phase_lock_current = create_api_variable("startup_phase_lock_current", var_startup_phase_lock_current);
const APIFloat var_startup_mbias(&System::actuator_.startup_motor_bias_);
__attribute__((used, section("api_list"))) constinit auto api_startup_mbias = create_api_variable("startup_mbias", var_startup_mbias);
const APICallback var_set_startup_bias([]()->std::string{ System::actuator_.set_bias(); return "ok"; });
__attribute__((used, section("api_list"))) constinit auto api_set_startup_bias = create_api_variable("set_startup_bias", var_set_startup_bias);
const APIFloat var_odir(&System::actuator_.main_loop_.output_encoder_dir_);
__attribute__((used, section("api_list"))) constinit auto api_odir = create_api_variable("odir", var_odir);
const APIFloat var_tdir(&System::actuator_.main_loop_.torque_sensor_dir_);
__attribute__((used, section("api_list"))) constinit auto api_tdir = create_api_variable("tdir", var_tdir);
const APIFloat var_mdir(&System::actuator_.fast_loop_.motor_encoder_dir_);
__attribute__((used, section("api_list"))) constinit auto api_mdir = create_api_variable("mdir", var_mdir);
        //API_ADD_FILTER(output_motor_velocity_filter2, FirstOrderLowPassFilter, System::actuator_.main_loop_.motor_velocity_filter_);
        API_ADD_FILTER(output_motor_position_filter2, FirstOrderLowPassFilter, System::actuator_.main_loop_.motor_position_filter_);
        //API_ADD_FILTER(output_output_velocity_filter, FirstOrderLowPassFilter, System::actuator_.main_loop_.output_velocity_filter_);
        API_ADD_FILTER(output_output_position_filter, FirstOrderLowPassFilter, System::actuator_.main_loop_.output_position_filter_);
        API_ADD_FILTER(output_torque_filter, FirstOrderLowPassFilter, System::actuator_.main_loop_.torque_filter_);
const APIFloat var_idir(&System::actuator_.fast_loop_.current_direction_);
__attribute__((used, section("api_list"))) constinit auto api_idir = create_api_variable("idir", var_idir);
const APICallbackUint32 var_uptime(get_uptime);
__attribute__((used, section("api_list"))) constinit auto api_uptime = create_api_variable("uptime", var_uptime);
const APICallback var_power_on_time([]{
            char t[9];
            RTClock::power_on_time(t);
            return std::string(t);
        });
__attribute__((used, section("api_list"))) constinit auto api_power_on_time = create_api_variable("power_on_time", var_power_on_time);
const APIInt32 var_menc(&System::actuator_.fast_loop_.motor_enc);
__attribute__((used, section("api_list"))) constinit auto api_menc = create_api_variable("menc", var_menc);
const APICallbackInt32 var_oenc([](){ return System::actuator_.main_loop_.output_encoder_.get_value(); });
__attribute__((used, section("api_list"))) constinit auto api_oenc = create_api_variable("oenc", var_oenc);
const APIFloat var_amax(&System::actuator_.main_loop_.admittance_controller_.torque_controller_.command_max_);
__attribute__((used, section("api_list"))) constinit auto api_amax = create_api_variable("amax", var_amax);
const APIFloat var_akp(&System::actuator_.main_loop_.admittance_controller_.torque_controller_.kp_);
__attribute__((used, section("api_list"))) constinit auto api_akp = create_api_variable("akp", var_akp);
const APIFloat var_Tmotor_est(&System::actuator_.main_loop_.status_.motor_temperature_estimate);
__attribute__((used, section("api_list"))) constinit auto api_Tmotor_est = create_api_variable("Tmotor_est", var_Tmotor_est);
        API_ADD_FILTER(a_output_filter, FirstOrderLowPassFilter, System::actuator_.main_loop_.admittance_controller_.torque_controller_.output_filter_);
const APICallback var_fast_loop_status([](){ 
            FastLoopStatus status = System::actuator_.fast_loop_.status_.top();
            uint8_t len = 192;
            char c[len];
            std::snprintf(c, len, "%" PRIu32", %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
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
__attribute__((used, section("api_list"))) constinit auto api_fast_loop_status = create_api_variable("fast_loop_status", var_fast_loop_status);
const APIFloat var_id_des(&System::actuator_.fast_loop_.foc_command_.desired.i_d);
__attribute__((used, section("api_list"))) constinit auto api_id_des = create_api_variable("id_des", var_id_des);
const APICallback var_trigger_fast_log([]()->std::string{ System::actuator_.fast_loop_.trigger_status_log(); return "triggered"; });
__attribute__((used, section("api_list"))) constinit auto api_trigger_fast_log = create_api_variable("trigger_fast_log", var_trigger_fast_log);
const APICallbackFloat var_ilimit([](){ return System::actuator_.fast_loop_.foc_->get_iq_limit(); },
            [](float f){ System::actuator_.fast_loop_.foc_->set_iq_limit(f); });
__attribute__((used, section("api_list"))) constinit auto api_ilimit = create_api_variable("ilimit", var_ilimit);
const APICallbackFloat var_idlimit([](){ return System::actuator_.fast_loop_.foc_->get_id_limit(); },
            [](float f){ System::actuator_.fast_loop_.foc_->set_id_limit(f); });
__attribute__((used, section("api_list"))) constinit auto api_idlimit = create_api_variable("idlimit", var_idlimit);
const APIFloat var_num_poles(&System::actuator_.fast_loop_.foc_->num_poles_);
__attribute__((used, section("api_list"))) constinit auto api_num_poles = create_api_variable("num_poles", var_num_poles);
const APICallbackUint32 var_timestamp(get_clock);
__attribute__((used, section("api_list"))) constinit auto api_timestamp = create_api_variable("timestamp", var_timestamp);
const APICallbackFloat var_mrollover([](){ return System::actuator_.fast_loop_.get_rollover(); });
__attribute__((used, section("api_list"))) constinit auto api_mrollover = create_api_variable("mrollover", var_mrollover);
const APIFloat var_gear_ratio(&param->startup_param.gear_ratio);
__attribute__((used, section("api_list"))) constinit auto api_gear_ratio = create_api_variable("gear_ratio", var_gear_ratio);
const APIStringView var_version(OBOT_VERSION);
__attribute__((used, section("api_list"))) constinit auto api_version = create_api_variable("version", var_version);
const APIStringView var_obot_hash(OBOT_HASH);
__attribute__((used, section("api_list"))) constinit auto api_obot_hash = create_api_variable("obot_hash", var_obot_hash);
const APIStringView var_motorlib_hash(MOTORLIB_HASH);
__attribute__((used, section("api_list"))) constinit auto api_motorlib_hash = create_api_variable("motorlib_hash", var_motorlib_hash);
const APIStringView var_name(param->name);
__attribute__((used, section("api_list"))) constinit auto api_name = create_api_variable("name", var_name);
        uint32_t api_timeout_us = 10000;
const APIUint32 var_api_timeout(&api_timeout_us);
__attribute__((used, section("api_list"))) constinit auto api_api_timeout = create_api_variable("api_timeout", var_api_timeout);
const APIStringView var_notes(NOTES);
__attribute__((used, section("api_list"))) constinit auto api_notes = create_api_variable("notes", var_notes);
const APIFloat var_tuning_desired(&System::actuator_.main_loop_.tuning_trajectory_generator_.trajectory_value_.value );
__attribute__((used, section("api_list"))) constinit auto api_tuning_desired = create_api_variable("tuning_desired", var_tuning_desired);
const APIFloat var_dft_frequency(&System::actuator_.main_loop_.dft_.desired_.frequency_last_);
__attribute__((used, section("api_list"))) constinit auto api_dft_frequency = create_api_variable("dft_frequency", var_dft_frequency);
const APIFloat var_dft_desired_magnitude(&System::actuator_.main_loop_.dft_.desired_.magnitude_last_);
__attribute__((used, section("api_list"))) constinit auto api_dft_desired_magnitude = create_api_variable("dft_desired_magnitude", var_dft_desired_magnitude);
const APIFloat var_dft_phase(&System::actuator_.main_loop_.dft_.phase_);
__attribute__((used, section("api_list"))) constinit auto api_dft_phase = create_api_variable("dft_phase", var_dft_phase);
const APIFloat var_dft_magnitude(&System::actuator_.main_loop_.dft_.magnitude_);
__attribute__((used, section("api_list"))) constinit auto api_dft_magnitude = create_api_variable("dft_magnitude", var_dft_magnitude);
const APICallbackHex<uint32_t> var_gpioa([](){ return GPIOA->IDR; }, [](uint32_t u){ GPIOA->ODR = u; });
__attribute__((used, section("api_list"))) constinit auto api_gpioa = create_api_variable("gpioa", var_gpioa);
const APICallbackHex<uint32_t> var_gpiob([](){ return GPIOB->IDR; }, [](uint32_t u){ GPIOB->ODR = u; });
__attribute__((used, section("api_list"))) constinit auto api_gpiob = create_api_variable("gpiob", var_gpiob);
const APICallbackHex<uint32_t> var_gpioc([](){ return GPIOC->IDR; }, [](uint32_t u){ GPIOC->ODR = u; });
__attribute__((used, section("api_list"))) constinit auto api_gpioc = create_api_variable("gpioc", var_gpioc);
const APICallbackHex<uint32_t> var_gpiod([](){ return GPIOD->IDR; }, [](uint32_t u){ GPIOD->ODR = u; });
__attribute__((used, section("api_list"))) constinit auto api_gpiod = create_api_variable("gpiod", var_gpiod);
const APICallbackHex<uint32_t> var_gpioe([](){ return GPIOE->IDR; }, [](uint32_t u){ GPIOE->ODR = u; });
__attribute__((used, section("api_list"))) constinit auto api_gpioe = create_api_variable("gpioe", var_gpioe);
const APICallback var_board_name([]()->std::string{ return otp->version == 1 ? otp->name : ""; });
__attribute__((used, section("api_list"))) constinit auto api_board_name = create_api_variable("board_name", var_board_name);
const APICallback var_board_rev([]()->std::string{ return otp->version == 1 ? otp->rev : ""; });
__attribute__((used, section("api_list"))) constinit auto api_board_rev = create_api_variable("board_rev", var_board_rev);
const APIInt32 var_board_num(&otp->num);
__attribute__((used, section("api_list"))) constinit auto api_board_num = create_api_variable("board_num", var_board_num);
const APICallback var_long_packet([]{ 
          char long_packet[MAX_API_DATA_SIZE+1] = "This is a long packet test\n";
          int len = std::strlen(long_packet);
          for (int i=0; i<MAX_API_DATA_SIZE-len; i++) {
            long_packet[i+len] = '0' + (i % 10);
          }
          return std::string((char *) &long_packet, sizeof(long_packet));
        });
__attribute__((used, section("api_list"))) constinit auto api_long_packet = create_api_variable("long_packet", var_long_packet);
const APICallback var_really_long_packet([]{
          char long_packet[MAX_API_LONG_DATA_SIZE];
          for (int i=0; i<MAX_API_LONG_DATA_SIZE; i++) {
            long_packet[i] = '0' + (i % 10);
          }
          return std::string((char *) &long_packet, sizeof(long_packet));
        });
__attribute__((used, section("api_list"))) constinit auto api_really_long_packet = create_api_variable("really_long_packet", var_really_long_packet);
const APIStringView var_config(CONFIG);
__attribute__((used, section("api_list"))) constinit auto api_config = create_api_variable("config", var_config);
const APIStringView var_serial(get_serial_number());
__attribute__((used, section("api_list"))) constinit auto api_serial = create_api_variable("serial", var_serial);
const APIFloat var_olimit_max(&System::actuator_.main_loop_.encoder_limits_.output_hard_max);
__attribute__((used, section("api_list"))) constinit auto api_olimit_max = create_api_variable("olimit_max", var_olimit_max);
const APIFloat var_olimit_min(&System::actuator_.main_loop_.encoder_limits_.output_hard_min);
__attribute__((used, section("api_list"))) constinit auto api_olimit_min = create_api_variable("olimit_min", var_olimit_min);
const APIFloat var_mlimit_max(&System::actuator_.main_loop_.encoder_limits_.motor_hard_max);
__attribute__((used, section("api_list"))) constinit auto api_mlimit_max = create_api_variable("mlimit_max", var_mlimit_max);
const APIFloat var_mlimit_min(&System::actuator_.main_loop_.encoder_limits_.motor_hard_min);
__attribute__((used, section("api_list"))) constinit auto api_mlimit_min = create_api_variable("mlimit_min", var_mlimit_min);
const APIFloat var_msoftlimit_max(&System::actuator_.main_loop_.encoder_limits_.motor_controlled_max);
__attribute__((used, section("api_list"))) constinit auto api_msoftlimit_max = create_api_variable("msoftlimit_max", var_msoftlimit_max);
const APIFloat var_msoftlimit_min(&System::actuator_.main_loop_.encoder_limits_.motor_controlled_min);
__attribute__((used, section("api_list"))) constinit auto api_msoftlimit_min = create_api_variable("msoftlimit_min", var_msoftlimit_min);
const APICallbackUint8 var_is_sbank([]()->uint8_t{ return (*((uint8_t *) 0x1fff7802) & 0x40) == 0; });
__attribute__((used, section("api_list"))) constinit auto api_is_sbank = create_api_variable("is_sbank", var_is_sbank);
const APICallbackFloat var_invalid_command_leak_rate_s([]{
            return System::actuator_.main_loop_.invalid_command_fault_.get_leak_period_s(System::actuator_.main_loop_.dt_); },
            [](float f){ System::actuator_.main_loop_.invalid_command_fault_.set_leak_period(f, System::actuator_.main_loop_.dt_); });
__attribute__((used, section("api_list"))) constinit auto api_invalid_command_leak_rate_s = create_api_variable("invalid_command_leak_rate_s", var_invalid_command_leak_rate_s);
const APIUint32 var_invalid_command_limit(&System::actuator_.main_loop_.invalid_command_limit_);
__attribute__((used, section("api_list"))) constinit auto api_invalid_command_limit = create_api_variable("invalid_command_limit", var_invalid_command_limit);
const APIUint32 var_invalid_command_count(&System::actuator_.main_loop_.invalid_command_fault_.count_);
__attribute__((used, section("api_list"))) constinit auto api_invalid_command_count = create_api_variable("invalid_command_count", var_invalid_command_count);
const APICallbackHex<uint32_t> var_fault([](){ return System::actuator_.main_loop_.status_.error.all; });
__attribute__((used, section("api_list"))) constinit auto api_fault = create_api_variable("fault", var_fault);
const APICallback var_fault_str([](){
            char c[600];
            System::actuator_.main_loop_.get_fault_str(c, 600);
            return std::string(c);
        });
__attribute__((used, section("api_list"))) constinit auto api_fault_str = create_api_variable("fault_str", var_fault_str);
const APICallbackUint8 var_reset([]()->uint8_t{ NVIC_SystemReset(); return 0; });
__attribute__((used, section("api_list"))) constinit auto api_reset = create_api_variable("reset", var_reset);