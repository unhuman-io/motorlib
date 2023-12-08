#include <string>

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

Logger logger;
RoundRobinLogger round_robin_logger;
uint32_t System::count_ = 0;
ParameterAPI System::api = {};
FrequencyLimiter System::cpu_rate(10);
float System::cpu_comint = 0;
float System::cpu_mainloop = 0;
float System::cpu_fastloop = 0;
uint32_t System::last_comint_count;
uint32_t System::last_mainloop_count;
uint32_t System::last_fastloop_count;