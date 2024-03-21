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

void system_loop_interrupt() {
    us_delay(200);
}

Logger logger;
RoundRobinLogger round_robin_logger;
uint32_t System::count_ = 0;
ParameterAPI System::api = {};