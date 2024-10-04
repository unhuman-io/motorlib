#include <string>
#include <fcntl.h>
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

__attribute__ ((init_priority(LOGGER_INIT_PRIORITY))) Logger logger;
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
