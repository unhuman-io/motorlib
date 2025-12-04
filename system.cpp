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

__attribute__((used, section("api_list"))) constinit const auto a = create_api_variable<APIUint32>("api_memory_used", &ParameterAPI::AllocatorBase::index_);