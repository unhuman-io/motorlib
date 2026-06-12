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

#ifndef CUSTOM_MAIN_MAINTENANCE_ASYNC
[[gnu::weak]] Task<void> main_maintenance_async(CycleScheduler& sched) {
    while (1) {
        main_maintenance(); // Calls the void version
        co_await sched.yield();
    }
}
#endif

Logger::CIndex log_index __attribute__((section(".noload")));
char log_queue[LOGGING_MAX_SIZE] __attribute__((section(".noload")));
__attribute__ ((init_priority(LOGGER_INIT_PRIORITY))) Logger logger(log_index, log_queue);
RoundRobinLogger round_robin_logger;

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
