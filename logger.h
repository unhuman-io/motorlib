#ifndef UNHUMAN_MOTORLIB_LOGGER_H_
#define UNHUMAN_MOTORLIB_LOGGER_H_

#include <stdarg.h>

#include <queue>
#include <string>

class Logger {
 public:
  void log(std::string str) {
    if (log_queue_.size() < 103) {
      log_queue_.push(str);
    }
  }
  void log_once(std::string str) {
    if (str != log_queue_.back()) {
      log(str);
    }
  }
  std::string get_log() {
    std::string str = "log end";
    if (!log_queue_.empty()) {
      str = log_queue_.front();
      log_queue_.pop();
    }
    return str;
  }
  void log_printf(const char *s, ...) {
    va_list args;
    char sout[MAX_API_DATA_SIZE];
    va_start(args, s);
    vsnprintf(sout, MAX_API_DATA_SIZE, s, args);
    va_end(args);
    log(sout);
  }

 private:
  std::queue<std::string> log_queue_ = {};
};

extern Logger logger;

#endif  // UNHUMAN_MOTORLIB_LOGGER_H_
