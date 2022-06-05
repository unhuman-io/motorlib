#pragma once
#include <string>
#include <queue>
#include <stdarg.h>

class Logger {
 public:
    void log(std::string str) {
        if (log_queue_.size() < 20) {
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
        char sout[64];
        vsnprintf(sout, 64, s, args);
        log(sout);
    }
 private:
    std::queue<std::string> log_queue_ = {};
};

extern Logger logger;
