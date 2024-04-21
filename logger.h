#ifndef UNHUMAN_MOTORLIB_LOGGER_H_
#define UNHUMAN_MOTORLIB_LOGGER_H_

#include <string>
#include <queue>
#include <stdarg.h>
#include "util.h"
#include "messages.h"
#include <cstring>

#define LOGGING_MAX_SIZE 2048
class Logger {
 public:
    void log(std::string_view str) {
        //log_queue_.push("(" + std::to_string(get_uptime()) + " " + std::to_string(get_clock()) + ") " + str);

        CIndex next_ptr = back_ + str.size() + 1;

        CIndex front_next = front_;
        if (next_ptr > front_) {
            while(true) {
                ++front_next;
                if (log_queue_[front_next] == '\0') {
                    num_elements_--;
                    if (front_next > next_ptr) {
                        break;
                    }
                }
            }
        }
        front_ = front_next;

        for (size_t i = 0; i < str.size()+1; i++) {
            log_queue_[back_] = str[i];
            ++back_;
        }

        num_elements_++;
    }
    void log_once(std::string_view str) {
        // if (str != log_queue_.back()) {
        //     log(str);
        // }
    }
    std::string get_log() {
        std::string str = "log end";

        if (num_elements_ > 0) {
            if (front_.wrapped()) {
                str = std::string(&log_queue_[front_], LOGGING_MAX_SIZE - front_) + std::string(log_queue_);
            } else {
                str = &log_queue_[front_];   
            }
            front_ = front_ + str.size() + 1;
            num_elements_--;
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
    class CIndex {
     public:
        CIndex() = default;
        CIndex(uint32_t value) : value_(value) {
            wrap();
        }
        void inc() { value_++; wrap(); }
        CIndex& operator++() { inc(); return *this; }
        bool operator>(const CIndex& other) const {
            if (!wrapped_ == !other.wrapped_) {
                return value_ > other.value_;
            }
            if (wrapped_ && !other.wrapped_) {
                return false;
            } else {
                // !wrapped_ && other.wrapped_
                return true;
            }
        }
        bool wrapped() const { return wrapped_; }
        operator uint32_t() const { return value_; }
        void wrap() {
            wrapped_ = value_ >= LOGGING_MAX_SIZE;
            value_ %= LOGGING_MAX_SIZE;
        }
     private:
        uint32_t value_ = 0;
        bool wrapped_ = false;
    };

    uint32_t num_elements_ = 0;
    CIndex front_;
    CIndex back_;
    char log_queue_[LOGGING_MAX_SIZE];
};

extern Logger logger;

#endif  // UNHUMAN_MOTORLIB_LOGGER_H_
