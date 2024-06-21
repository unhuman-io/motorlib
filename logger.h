#ifndef UNHUMAN_MOTORLIB_LOGGER_H_
#define UNHUMAN_MOTORLIB_LOGGER_H_

#include <string>
#include <stdarg.h>
#ifndef LOGGER_TEST
#include "util.h"
#endif
#include "messages.h"
#include <atomic>
#include <string_view>

#define LOGGING_MAX_SIZE 4096
class Logger {
 public:
    void log(std::string_view str) {
        char header[50];
        snprintf(header, sizeof(header), "(%lu %lu) ", get_uptime(), get_clock());
        front_log_.set_value(front_atomic_.load(std::memory_order_acquire));
        log_raw(header);
        log_raw(str);
        log_raw('\0');
        front_atomic_.store(front_log_, std::memory_order_release);
        num_elements_++;
    }
    // void log_once(std::string_view str) {
    //     // if (str != log_queue_.back()) {
    //     //     log(str);
    //     // }
    // }

    // get_log must be called from the same or lower priority task as log
    std::string get_log(&CIndex front) {
        std::string str;

        if (!empty()) {
            bool success = false;
            do {
                str = "";
                CIndex front_start = front_atomic_.load(std::memory_order_acquire);
                uint32_t front_expected = front_next;
                do {
                    str += log_queue_[front_next];
                    ++front_next;
                } while (log_queue_[front_next] != '\0');
                ++front_next;
                CIndex front_start2 = front_atomic_.load(std::memory_order_acquire);
                success = front_start2 == front_start;
            } while (!success);
            
        } else {
            str = "log end";
        }
        return str;
    }
    bool empty() const {
        return front_atomic_.load(std::memory_order_acquire) == back_;
    }
    void log_printf(const char *s, ...) {
        va_list args;
        char sout[MAX_API_DATA_SIZE];
        va_start(args, s);
        vsnprintf(sout, MAX_API_DATA_SIZE, s, args);
        va_end(args);
        log(sout);
    }
    uint32_t num_elements() const { return num_elements_; }
    void reset() {

    }

    static std::string_view extract_string(std::string_view str) {
        std::string_view data = str.substr(str.find(") ") + 2);
        return data;
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
        void set_value(uint32_t value) {
            value_ = value;
        }
        bool operator==(const CIndex& other) const {
            return value_ == other.value_;
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

    void log_raw(char c) {
        log_queue_[back_] = c;
        ++back_;
        move_front();
    }

    void log_raw(std::string_view str) {
        for (size_t i = 0; i < str.size(); i++) {
            log_raw(str[i]);
        }
    }

    void move_front() {
        if (back_ == front_log_) {
            // find next front
            while (log_queue_[++front_log_] != '\0');
            ++front_log_;
            num_elements_--;
        }
    }

    uint32_t num_elements_ = 0;
    CIndex front_log_;
    CIndex back_;
    std::atomic<uint32_t> front_atomic_{0};
    char log_queue_[LOGGING_MAX_SIZE] = {};
};

extern Logger logger;

#endif  // UNHUMAN_MOTORLIB_LOGGER_H_
