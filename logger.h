#ifndef UNHUMAN_MOTORLIB_LOGGER_H_
#define UNHUMAN_MOTORLIB_LOGGER_H_

#include <string>
#include <stdarg.h>
//#include "util.h"
#include "messages.h"
#include <cstring>
#include <iostream>
#include <atomic>

#define LOGGING_MAX_SIZE 2048
class Logger {
 public:
    void log(std::string_view str) {
        //log_queue_.push("(" + std::to_string(get_uptime()) + " " + std::to_string(get_clock()) + ") " + str);
        char header[50];
        snprintf(header, sizeof(header), "(%d %d) ", 0,0);// get_uptime(), get_clock());
        log_raw(header);
        log_raw(str);
        log_raw('\0');  
        num_elements_++;
    }
    void log_once(std::string_view str) {
        // if (str != log_queue_.back()) {
        //     log(str);
        // }
    }
    std::string get_log() {
        std::string str;

        if (num_elements_ > 0) {
            do {
                str = "";
                CIndex front_next = front_;
                CIndex front_expected = front_next;
                do {
                    str += log_queue_[front_next];
                    ++front_next;
                } while (log_queue_[front_next] != '\0');
                front_ = front_next;
                compare_exchange_strong(front_expected, front_next);
                num_elements_--;
            } while (0);
            
        } else {
            str = "log end";
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
    uint32_t num_elements() const { return num_elements_; }
 private:
    class CIndex {
     public:
        CIndex() = default;
        CIndex(uint32_t value) : value_(value) {
            wrap();
        }
        void inc() { value_++; wrap(); }
        CIndex& operator++() { inc(); return *this; }
        // bool operator>(const CIndex& other) const {
        //     if (!wrapped_ == !other.wrapped_) {
        //         return value_ > other.value_;
        //     }
        //     if (wrapped_ && !other.wrapped_) {
        //         return false;
        //     } else {
        //         // !wrapped_ && other.wrapped_
        //         return true;
        //     }
        // }
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
        // CIndex next_back = back_ + str.size() + 1;

        // CIndex front_next = front_;
        // if (front_ > back_ && next_back >= front_) {
        //     int i=0;
        //     while(true) {
        //         ++front_next;
        //         if (log_queue_[front_next] == '\0') {
        //             num_elements_--;
        //             ++front_next;
        //             if (front_next >= next_back) {
        //                 break;
        //             }
        //         }
        //         if (i++ > 100000) {
        //             std::cout << "what " << front_next << std::endl;
        //             exit(1);
        //         }
        //     }
        // }
        // std::cout << "front_next: " << front_next << std::endl;
        // std::cout << "front: " << front_ << std::endl;
        // front_ = front_next;

        for (size_t i = 0; i < str.size(); i++) {
            log_raw(str[i]);
        }
    }

    void move_front() {
        if (back_ == front_) {
            // find next front
            while (log_queue_[++front_] != '\0');
            ++front_;
            num_elements_--;
        }
    }

    uint32_t num_elements_ = 0;
    CIndex front_;
    CIndex back_;
    char log_queue_[LOGGING_MAX_SIZE] = {};
};

extern Logger logger;

#endif  // UNHUMAN_MOTORLIB_LOGGER_H_
