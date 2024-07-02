#define LOGGER_TEST
#include <cstdint>
uint64_t get_uptime();
uint64_t get_clock();

#include "../logger.h"
#include <iostream>
#include <charconv>

Logger logger;

uint64_t uptime = 0;
uint64_t clock1 = 0;

uint64_t get_uptime() {
    return uptime++;
}

uint64_t get_clock() {
    return clock1++;
}

void check_num_elements(int expected) {
    if (logger.num_elements() != expected) {
        std::cout << "expected " << expected << " elements, got " << logger.num_elements() << std::endl;
        exit(1);
    }
}

void check_str(std::string expected) {
    std::string actual = logger.get_log();
    std::cout << "actual: " << actual << std::endl;
    std::string_view data = logger.extract_string(actual);
    std::cout << "data: " << data << std::endl;
    if (expected != data) {
        std::cout << "expected " << expected << ", got " << actual << std::endl;
        exit(1);
    }
}

void check_log_end() {
    std::string actual = logger.get_log();
    if (actual != "log end") {
        std::cout << "expected end of log, got " << actual << std::endl;
        exit(1);
    }
}

int main() {
    check_num_elements(0);

    logger.log("hello");
    logger.log("world");
    logger.log("this is a test");

    check_num_elements(3);

    check_str("hello");
    check_str("world");
    check_str("this is a test");
    check_log_end();

    check_num_elements(0);

    for (int i=0; i<500; i++) {
        logger.log(std::to_string(i));
        std::cout << "num_elements: " << logger.num_elements() << std::endl;
    }
    std::string int1 = logger.get_log();
    uint32_t val_start;
    std::string_view data = logger.extract_string(int1);
    std::from_chars(data.data(), data.data() + data.size(), val_start);
    std::cout << "int1: " << int1 << std::endl;
    std::cout << "num_elements: " << logger.num_elements() << std::endl;
    uint32_t num = logger.num_elements();
    for (int i=0; i<num; i++) {
        //std::string int2 = logger.get_log();
        //std::cout << "int2: " << int2 << std::endl;
        check_str(std::to_string(i+val_start+1));
    }
    check_num_elements(0);

    return 0;
}