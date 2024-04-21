#include "../logger.h"
#include <iostream>

Logger logger;

void check_num_elements(int expected) {
    if (logger.num_elements() != expected) {
        std::cout << "expected " << expected << " elements, got " << logger.num_elements() << std::endl;
        exit(1);
    }
}

void check_str(std::string expected) {
    std::string actual = logger.get_log();
    std::cout << "actual: " << actual << std::endl;
    std::string data = actual.substr(actual.find(") ") + 2);
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
        logger.log("hello");
        std::cout << "num_elements: " << logger.num_elements() << std::endl;
    }
    return 0;
}