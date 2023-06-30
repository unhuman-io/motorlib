#include <iostream>
#include "../control_fun.h"

template<typename T>
void print_vector(std::vector<T> v) {
    for (int i=0; i<v.size(); i++) {
        std::cout << v[i] << ", ";
    }
    std::cout << std::endl;
}

void test_median_filter() {
    MedianFilter<> filt;
    assert(filt.update(1) == 0);
    assert(filt.update(2) == 0);
    assert(filt.update(17) == 1);
    assert(filt.update(-10) == 1);
    assert(filt.update(17) == 2);
    assert(filt.update(3) == 3);
    assert(filt.update(22) == 17);
    print_vector(filt.sort());
}

int main() {
    test_median_filter();
    std::cout << "success!" << std::endl;
}