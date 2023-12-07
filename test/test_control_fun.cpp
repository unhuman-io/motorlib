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

bool close_to(float a, float b, float tol = FLT_EPSILON*3) {
    return std::fabs(a-b) < tol;
}

void test_output_rollover() {
    for (float output_bias = -50; output_bias < 50; output_bias += .1) {
        for (float rollover = -2*M_PI; rollover < 2*M_PI; rollover += .1) {
            for (float position = 0; position < 2*M_PI; position += .1) {
                float answer = fun(position, output_bias, rollover);
                // answer should be equivalent to position+bias ignoring differences in 2*pi wraps
                assert(close_to(std::fmod(answer, 2*M_PI), std::fmod(position + output_bias), 2*M_PI));
                // answer should be in the range dictated by rollover
                if (rollover >= 0) {
                    assert(answer <= rollover);
                    assert(rollover - 2*M_PI <= answer);
                } else {
                    assert(rollover <= answer);
                    assert(answer <= rollover + 2*M_PI);
                }
                
            }
        }
    }
}

int main() {
    test_median_filter();
    std::cout << "success!" << std::endl;
}