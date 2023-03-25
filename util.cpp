#include "util.h"

void ms_delay(uint16_t ms) {
    uint32_t t_start = get_clock();
    while((get_clock() - t_start) < ms*CPU_FREQUENCY_HZ/1000);
}

void us_delay(uint16_t us) {
    uint32_t t_start = get_clock();
    while((get_clock() - t_start) < us*CPU_FREQUENCY_HZ/1000000);
}

void ns_delay(uint16_t ns) {
    uint32_t t_start = get_clock();
    while((get_clock() - t_start) < ns/(uint16_t) (1e9/CPU_FREQUENCY_HZ));
}
