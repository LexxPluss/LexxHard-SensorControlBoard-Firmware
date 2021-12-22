#pragma once

#include <zephyr.h>

namespace lexxfirm::adc_reader {

void init();
void run(void *p1, void *p2, void *p3);
int32_t get(int index);
extern k_thread thread;

enum {
    DOWNWARD_R = 0,
    DOWNWARD_L,
    ACTUATOR_0,
    ACTUATOR_1,
    ACTUATOR_2,
    TROLLEY,
    NUM_CHANNELS
};

}

// vim: set expandtab shiftwidth=4:
