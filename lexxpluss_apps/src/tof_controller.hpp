#pragma once

#include <zephyr.h>

namespace lexxhard::tof_controller {

struct msg {
    int32_t left, right;
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
extern k_thread thread;
extern k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
