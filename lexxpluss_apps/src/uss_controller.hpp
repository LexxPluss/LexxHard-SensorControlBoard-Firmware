#pragma once

#include <zephyr.h>

namespace lexxfirm::uss_controller {

struct msg {
    uint32_t front_left, front_right;
    uint32_t left, right, back;
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
extern k_thread thread;
extern k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
