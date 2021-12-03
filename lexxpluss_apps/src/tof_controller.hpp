#pragma once

#include <zephyr.h>

struct msg_tof2ros {
    int32_t left, right;
} __attribute__((aligned(4)));

struct tof_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static k_thread thread;
};

extern k_msgq msgq_tof2ros;

// vim: set expandtab shiftwidth=4:
