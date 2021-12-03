#pragma once

#include <zephyr.h>

struct msg_uss2ros {
    uint32_t front_left, front_right;
    uint32_t left, right, back;
} __attribute__((aligned(4)));

struct uss_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static k_thread thread;
};

extern k_msgq msgq_uss2ros;

// vim: set expandtab shiftwidth=4:
