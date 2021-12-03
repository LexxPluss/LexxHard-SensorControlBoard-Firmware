#pragma once

#include <zephyr.h>

struct msg_log {
    char message[64];
} __attribute__((aligned(4)));

struct log_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static k_thread thread;
};

extern k_msgq msgq_log;

// vim: set expandtab shiftwidth=4:
