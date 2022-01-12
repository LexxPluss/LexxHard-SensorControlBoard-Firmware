#pragma once

#include <zephyr.h>

namespace lexxfirm::log_controller {

struct msg {
    char message[64];
} __attribute__((aligned(4)));


void init();
void run(void *p1, void *p2, void *p3);
void output(const char *fmt, ...);
extern k_thread thread;
extern k_msgq msgq;

#define sdlog(fmt, ...) log_controller::output(__VA_ARGS__)

}

// vim: set expandtab shiftwidth=4:
