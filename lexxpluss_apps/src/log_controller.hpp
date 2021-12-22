#pragma once

#include <zephyr.h>

struct msg_log {
    char message[64];
} __attribute__((aligned(4)));

extern k_msgq msgq_log;

struct log_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static void output(const char *fmt, ...) {
        msg_log message;
        va_list arg;
        va_start(arg, fmt);
        vsnprintk(message.message, sizeof message.message, fmt, arg);
        va_end(arg);
        while (k_msgq_put(&msgq_log, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_log);
    }
    static k_thread thread;
};

#define sdlog(fmt, ...) log_controller::output(__VA_ARGS__)

// vim: set expandtab shiftwidth=4:
