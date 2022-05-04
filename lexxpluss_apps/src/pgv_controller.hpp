#pragma once

#include <zephyr.h>

namespace lexxhard::pgv_controller {

struct msg {
    uint32_t xp, tag;
    int32_t xps;
    int16_t yps;
    uint16_t ang, cc1, cc2, wrn;
    uint8_t addr, lane, o1, s1, o2, s2;
    struct {
        bool cc2, cc1, wrn, np, err, tag, rp, nl, ll, rl;
    } f;
} __attribute__((aligned(4)));

struct msg_control {
    uint8_t dir_command;
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
extern k_thread thread;
extern k_msgq msgq, msgq_control;

}

// vim: set expandtab shiftwidth=4:
