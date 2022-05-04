#pragma once

#include <zephyr.h>

namespace lexxhard::rosserial {

void init();
void run(void *p1, void *p2, void *p3);
extern k_thread thread;

}

// vim: set expandtab shiftwidth=4:
