#pragma once

#include <zephyr.h>

struct misc_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static float get_main_board_temp();
    static float get_actuator_board_temp(int index = 0);
    static k_thread thread;
};

// vim: set expandtab shiftwidth=4:
