#pragma once

#include <zephyr.h>

namespace lexxfirm::misc_controller {

void init();
void run(void *p1, void *p2, void *p3);
float get_main_board_temp();
float get_actuator_board_temp(int index = 0);
extern k_thread thread;

}

// vim: set expandtab shiftwidth=4:
