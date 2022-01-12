#pragma once

#include <zephyr.h>

namespace lexxfirm::imu_controller {

struct msg {
    float accel[3];
    float gyro[3];
    float delta_ang[3];
    float delta_vel[3];
    float temp;
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
extern k_thread thread;
extern k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:

