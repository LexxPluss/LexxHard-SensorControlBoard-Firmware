#pragma once

#include <zephyr.h>

struct msg_imu2ros {
    float accel[3];
    float gyro[3];
    float delta_ang[3];
    float delta_vel[3];
    float temp;
} __attribute__((aligned(4)));

struct imu_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static k_thread thread;
};

extern k_msgq msgq_imu2ros;

// vim: set expandtab shiftwidth=4:

