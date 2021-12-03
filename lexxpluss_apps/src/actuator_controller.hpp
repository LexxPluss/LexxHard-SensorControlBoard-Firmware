#pragma once

#include <zephyr.h>

struct msg_ros2actuator {
    struct {
        int8_t direction;
        uint8_t power;
    } actuators[3];
    static constexpr int8_t DOWN{-1}, STOP{0}, UP{1};
} __attribute__((aligned(4)));

struct msg_actuator2ros {
    int32_t encoder_count[3];
    int32_t current[3];
    int32_t connect;
    bool fail[3];
} __attribute__((aligned(4)));

struct actuator_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static int init_location();
    static int to_location(const uint8_t location[3], const uint8_t power[3], uint8_t detail[3]);
    static k_thread thread;
};

extern k_msgq msgq_actuator2ros;
extern k_msgq msgq_ros2actuator;

// vim: set expandtab shiftwidth=4:
