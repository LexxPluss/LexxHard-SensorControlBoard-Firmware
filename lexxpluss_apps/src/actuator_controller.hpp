#pragma once

#include <zephyr.h>

namespace lexxfirm::actuator_controller {

struct msg_control {
    struct {
        int8_t direction;
        uint8_t power;
    } actuators[3];
    static constexpr int8_t DOWN{-1}, STOP{0}, UP{1};
} __attribute__((aligned(4)));

struct msg {
    int32_t encoder_count[3];
    int32_t current[3];
    int32_t connect;
    bool fail[3];
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
int init_location();
int to_location(const uint8_t location[3], const uint8_t power[3], uint8_t detail[3]);
extern k_thread thread;
extern k_msgq msgq, msgq_control;

}

// vim: set expandtab shiftwidth=4:
