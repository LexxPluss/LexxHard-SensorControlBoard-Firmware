/*
 * Copyright (c) 2022, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <zephyr.h>

namespace lexxhard::actuator_controller {

struct can_format_control {
    int8_t direction[3]; // Left / Center / Right, -1:down, 0:stop, 1:up
    uint8_t power_duty[3]; // Left / Center / Right, 0-100 duty[%]
} __attribute__((aligned(4)));

struct can_format_encoder {
    can_format_encoder(int16_t enc0,int16_t enc1,int16_t enc2) : encoder_count{enc0, enc1, enc2} {} 
    int16_t encoder_count[3]; // Left / Center / Right
} __attribute__((aligned(4)));

struct can_format_current {
    can_format_current(int16_t cur0,int16_t cur1,int16_t cur2, int16_t con) : current_mv{cur0, cur1, cur2}, connection_mv(con) {} 
    int16_t current_mv[3]; // Left / Center / Right
    int16_t connection_mv;
} __attribute__((aligned(4)));

struct msg_control {

    msg_control(can_format_control frame)
    : actuators
    {
        {
            frame.direction[0],
            frame.power_duty[0]
        },
        {
            frame.direction[1],
            frame.power_duty[1]
        },
        {
            frame.direction[2],
            frame.power_duty[2]
        }
    } {}

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
int to_location(const uint8_t (&location)[3], const uint8_t (&power)[3], uint8_t (&detail)[3]);
extern k_thread thread;
extern k_msgq msgq, msgq_control;

}

// vim: set expandtab shiftwidth=4:
