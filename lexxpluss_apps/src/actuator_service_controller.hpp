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

#include <zephyr/kernel.h>

namespace lexxhard::actuator_service_controller {

enum class service_mode: uint8_t {
    LOCATION = 0,
    INIT = 1,
};

struct msg_request {
    service_mode mode;
    struct {
        int8_t location;
        uint8_t power;
    } left, center, right;
    uint8_t counter;

    static msg_request from(uint8_t data[8]) {
        return {
            .mode = static_cast<service_mode>(data[0]),
            .left = {static_cast<int8_t>(data[1]), data[4]},
            .center = {static_cast<int8_t>(data[2]), data[5]},
            .right = {static_cast<int8_t>(data[3]), data[6]},
            .counter = data[7]
        };
    }
} __attribute__((aligned(4)));

struct msg_response {
    service_mode mode;
    bool success;
    struct {
        uint8_t detail;
    } left, center, right;
    uint8_t counter;

    void into(uint8_t data[8]) {
        data[0] = static_cast<int8_t>(mode);
        data[1] = static_cast<uint8_t>(success);
        data[2] = left.detail;
        data[3] = center.detail;
        data[4] = right.detail;
        data[5] = 0;
        data[6] = 0;
        data[7] = counter;
    }
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
extern k_thread thread;
extern k_msgq msgq_request, msgq_response;
}
