/*
 * Copyright (c) 2024, LexxPluss Inc.
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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <algorithm>
#include "can_dummy_v7.hpp"

namespace {

LOG_MODULE_REGISTER(can2_test);

void send_can_data(const device *dev, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > CAN_MAX_DLEN)
        return;
    can_frame frame{
        .id{id},
        .dlc{dlc}
    };
    std::copy(data, data + dlc, frame.data);
    can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
    LOG_INF("CAN2 send\n");
}

void send_can2_data(const device *dev)
{
    uint8_t buf[8];
    buf[0] = 0;
    buf[1] = 1;
    buf[2] = 2;
    buf[3] = 3;
    buf[4] = 4;
    buf[5] = 5;
    buf[6] = 6;
    buf[7] = 7;
    send_can_data(dev, 0x204, buf, sizeof buf);
}

void receive_can2_data(const uint8_t *data)
{
    LOG_INF("receive data: %d %d %d %d %d %d %d %d\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

CAN_MSGQ_DEFINE(msgq_can2, 8);

}

namespace lexxhard::can_dummy_v7 {

class can_dummy_v7_impl {
public:
    int init() {
        // dev = device_get_binding("CAN_2");
        dev = device_get_binding("can@40006800");

        if (!device_is_ready(dev)){
            LOG_INF("CAN2 not ready at init\n");
            return -1;
        }

        can_set_bitrate(dev, 1000000);
        can_set_mode(dev, CAN_MODE_NORMAL);
        
        static const can_filter filter{
            .id{0x203},
            .mask{0x7ff},
        };
        can_add_rx_filter_msgq(dev, &msgq_can2, &filter);
        LOG_INF("CAN2 init\n");
        return 0;
    }
    void run(void *p1, void *p2, void *p3) {
        LOG_INF("CAN2 run\n");
        if (!device_is_ready(dev)){
            LOG_INF("CAN2 not ready at run\n");
            return;
        }
        while (true) {
            uint32_t now_cycle{k_cycle_get_32()};
            uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle)};
            if (dt_ms > 1'000) {
                prev_cycle = now_cycle;
                send_can2_data(dev);
                // ++acc_gyro_counter;
            }
            can_frame frame;
            if (k_msgq_get(&msgq_can2, &frame, K_NO_WAIT) == 0) {
                switch (frame.id) {
                case 0x203: receive_can2_data(frame.data); break;
                }
            }
            k_msleep(10);
        }
    }
private:
    const device *dev{nullptr};
    uint32_t prev_cycle{0};
    uint8_t acc_gyro_counter{0};
} impl;

int info(const shell *shell, size_t argc, char **argv)
{
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_can,
    SHELL_CMD(info, NULL, "CAN2 dummy information", info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(can2, &sub_can, "CAN2 dummy commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run(p1, p2, p3);
}

k_thread thread;

}
