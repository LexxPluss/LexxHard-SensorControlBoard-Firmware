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

#include <zephyr.h>
#include <device.h>
#include <drivers/can.h>
#include <shell/shell.h>
#include <algorithm>
#include <logging/log.h>
#include "can_dummy_v7_2.hpp"

namespace {

LOG_MODULE_REGISTER(can1_test);

void send_can_data(const device *dev, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > CAN_MAX_DLEN)
        return;
    zcan_frame frame{
        .id{id},
        .rtr{CAN_DATAFRAME},
        .id_type{CAN_STANDARD_IDENTIFIER},
        .dlc{dlc}
    };
    std::copy(data, data + dlc, frame.data);
    can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
    LOG_INF("CAN1 send\n");
}

void send_can1_data(const device *dev)
{
    uint8_t buf[8];
    buf[0] = 7;
    buf[1] = 6;
    buf[2] = 5;
    buf[3] = 4;
    buf[4] = 3;
    buf[5] = 2;
    buf[6] = 1;
    buf[7] = 0;
    send_can_data(dev, 0x203, buf, sizeof buf);
}

void receive_can1_data(const uint8_t *data)
{
    LOG_INF("receive data: %d %d %d %d %d %d %d %d\n",data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

CAN_DEFINE_MSGQ(msgq_can1, 8);

}

namespace lexxhard::can_dummy_v7_2 {

class can_dummy_v7_2_impl {
public:
    int init() {
        dev = device_get_binding("CAN_1");
        if (!device_is_ready(dev))
            return -1;
        can_configure(dev, CAN_NORMAL_MODE, 1'000'000);
        static const zcan_filter filter{
            .id{0x204},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{0x7ff},
            .rtr_mask{1}
        };
        can_attach_msgq(dev, &msgq_can1, &filter);
        LOG_INF("CAN1 init\n");
        return 0;
    }
    void run(void *p1, void *p2, void *p3) {
        LOG_INF("CAN1 run\n");
        if (!device_is_ready(dev)){
            LOG_INF("CAN1 not ready\n");
            return;
        }
        while (true) {
            uint32_t now_cycle{k_cycle_get_32()};
            uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle)};
            if (dt_ms > 1'000) {
                prev_cycle = now_cycle;
                send_can1_data(dev);
                // ++acc_gyro_counter;
            }
            zcan_frame frame;
            if (k_msgq_get(&msgq_can1, &frame, K_NO_WAIT) == 0) {
                switch (frame.id) {
                case 0x204: receive_can1_data(frame.data); break;
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
    SHELL_CMD(info, NULL, "CAN1 dummy2 information", info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(can1, &sub_can, "CAN1 dummy2 commands", NULL);

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
