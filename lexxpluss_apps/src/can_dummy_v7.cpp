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
#include "can_dummy_v7.hpp"

namespace {

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
    memcpy(frame.data, data, dlc);
    can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
}

void send_uss(const device *dev)
{
    uint32_t data[5]{1000, 1001, 1002, 1003, 1004};
    uint64_t work{0};
    for (const auto i: data) {
        work <<= 12;
        work |= i & 0xfff;
    }
    work <<= 4;
    uint8_t buf[8];
    buf[0] = work >> 56;
    buf[1] = work >> 48;
    buf[2] = work >> 40;
    buf[3] = work >> 32;
    buf[4] = work >> 24;
    buf[5] = work >> 16;
    buf[6] = work >>  8;
    buf[7] = work;
    send_can_data(dev, 0x204, buf, sizeof buf);
}

void send_acc(const device *dev, uint8_t counter)
{
    uint32_t data[3]{2000, 2001, 2002};
    uint8_t buf[7];
    buf[0] = data[0] >> 8;
    buf[1] = data[0];
    buf[2] = data[1] >> 8;
    buf[3] = data[1];
    buf[4] = data[2] >> 8;
    buf[5] = data[2];
    buf[6] = counter;
    send_can_data(dev, 0x206, buf, sizeof buf);
}

void send_gyro(const device *dev, uint8_t counter)
{
    uint32_t data[3]{3000, 3001, 3002};
    uint8_t buf[7];
    buf[0] = data[0] >> 8;
    buf[1] = data[0];
    buf[2] = data[1] >> 8;
    buf[3] = data[1];
    buf[4] = data[2] >> 8;
    buf[5] = data[2];
    buf[6] = counter;
    send_can_data(dev, 0x207, buf, sizeof buf);
}

void send_encoder(const device *dev)
{
    uint32_t data[3]{4000, 4001, 4002};
    uint8_t buf[6];
    buf[0] = data[0] >> 8;
    buf[1] = data[0];
    buf[2] = data[1] >> 8;
    buf[3] = data[1];
    buf[4] = data[2] >> 8;
    buf[5] = data[2];
    send_can_data(dev, 0x209, buf, sizeof buf);
}

void send_current(const device *dev)
{
    uint32_t data[4]{5000, 5001, 5002, 2000};
    uint8_t buf[8];
    buf[0] = data[0] >> 8;
    buf[1] = data[0];
    buf[2] = data[1] >> 8;
    buf[3] = data[1];
    buf[4] = data[2] >> 8;
    buf[5] = data[2];
    buf[6] = data[3] >> 8;
    buf[7] = data[3];
    send_can_data(dev, 0x20a, buf, sizeof buf);
}

}

namespace lexxhard::can_dummy_v7 {

class can_dummy_v7_impl {
public:
    int init() {
        dev = device_get_binding("CAN_2");
        if (!device_is_ready(dev))
            return -1;
        can_configure(dev, CAN_NORMAL_MODE, 1'000'000);
        return 0;
    }
    void run(void *p1, void *p2, void *p3) {
        if (!device_is_ready(dev))
            return;
        while (true) {
            uint32_t now_cycle{k_cycle_get_32()};
            uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle)};
            if (dt_ms > 1'000) {
                prev_cycle = now_cycle;
                send_uss(dev);
                send_acc(dev, acc_gyro_counter);
                send_gyro(dev, acc_gyro_counter);
                send_encoder(dev);
                send_current(dev);
                ++acc_gyro_counter;
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
    SHELL_CMD(info, NULL, "CAN dummy information", info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(can, &sub_can, "CAN dummy commands", NULL);

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
