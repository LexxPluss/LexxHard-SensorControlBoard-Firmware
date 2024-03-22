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

#include <algorithm>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include "can_test_v7.hpp"
#include "firmware_updater.hpp"

namespace {

CAN_MSGQ_DEFINE(msgq, 16);

}

namespace lexxhard::can_test_v7 {

class can_test_v7_impl {
public:
    int init() {
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev))
            return -1;
        can_set_bitrate(dev, 1'000'000);
        can_set_mode(dev, CAN_MODE_NORMAL);
        can_start(dev);
        const can_filter filter{
            .id{0x200},
            .mask{0x7f0}
        };
        can_add_rx_filter_msgq(dev, &msgq, &filter);
        return 0;
    }
    void run() const {
        if (!device_is_ready(dev))
            return;
        const gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(led2), gpios);
        gpio_is_ready_dt(&led) && gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
        while (true) {
            auto handled{false};
            if (can_frame frame; k_msgq_get(&msgq, &frame, K_NO_WAIT) == 0) {
                if (frame.id == 0x20d && frame.dlc == sizeof (firmware_updater::command_packet)) {
                    while (k_msgq_put(&firmware_updater::msgq_command, frame.data, K_NO_WAIT) != 0)
                        k_msgq_purge(&firmware_updater::msgq_command);
                }
                handled = true;
            }
            if (firmware_updater::response_packet response;
                k_msgq_get(&firmware_updater::msgq_response, &response, K_NO_WAIT) == 0) {
                can_frame frame{
                    .id{0x20e},
                    .dlc{sizeof response}
                };
                std::copy_n(reinterpret_cast<uint8_t*>(&response), sizeof response, frame.data);
                can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
                handled = true;
            }
            if (!handled)
                k_msleep(1);
            else
                gpio_is_ready_dt(&led) && gpio_pin_toggle_dt(&led);
        }
    }
private:
    const device *dev{nullptr};
} impl;

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread thread;

}
