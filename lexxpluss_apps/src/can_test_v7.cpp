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

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include "can_test_v7.hpp"

namespace {

CAN_MSGQ_DEFINE(msgq, 4);

}

namespace lexxhard {

int can_test_v7::init()
{
    can1 = DEVICE_DT_GET(DT_NODELABEL(can1));
    can2 = DEVICE_DT_GET(DT_NODELABEL(can2));
    if (!device_is_ready(can1) || !device_is_ready(can2))
        return -1;
    can_set_bitrate(can1, 1000000);
    can_set_bitrate(can2, 1000000);
    can_set_mode(can1, CAN_MODE_NORMAL);
    can_set_mode(can2, CAN_MODE_NORMAL);
    can_start(can1);
    can_start(can2);
    const can_filter filter{
        .id{0x100},
        .mask{0x7ff}
    };
    can_add_rx_filter_msgq(can2, &msgq, &filter);
    prev = k_cycle_get_32();
    led_s = GPIO_DT_SPEC_GET(DT_NODELABEL(led2), gpios);
    led_r = GPIO_DT_SPEC_GET(DT_NODELABEL(led3), gpios);
    gpio_is_ready_dt(&led_s) && gpio_pin_configure_dt(&led_s, GPIO_OUTPUT_INACTIVE);
    gpio_is_ready_dt(&led_r) && gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
    return 0;
}

int can_test_v7::poll()
{
    if (!device_is_ready(can1) || !device_is_ready(can2))
        return -1;
    if (can_frame frame; k_msgq_get(&msgq, &frame, K_NO_WAIT) == 0) {
        printk("Received frame: %08x\n", frame.id);
        gpio_is_ready_dt(&led_r) && gpio_pin_toggle_dt(&led_r);
    }
    if (auto now{k_cycle_get_32()}; k_cyc_to_ms_near32(now - prev) > 1'000) {
        prev = now;
        can_frame frame{
            .id{0x100},
            .dlc{8},
            .data{0, 1, 2, 3, 4, 5, 6, 7}
        };
        can_send(can1, &frame, K_MSEC(100), nullptr, nullptr);
        printk("Sent frame: %08x\n", frame.id);
        gpio_is_ready_dt(&led_s) && gpio_pin_toggle_dt(&led_s);
    }
    return 0;
}

}
