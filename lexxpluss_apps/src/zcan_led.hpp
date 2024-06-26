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
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "led_controller.hpp"

#define CAN_ID_LED 0x205 //based on CAN ID assignment

namespace lexxhard::zcan_led {

LOG_MODULE_REGISTER(zcan_led);

char __aligned(4) msgq_led_buffer[8 * sizeof (struct can_frame)];

CAN_MSGQ_DEFINE(msgq_can_led, 16);

class zcan_led {
public:
    void init() {
        //can device bind
        k_msgq_init(&msgq_can_led, msgq_led_buffer, sizeof (struct can_frame), 8);

        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)){
            LOG_ERR("CAN_2 is not ready");
            return;
        }

        //setup can filter
        static const can_filter filter_led{
            .id{CAN_ID_LED},
            .mask{0x7ff}
        };

        can_add_rx_filter_msgq(dev, &msgq_can_led, &filter_led);
        return;
    }

    void poll()
    {
        struct can_frame can_frame;
        while (k_msgq_get(&msgq_can_led, &can_frame, K_NO_WAIT) == 0) {
            can_led_frame.pattern = can_frame.data[0];
            can_led_frame.count_per_minutes = (can_frame.data[1] << 8) | can_frame.data[2];
            can_led_frame.rgb[0] = can_frame.data[3];
            can_led_frame.rgb[1] = can_frame.data[4];
            can_led_frame.rgb[2] = can_frame.data[5];

            can2led = led_controller::msg(can_led_frame);
            while (k_msgq_put(&led_controller::msgq, &can2led, K_NO_WAIT) != 0)
                k_msgq_purge(&led_controller::msgq);
        }
    }
private:
    led_controller::can_format can_led_frame;
    led_controller::msg can2led;
    const device *dev{nullptr};
};
}

// vim: set expandtab shiftwidth=4:
