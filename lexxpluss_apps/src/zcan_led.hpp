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
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/can.h>
#include <logging/log.h>
#include "led_controller.hpp"

#define CAN_ID_LED 0x205 //based on saito-san's CAN ID assignment

namespace lexxhard::zcan_led {

LOG_MODULE_REGISTER(zcan_led);

char __aligned(4) msgq_led_buffer[8 * sizeof (led_controller::msg)];
k_msgq msgq_can_led;

class zcan_led {
public:
    void init()
    {
        //can device bind`
        k_msgq_init(&msgq_can_led, msgq_led_buffer, sizeof (led_controller::msg), 8);
        dev = device_get_binding("CAN_2");
        if (!device_is_ready(dev)){
            LOG_INF("CAN_2 is not ready");
            return;
        }

        //setup can filter
        static const zcan_filter filter_led{
            .id{CAN_ID_LED},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{0x7ff},
            .rtr_mask{1}
        };
        can_attach_msgq(dev, &msgq_can_led, &filter_led);
        return;
    }

    void poll()
    {
        while (k_msgq_get(&msgq_can_led, &can_led_frame, K_NO_WAIT) == 0) {
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
