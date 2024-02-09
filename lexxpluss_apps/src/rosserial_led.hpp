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
#include <drivers/can.h>
#include <drivers/gpio.h>
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "lexxauto_msgs/Led.h"
#include "led_controller.hpp"

namespace lexxhard {

char __aligned(4) msgq_led_buffer[8 * sizeof (msg_led)];

struct msg_led {
    uint8_t pattern;
    uint16_t count_per_minutes;
    uint8_t r,g,b;
} __attribute__((aligned(4)));

class can_led { //can_led
public:
    int init()
    {
        //canのinit関係をここに記入すr
        //can device
        k_msgq_init(&msgq_can_led, msgq_led_buffer, sizeof (msg_led), 8);
        dev = device_get_binding("CAN_2");
        if (!device_is_ready(dev))
            return -1;
        can_configure(dev, CAN_NORMAL_MODE, 500000); //最後はボーレート 1000000で良いはず
        return 0;

        //setup can filter
        static const zcan_filter filter_led{
            .id{0x205},  //changed based on saito-san's 
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{0x7c0},
            .rtr_mask{1}
        };
        can_attach_msgq(dev, &msgq_can_led, &filter_led);
    }
    void poll()
    {
        while (true) {
            zcan_frame frame;
            if (k_msgq_get(&msgq_can_led, &frame, K_NO_WAIT) == 0) {
                while (k_msgq_put(&msgq_can_led, &bmu2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_bmu);
            }
            else
            {
                k_msleep(1);
            }
        }
    }
private:
    void callback_string(const std_msgs::String &req) {
        //
        led_controller::msg message{req.data};
        while (k_msgq_put(&led_controller::msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&led_controller::msgq);
    }
    void callback_direct(const lexxauto_msgs::Led &req) {
        led_controller::msg message;
        message.pattern = led_controller::msg::RGB;
        message.interrupt_ms = 0;
        message.rgb[0] = req.r;
        message.rgb[1] = req.g;
        message.rgb[2] = req.b;
        while (k_msgq_put(&led_controller::msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&led_controller::msgq);
    }
    // ros::Subscriber<std_msgs::String, ros_led> sub_string{"/body_control/led", &ros_led::callback_string, this};
    // ros::Subscriber<lexxauto_msgs::Led, ros_led> sub_direct{"/body_control/led_direct", &ros_led::callback_direct, this};
    const device *dev{nullptr};


};

k_thread thread;
k_msgq msgq_can_led;
}

// vim: set expandtab shiftwidth=4:
