/*
 * Copyright (c) 2022-2023, LexxPluss Inc.
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
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "lexxauto_msgs/Led.h"
#include "led_controller.hpp"

namespace lexxhard {

class ros_led {
public:
    void init(ros::NodeHandle &nh) {
        nh.subscribe(sub_string);
        nh.subscribe(sub_direct);
    }
    void poll() {}
private:
    void callback_string(const std_msgs::String &req) {
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
    ros::Subscriber<std_msgs::String, ros_led> sub_string{"/body_control/led", &ros_led::callback_string, this};
    ros::Subscriber<lexxauto_msgs::Led, ros_led> sub_direct{"/body_control/led_direct", &ros_led::callback_direct, this};
};

}

// vim: set expandtab shiftwidth=4:
