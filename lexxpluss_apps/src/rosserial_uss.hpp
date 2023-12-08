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
#include "ros/node_handle.h"
#include "std_msgs/Float64MultiArray.h"
#include "uss_controller.hpp"

namespace lexxhard {

class ros_uss {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        msg.data = msg_data;
        msg.data_length = sizeof msg_data / sizeof msg_data[0];
    }
    void poll() {
        uss_controller::msg message;
        while (k_msgq_get(&uss_controller::msgq, &message, K_NO_WAIT) == 0) {
            msg.data[0] = message.front_left * 1e-3f;
            msg.data[1] = message.front_right * 1e-3f;
            msg.data[2] = message.left * 1e-3f;
            msg.data[3] = message.right * 1e-3f;
            msg.data[4] = message.back * 1e-3f;
            pub.publish(&msg);
        }
    }
private:
    std_msgs::Float64MultiArray msg;
    float msg_data[5];
    ros::Publisher pub{"/sensor_set/ultrasonic", &msg};
};

}

// vim: set expandtab shiftwidth=4:
