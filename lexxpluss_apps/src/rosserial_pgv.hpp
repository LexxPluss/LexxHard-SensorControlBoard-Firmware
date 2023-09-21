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
#include <cstdio>
#include "ros/node_handle.h"
#include "std_msgs/UInt8.h"
#include "lexxauto_msgs/PositionGuideVision.h"
#include "common.hpp"
#include "pgv_controller.hpp"

namespace lexxhard {

class ros_pgv {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        nh.subscribe(sub);
    }
    void poll() {
        pgv_controller::msg message;
        while (k_msgq_get(&pgv_controller::msgq, &message, K_NO_WAIT) == 0)
            publish(message);
    }
private:
    void publish(const pgv_controller::msg &message) {
        float ang{static_cast<float>(message.ang) * 0.1f};
        if (ang < 180.0f)
            ang *= -1.0f;
        else
            ang = 360.0f - ang;
        float xpos{static_cast<float>(message.xps)};
        if (!message.f.tag)
            xpos = message.xp;
        float ypos{static_cast<float>(message.yps)};
        msg.angle = ang * M_PI / 180.0f;
        msg.x_pos = xpos * 1e-4f;
        msg.y_pos = ypos * 1e-4f;
        msg.direction = direction;
        msg.color_lane_count = message.lane;
        msg.no_color_lane = message.f.nl;
        msg.no_pos = message.f.np;
        msg.tag_detected = message.f.tag;
        msg.control_code1_detected = message.f.cc1;
        msg.control_code2_detected = message.f.cc2;
        pub.publish(&msg);
    }
    void callback(const std_msgs::UInt8 &req) {
        switch (req.data) {
        case 0:
            snprintf(direction, sizeof direction, "No lane is selected");
            break;
        case 1:
            snprintf(direction, sizeof direction, "Right lane is selected");
            break;
        case 2:
            snprintf(direction, sizeof direction, "Left lane is selected");
            break;
        case 3:
            snprintf(direction, sizeof direction, "Straight Ahead");
            break;
        }
        pgv_controller::msg_control ros2pgv;
        ros2pgv.dir_command = req.data;
        while (k_msgq_put(&pgv_controller::msgq_control, &ros2pgv, K_NO_WAIT) != 0)
            k_msgq_purge(&pgv_controller::msgq_control);
    }
    lexxauto_msgs::PositionGuideVision msg;
    ros::Publisher pub{"/sensor_set/pgv", &msg};
    ros::Subscriber<std_msgs::UInt8, ros_pgv> sub{"/sensor_set/pgv_dir", &ros_pgv::callback, this};
    char direction[64]{"Straight Ahead"};
};

}

// vim: set expandtab shiftwidth=4:
