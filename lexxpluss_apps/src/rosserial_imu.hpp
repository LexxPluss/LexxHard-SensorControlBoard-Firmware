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
#include "lexxauto_msgs/Imu.h"
#include "imu_controller.hpp"

namespace lexxhard {

class ros_imu {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        nh.advertise(pub_min_max);
    }
    void poll() {
        imu_controller::msg message;
        while (k_msgq_get(&imu_controller::msgq, &message, K_NO_WAIT) == 0) {
            //NORMAL Message
            msg.gyro.x = message.gyro[0];
            msg.gyro.y = message.gyro[1];
            msg.gyro.z = message.gyro[2];
            msg.accel.x = message.accel[0];
            msg.accel.y = message.accel[1];
            msg.accel.z = message.accel[2];
            msg.ang.x = message.delta_ang[0];
            msg.ang.y = message.delta_ang[1];
            msg.ang.z = message.delta_ang[2];
            msg.vel.x = message.delta_vel[0];
            msg.vel.y = message.delta_vel[1];
            msg.vel.z = message.delta_vel[2];
            //ACCEL MIN/MAX
            msg_min_max.gyro.x = message.accel_max[0];
            msg_min_max.gyro.y = message.accel_max[1];
            msg_min_max.gyro.z = message.accel_max[2];
            msg_min_max.accel.x = message.accel_min[0];
            msg_min_max.accel.y = message.accel_min[1];
            msg_min_max.accel.z = message.accel_min[2];
            msg_min_max.ang.x = 0;
            msg_min_max.ang.y = 0;
            msg_min_max.ang.z = 0;
            msg_min_max.vel.x = 0;
            msg_min_max.vel.y = 0;
            msg_min_max.vel.z = 0;
            pub.publish(&msg);
            pub.publish(&msg_min_max);
        }
    }
private:
    lexxauto_msgs::Imu msg, msg_min_max;
    ros::Publisher pub{"/sensor_set/imu", &msg};
    ros::Publisher pub_min_max{"/sensor_set/imu_min_max", &msg_min_max};
};

}

// vim: set expandtab shiftwidth=4:
