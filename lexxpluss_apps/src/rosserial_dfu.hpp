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
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "firmware_updater.hpp"

namespace lexxhard {

class ros_dfu {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        nh.subscribe(sub);
        response.data = response_data;
        response.data_length = sizeof response_data / sizeof response_data[0];
    }
    void poll() {
        if (k_msgq_get(&firmware_updater::msgq_response, response.data, K_NO_WAIT) == 0)
            pub.publish(&response);
    }
private:
    void callback(const std_msgs::UInt8MultiArray &packet) {
        while (k_msgq_put(&firmware_updater::msgq_data, packet.data, K_NO_WAIT) != 0)
            k_msgq_purge(&firmware_updater::msgq_data);
    }
    ros::Publisher pub{"/lexxhard/dfu_response", &response};
    ros::Subscriber<std_msgs::UInt8MultiArray, ros_dfu> sub{"/lexxhard/dfu_data", &ros_dfu::callback, this};
    std_msgs::UInt16MultiArray response;
    uint16_t response_data[2];
};

}

// vim: set expandtab shiftwidth=4:
