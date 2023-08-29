/*
 * Copyright (c) 2023, LexxPluss Inc.
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
#include "std_msgs/Bool.h"
#include "towing_unit.hpp"

namespace lexxhard {

class ros_towing_unit {
public:
    void init(ros::NodeHandle &nh) {
        nh.subscribe(sub_towing_unit_power_on);
        nh.advertise(pub_towing_unit_status);
        message_towing_unit_status.left_sw = false;
        message_towing_unit_status.right_sw = false;
        message_towing_unit_status.power_good = false;
        message_towing_unit_status.power_on = true;
    }
    void poll() {
        //ユニットの値をPUB
        while (k_msgq_get(&towing_unit_controller::msg_towing_unit_status, &message, K_NO_WAIT) == 0) {
            message_towing_unit_status.left_sw = message.left_sw;
            message_towing_unit_status.right_sw = message.right_sw;
            message_towing_unit_status.power_good = message.power_good;
            message_towing_unit_status.power_on = message.power_on;
            pub_towing_unit_status.publish(&message_towing_unit_status);
        }
    }
private:
    void callback_towing_unit_power_on(const std_msgs::Bool &msg) {
        message_towing_unit_status.power_on = msg.data;
        //SUBで値を受け取ってPUB
        while (k_msgq_put(&towing_unit::msgq_towing_unit_status, &message_towing_unit_status, K_NO_WAIT) != 0){
            k_msgq_purge(&towing_unit::msgq_towing_unit_status);
        }
    }
    lexxhard::towing_unit_controller::msg_towing_unit_status message_towing_unit_status;
    ros::Publisher pub_towing_unit_status{"/control/towing_unit_loading_status", &msg_towing_unit_status};
    ros::Subscriber<std_msgs::Bool, ros_towing_unit> sub_towing_unit_power_on{"/control/towing_unit_power_on", &ros_towing_unit::callback_towing_unit_power_on, this};
};  // class ros_towing_unit

}  // namespace lexxhard

// vim: set expandtab shiftwidth=4:
