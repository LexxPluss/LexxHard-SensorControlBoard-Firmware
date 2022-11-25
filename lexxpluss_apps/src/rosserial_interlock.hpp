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
#include "std_msgs/Bool.h"
#include "interlock_controller.hpp"

namespace lexxhard {

class ros_interlock {
public:
    void init(ros::NodeHandle &nh) {
        nh.subscribe(sub_done_operation);
        nh.advertise(pub_enable_operation);
        msg_enable_operation.data = false;
    }
    void poll() {
        interlock_controller::msg_enable message;
        while (k_msgq_get(&interlock_controller::msgq_enable_amr, &message, K_NO_WAIT) == 0) {
            msg_enable_operation.data = message.enable;
            pub_enable_operation.publish(&msg_enable_operation);
        }
    }
private:
    void callback_done(const std_msgs::Bool &msg) {
        interlock_controller::msg_done message{msg.data};
	while (k_msgq_put(&interlock_controller::msgq_done_amr, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&interlock_controller::msgq_done_amr);
    }
    std_msgs::Bool msg_enable_operation;
    ros::Publisher pub_enable_operation{"/control/enable_operation", &msg_enable_operation};
    ros::Subscriber<std_msgs::Bool, ros_interlock> sub_done_operation{"/control/done_operation", &ros_interlock::callback_done, this};
};  // class ros_interlock

}  // namespace lexxhard

// vim: set expandtab shiftwidth=4:
