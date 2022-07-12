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
#include "lexxauto_msgs/InitLinearActuator.h"
#include "lexxauto_msgs/LinearActuatorLocation.h"
#include "actuator_controller.hpp"

namespace lexxhard {

class ros_actuator_service {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertiseService(service_location);
        nh.advertiseService(service_init);
    }
private:
    void callback_location(const lexxauto_msgs::LinearActuatorLocationRequest &req, lexxauto_msgs::LinearActuatorLocationResponse &res) {
        // ROS:[center,left,right], ROBOT:[left,center,right]
        uint8_t location[3]{
            req.location.data[1],
            req.location.data[0],
            req.location.data[2]
        };
        uint8_t power[3]{
            req.power.data[1],
            req.power.data[0],
            req.power.data[2]
        };
        uint8_t detail[3];
        res.success = actuator_controller::to_location(location, power, detail) == 0;
        res.detail.data[0] = detail[1];
        res.detail.data[1] = detail[0];
        res.detail.data[2] = detail[2];
        res.detail.data_length = sizeof detail / sizeof detail[0];
    }
    void callback_init(const lexxauto_msgs::InitLinearActuatorRequest &req, lexxauto_msgs::InitLinearActuatorResponse &res) {
        res.success = actuator_controller::init_location() == 0;
    }
    ros::ServiceServer<lexxauto_msgs::LinearActuatorLocationRequest, lexxauto_msgs::LinearActuatorLocationResponse, ros_actuator_service>
        service_location{"/body_control/linear_actuator_location", &ros_actuator_service::callback_location, this};
    ros::ServiceServer<lexxauto_msgs::InitLinearActuatorRequest, lexxauto_msgs::InitLinearActuatorResponse, ros_actuator_service>
        service_init{"/body_control/init_linear_actuator", &ros_actuator_service::callback_init, this};
};

}

// vim: set expandtab shiftwidth=4:
