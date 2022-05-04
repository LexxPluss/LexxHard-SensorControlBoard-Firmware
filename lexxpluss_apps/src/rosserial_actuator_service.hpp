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
