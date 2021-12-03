#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "lexxauto_msgs/InitLinearActuator.h"
#include "lexxauto_msgs/LinearActuatorLocation.h"
#include "actuator_controller.hpp"

class ros_actuator_service {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertiseService(service_location);
        nh.advertiseService(service_init);
    }
private:
    void callback_location(const lexxauto_msgs::LinearActuatorLocationRequest &req, lexxauto_msgs::LinearActuatorLocationResponse &res) {
        res.success = actuator_controller::to_location(req.location.data, req.power.data, res_detail_data) == 0;
        res.detail.data = res_detail_data;
        res.detail.data_length = sizeof res_detail_data / sizeof res_detail_data[0];
    }
    void callback_init(const lexxauto_msgs::InitLinearActuatorRequest &req, lexxauto_msgs::InitLinearActuatorResponse &res) {
        res.success = actuator_controller::init_location() == 0;
    }
    uint8_t res_detail_data[3];
    ros::ServiceServer<lexxauto_msgs::LinearActuatorLocationRequest, lexxauto_msgs::LinearActuatorLocationResponse, ros_actuator_service>
        service_location{"/body_control/linear_actuator_location", &ros_actuator_service::callback_location, this};
    ros::ServiceServer<lexxauto_msgs::InitLinearActuatorRequest, lexxauto_msgs::InitLinearActuatorResponse, ros_actuator_service>
        service_init{"/body_control/init_linear_actuator", &ros_actuator_service::callback_init, this};
};

// vim: set expandtab shiftwidth=4:
