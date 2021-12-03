#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/Float64MultiArray.h"
#include "uss_controller.hpp"

class ros_uss {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        msg.data = msg_data;
        msg.data_length = sizeof msg_data / sizeof msg_data[0];
    }
    void poll() {
        msg_uss2ros message;
        while (k_msgq_get(&msgq_uss2ros, &message, K_NO_WAIT) == 0) {
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

// vim: set expandtab shiftwidth=4:
