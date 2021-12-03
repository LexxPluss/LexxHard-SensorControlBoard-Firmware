#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/Float64MultiArray.h"
#include "tof_controller.hpp"

class ros_tof {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        msg.data = msg_data;
        msg.data_length = sizeof msg_data / sizeof msg_data[0];
    }
    void poll() {
        msg_tof2ros message;
        while (k_msgq_get(&msgq_tof2ros, &message, K_NO_WAIT) == 0) {
            static constexpr float meter_per_volt{0.7575f};
            msg.data[0] = message.left * 1e-3f * meter_per_volt;
            msg.data[1] = message.right * 1e-3f * meter_per_volt;
            pub.publish(&msg);
        }
    }
private:
    std_msgs::Float64MultiArray msg;
    float msg_data[2];
    ros::Publisher pub{"/sensor_set/downward", &msg};
};

// vim: set expandtab shiftwidth=4:
