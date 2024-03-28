#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "firmware_updater.hpp"

namespace lexxhard::zcan_dfu {

class zcan_dfu {
public:
    // void init(ros::NodeHandle &nh) {
    //     nh.advertise(pub);
    //     nh.subscribe(sub);
    //     response.data = response_data;
    //     response.data_length = sizeof response_data / sizeof response_data[0];
    // }
    // void poll() {
    //     if (k_msgq_get(&firmware_updater::msgq_response, response.data, K_NO_WAIT) == 0)
    //         pub.publish(&response);
    // }
private:
    // void callback(const std_msgs::UInt8MultiArray &packet) {
    //     while (k_msgq_put(&firmware_updater::msgq_data, packet.data, K_NO_WAIT) != 0)
    //         k_msgq_purge(&firmware_updater::msgq_data);
    // }
    // ros::Publisher pub{"/lexxhard/dfu_response", &response};
    // ros::Subscriber<std_msgs::UInt8MultiArray, ros_dfu> sub{"/lexxhard/dfu_data", &ros_dfu::callback, this};
    // std_msgs::UInt16MultiArray response;
    // uint16_t response_data[2];
};

}