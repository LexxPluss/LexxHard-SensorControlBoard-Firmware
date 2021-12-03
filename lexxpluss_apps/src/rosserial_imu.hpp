#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "lexxauto_msgs/Imu.h"
#include "imu_controller.hpp"

class ros_imu {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
    }
    void poll() {
        msg_imu2ros message;
        while (k_msgq_get(&msgq_imu2ros, &message, K_NO_WAIT) == 0) {
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
            pub.publish(&msg);
        }
    }
private:
    lexxauto_msgs::Imu msg;
    ros::Publisher pub{"/sensor_set/imu", &msg};
};

// vim: set expandtab shiftwidth=4:
