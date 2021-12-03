#pragma once

#include <zephyr.h>
#include <cstdio>
#include "ros/node_handle.h"
#include "std_msgs/UInt8.h"
#include "lexxauto_msgs/PositionGuideVision.h"
#include "pgv_controller.hpp"

#define M_PI 3.14159265358979323846

class ros_pgv {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        nh.subscribe(sub);
    }
    void poll() {
        msg_pgv2ros message;
        while (k_msgq_get(&msgq_pgv2ros, &message, K_NO_WAIT) == 0)
            publish(message);
    }
private:
    void publish(const msg_pgv2ros &message) {
        float ang{static_cast<float>(message.ang) * 1.0f};
        if (ang < 180.0f)
            ang *= -1.0f;
        else
            ang = 360.0f - ang;
        float xpos{static_cast<float>(message.xps)};
        if (!message.f.tag)
            xpos = message.xp;
        float ypos{static_cast<float>(message.yps)};
        msg.angle = ang * M_PI / 180.0f;
        msg.x_pos = xpos * 1e-3f;
        msg.y_pos = ypos * 1e-3f;
        msg.direction = direction;
        msg.color_lane_count = message.lane;
        msg.no_color_lane = message.f.nl;
        msg.no_pos = message.f.np;
        msg.tag_detected = message.f.tag;
        msg.control_code1_detected = message.f.cc1;
        msg.control_code2_detected = message.f.cc2;
        pub.publish(&msg);
    }
    void callback(const std_msgs::UInt8 &req) {
        switch (req.data) {
        case 0:
            snprintf(direction, sizeof direction, "No lane is selected");
            break;
        case 1:
            snprintf(direction, sizeof direction, "Right lane is selected");
            break;
        case 2:
            snprintf(direction, sizeof direction, "Left lane is selected");
            break;
        case 3:
            snprintf(direction, sizeof direction, "Straight Ahead");
            break;
        }
        msg_ros2pgv ros2pgv;
        ros2pgv.dir_command = req.data;
        while (k_msgq_put(&msgq_ros2pgv, &ros2pgv, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_ros2pgv);
    }
    lexxauto_msgs::PositionGuideVision msg;
    ros::Publisher pub{"/sensor_set/pgv", &msg};
    ros::Subscriber<std_msgs::UInt8, ros_pgv> sub{"/sensor_set/pgv_dir", &ros_pgv::callback, this};
    char direction[64]{"Straight Ahead"};
};

// vim: set expandtab shiftwidth=4:
