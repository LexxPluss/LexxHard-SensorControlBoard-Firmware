#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "lexxauto_msgs/LinearActuatorControlArray.h"
#include "actuator_controller.hpp"

class ros_actuator {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub_encoder);
        nh.advertise(pub_connection);
        nh.subscribe(sub_control);
        msg_encoder.data = msg_encoder_data;
        msg_encoder.data_length = sizeof msg_encoder_data / sizeof msg_encoder_data[0];
        msg_connection.data = msg_connection_data;
        msg_connection.data_length = sizeof msg_connection_data / sizeof msg_connection_data[0];
    }
    void poll() {
        msg_actuator2ros message;
        while (k_msgq_get(&msgq_actuator2ros, &message, K_NO_WAIT) == 0) {
            for (int i{0}; i < 3; ++i)
                msg_encoder.data[i] = message.encoder_count[i];
            pub_encoder.publish(&msg_encoder);
            msg_connection.data[0] = message.connect * 1e-3f;
            pub_connection.publish(&msg_connection);
        }
    }
private:
    void callback_control(const lexxauto_msgs::LinearActuatorControlArray &req) {
        msg_ros2actuator message;
        for (int i{0}; i < 3; ++i) {
            message.actuators[i].direction = req.actuators[i].direction;
            message.actuators[i].power = req.actuators[i].power;
        }
        while (k_msgq_put(&msgq_ros2actuator, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_ros2actuator);
    }
    std_msgs::Int32MultiArray msg_encoder;
    std_msgs::Float32MultiArray msg_connection;
    int32_t msg_encoder_data[3];
    float msg_connection_data[1];
    ros::Publisher pub_encoder{"/body_control/encoder_count", &msg_encoder};
    ros::Publisher pub_connection{"/body_control/shelf_connection", &msg_connection};
    ros::Subscriber<lexxauto_msgs::LinearActuatorControlArray, ros_actuator>
        sub_control{"/body_control/linear_actuator", &ros_actuator::callback_control, this};
};

// vim: set expandtab shiftwidth=4:
