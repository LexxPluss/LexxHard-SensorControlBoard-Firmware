#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "lexxauto_msgs/LinearActuatorControlArray.h"
#include "actuator_controller.hpp"

namespace lexxfirm {

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
        actuator_controller::msg message;
        while (k_msgq_get(&actuator_controller::msgq, &message, K_NO_WAIT) == 0) {
            // ROS:[center,left,right], ROBOT:[left,center,right]
            msg_encoder.data[0] = message.encoder_count[1];
            msg_encoder.data[1] = message.encoder_count[0];
            msg_encoder.data[2] = message.encoder_count[2];
            pub_encoder.publish(&msg_encoder);
            msg_connection.data[0] = message.connect * 1e-3f;
            pub_connection.publish(&msg_connection);
        }
    }
private:
    void callback_control(const lexxauto_msgs::LinearActuatorControlArray &req) {
        actuator_controller::msg_control message;
        // ROS:[center,left,right], ROBOT:[left,center,right]
        message.actuators[0].direction = req.actuators[1].direction;
        message.actuators[1].direction = req.actuators[0].direction;
        message.actuators[2].direction = req.actuators[2].direction;
        message.actuators[0].power = req.actuators[1].power;
        message.actuators[1].power = req.actuators[0].power;
        message.actuators[2].power = req.actuators[2].power;
        while (k_msgq_put(&actuator_controller::msgq_control, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&actuator_controller::msgq_control);
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

}

// vim: set expandtab shiftwidth=4:
