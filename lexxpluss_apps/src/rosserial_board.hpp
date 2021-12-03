#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Byte.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "lexxauto_msgs/BoardTemperatures.h"
#include "can_controller.hpp"

class ros_board {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub_fan);
        nh.advertise(pub_bumper);
        nh.advertise(pub_emergency);
        nh.advertise(pub_charge);
        nh.advertise(pub_temperature);
        nh.advertise(pub_power);
        nh.subscribe(sub_emergency);
        nh.subscribe(sub_poweroff);
        msg_fan.data = msg_fan_data;
        msg_fan.data_length = sizeof msg_fan_data / sizeof msg_fan_data[0];
        msg_bumper.data = msg_bumper_data;
        msg_bumper.data_length = sizeof msg_bumper_data / sizeof msg_bumper_data[0];
    }
    void poll() {
        msg_board2ros message;
        while (k_msgq_get(&msgq_board2ros, &message, K_NO_WAIT) == 0) {
            publish_fan(message);
            publish_bumper(message);
            publish_emergency(message);
            publish_charge(message);
            publish_temperature(message);
            publish_power(message);
        }
    }
private:
    void publish_fan(const msg_board2ros &message) {
        msg_fan.data[0] = message.fan_duty;
        pub_fan.publish(&msg_fan);
    }
    void publish_bumper(const msg_board2ros &message) {
        msg_bumper.data[0] = message.bumper_switch[0];
        msg_bumper.data[1] = message.bumper_switch[1];
        pub_bumper.publish(&msg_bumper);
    }
    void publish_emergency(const msg_board2ros &message) {
        msg_emergency.data = message.emergency_switch[0] || message.emergency_switch[1];
        pub_emergency.publish(&msg_emergency);
    }
    void publish_charge(const msg_board2ros &message) {
        if (message.manual_charging)
            msg_charge.data = 2;
        else if (message.auto_charging)
            msg_charge.data = 1;
        else
            msg_charge.data = 0;
        pub_charge.publish(&msg_charge);
    }
    void publish_temperature(const msg_board2ros &message) {
        msg_temperature.main.temperature = message.main_board_temp;
        msg_temperature.power.temperature = message.power_board_temp;
        msg_temperature.linear_actuator_center.temperature = message.actuator_board_temp[0];
        msg_temperature.linear_actuator_left.temperature = message.actuator_board_temp[1];
        msg_temperature.linear_actuator_right.temperature = message.actuator_board_temp[2];
        msg_temperature.charge_plus.temperature = message.charge_connector_temp[0];
        msg_temperature.charge_minus.temperature = message.charge_connector_temp[1];
        pub_temperature.publish(&msg_temperature);
    }
    void publish_power(const msg_board2ros &message) {
        msg_power.data = message.wait_shutdown ? 1 : 0;
        pub_power.publish(&msg_power);
    }
    void callback_emergency(const std_msgs::Bool &req) {
        ros2board.emergency_stop = req.data;
        while (k_msgq_put(&msgq_ros2board, &ros2board, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_ros2board);
    }
    void callback_poweroff(const std_msgs::Bool &req) {
        ros2board.power_off = req.data;
        while (k_msgq_put(&msgq_ros2board, &ros2board, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_ros2board);
    }
    std_msgs::UInt8MultiArray msg_fan;
    std_msgs::ByteMultiArray msg_bumper;
    std_msgs::Bool msg_emergency;
    std_msgs::Byte msg_charge, msg_power;
    lexxauto_msgs::BoardTemperatures msg_temperature;
    msg_ros2board ros2board{0};
    uint8_t msg_fan_data[1];
    int8_t msg_bumper_data[2];
    ros::Publisher pub_fan{"/sensor_set/fan", &msg_fan};
    ros::Publisher pub_bumper{"/sensor_set/bumper", &msg_bumper};
    ros::Publisher pub_emergency{"/sensor_set/emergency_switch", &msg_emergency};
    ros::Publisher pub_charge{"/body_control/charge_status", &msg_charge};
    ros::Publisher pub_temperature{"/sensor_set/temperature", &msg_temperature};
    ros::Publisher pub_power{"/body_control/power_state", &msg_power};
    ros::Subscriber<std_msgs::Bool, ros_board> sub_emergency{"/control/request_emergency_stop", &ros_board::callback_emergency, this};
    ros::Subscriber<std_msgs::Bool, ros_board> sub_poweroff{"/control/request_power_off", &ros_board::callback_poweroff, this};
};

// vim: set expandtab shiftwidth=4:
