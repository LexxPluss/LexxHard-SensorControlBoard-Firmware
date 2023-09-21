/*
 * Copyright (c) 2022-2023, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <zephyr/kernel.h>
#include "ros/node_handle.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Byte.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "lexxauto_msgs/BoardTemperatures.h"
#include "can_controller.hpp"

namespace lexxhard {

class ros_board {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub_fan);
        nh.advertise(pub_bumper);
        nh.advertise(pub_emergency);
        nh.advertise(pub_charge);
        nh.advertise(pub_temperature);
        nh.advertise(pub_power);
        nh.advertise(pub_charge_delay);
        nh.advertise(pub_charge_voltage);
        nh.subscribe(sub_emergency);
        nh.subscribe(sub_poweroff);
        nh.subscribe(sub_lexxhard);
        msg_fan.data = msg_fan_data;
        msg_fan.data_length = sizeof msg_fan_data / sizeof msg_fan_data[0];
        msg_bumper.data = msg_bumper_data;
        msg_bumper.data_length = sizeof msg_bumper_data / sizeof msg_bumper_data[0];
    }
    void poll() {
        can_controller::msg_board message;
        while (k_msgq_get(&can_controller::msgq_board, &message, K_NO_WAIT) == 0) {
            publish_fan(message);
            publish_bumper(message);
            publish_emergency(message);
            publish_charge(message);
            publish_temperature(message);
            publish_power(message);
            publish_charge_delay(message);
            publish_charge_voltage(message);
        }
    }
private:
    void publish_fan(const can_controller::msg_board &message) {
        msg_fan.data[0] = message.fan_duty;
        pub_fan.publish(&msg_fan);
    }
    void publish_bumper(const can_controller::msg_board &message) {
        msg_bumper.data[0] = message.bumper_switch[0];
        msg_bumper.data[1] = message.bumper_switch[1];
        pub_bumper.publish(&msg_bumper);
    }
    void publish_emergency(const can_controller::msg_board &message) {
        msg_emergency.data = message.emergency_switch[0] || message.emergency_switch[1];
        pub_emergency.publish(&msg_emergency);
    }
    void publish_charge(const can_controller::msg_board &message) {
        static constexpr uint8_t MANUAL_CHARGE_STATE{6}, AUTO_CHARGE_STATE{5};
        if (message.state == MANUAL_CHARGE_STATE)
            msg_charge.data = 2;
        else if (message.state == AUTO_CHARGE_STATE)
            msg_charge.data = 1;
        else
            msg_charge.data = 0;
        pub_charge.publish(&msg_charge);
    }
    void publish_temperature(const can_controller::msg_board &message) {
        msg_temperature.main.temperature = message.main_board_temp;
        msg_temperature.power.temperature = message.power_board_temp;
        // ROS:[center,left,right], ROBOT:[left,center,right]
        msg_temperature.linear_actuator_center.temperature = message.actuator_board_temp[1];
        msg_temperature.linear_actuator_left.temperature = message.actuator_board_temp[0];
        msg_temperature.linear_actuator_right.temperature = message.actuator_board_temp[2];
        msg_temperature.charge_plus.temperature = message.charge_connector_temp[0];
        msg_temperature.charge_minus.temperature = message.charge_connector_temp[1];
        pub_temperature.publish(&msg_temperature);
    }
    void publish_power(const can_controller::msg_board &message) {
        msg_power.data = message.wait_shutdown ? message.shutdown_reason : 0;
        pub_power.publish(&msg_power);
    }
    void publish_charge_delay(const can_controller::msg_board &message) {
        msg_charge_delay.data = message.charge_heartbeat_delay;
        pub_charge_delay.publish(&msg_charge_delay);
    }
    void publish_charge_voltage(const can_controller::msg_board &message) {
        msg_charge_voltage.data = message.charge_connector_voltage;
        pub_charge_voltage.publish(&msg_charge_voltage);
    }
    void callback_emergency(const std_msgs::Bool &req) {
        ros2board.emergency_stop = req.data;
        while (k_msgq_put(&can_controller::msgq_control, &ros2board, K_NO_WAIT) != 0)
            k_msgq_purge(&can_controller::msgq_control);
    }
    void callback_poweroff(const std_msgs::Bool &req) {
        ros2board.power_off = req.data;
        while (k_msgq_put(&can_controller::msgq_control, &ros2board, K_NO_WAIT) != 0)
            k_msgq_purge(&can_controller::msgq_control);
    }
    void callback_lexxhard(const std_msgs::String &req) {
        if (strncmp(req.data, "wheel_", 6) == 0)
            ros2board.wheel_power_off = strcmp(req.data, "wheel_poweroff") == 0;
        while (k_msgq_put(&can_controller::msgq_control, &ros2board, K_NO_WAIT) != 0)
            k_msgq_purge(&can_controller::msgq_control);
    }
    std_msgs::UInt8MultiArray msg_fan;
    std_msgs::ByteMultiArray msg_bumper;
    std_msgs::Bool msg_emergency;
    std_msgs::Byte msg_charge, msg_power;
    lexxauto_msgs::BoardTemperatures msg_temperature;
    std_msgs::UInt8 msg_charge_delay;
    std_msgs::Float32 msg_charge_voltage;
    can_controller::msg_control ros2board{0};
    uint8_t msg_fan_data[1];
    int8_t msg_bumper_data[2];
    ros::Publisher pub_fan{"/sensor_set/fan", &msg_fan};
    ros::Publisher pub_bumper{"/sensor_set/bumper", &msg_bumper};
    ros::Publisher pub_emergency{"/sensor_set/emergency_switch", &msg_emergency};
    ros::Publisher pub_charge{"/body_control/charge_status", &msg_charge};
    ros::Publisher pub_temperature{"/sensor_set/temperature", &msg_temperature};
    ros::Publisher pub_power{"/body_control/power_state", &msg_power};
    ros::Publisher pub_charge_delay{"/body_control/charge_heartbeat_delay", &msg_charge_delay};
    ros::Publisher pub_charge_voltage{"/body_control/charge_connector_voltage", &msg_charge_voltage};
    ros::Subscriber<std_msgs::Bool, ros_board> sub_emergency{
        "/control/request_emergency_stop", &ros_board::callback_emergency, this
    };
    ros::Subscriber<std_msgs::Bool, ros_board> sub_poweroff{
        "/control/request_power_off", &ros_board::callback_poweroff, this
    };
    ros::Subscriber<std_msgs::String, ros_board> sub_lexxhard{
        "/lexxhard/setup", &ros_board::callback_lexxhard, this
    };
};

}

// vim: set expandtab shiftwidth=4:
