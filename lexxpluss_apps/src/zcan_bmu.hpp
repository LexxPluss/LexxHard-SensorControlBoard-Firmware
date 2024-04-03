// /*
//  * Copyright (c) 2022, LexxPluss Inc.
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright notice,
//  *    this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  *    this list of conditions and the following disclaimer in the documentation
//  *    and/or other materials provided with the distribution.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
//  * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */

// #pragma once

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <cstdio>
#include "ros/node_handle.h"
#include "lexxauto_msgs/Battery.h"
#include "can_controller.hpp"

namespace lexxhard::zcan_bmu {

LOG_MODULE_REGISTER(zcan_bmu);

class zcan_bmu {
public:
//     void init(ros::NodeHandle &nh) {
//         nh.advertise(pub);
//         msg.temps = temps;
//         msg.temps_length = sizeof temps / sizeof temps[0];
//     }
//     void poll() {
//         can_controller::msg_bmu message;
//         while (k_msgq_get(&can_controller::msgq_bmu, &message, K_NO_WAIT) == 0) {
//             float cell_voltage[2];
//             cell_voltage[0] = message.max_cell_voltage.value;
//             cell_voltage[1] = message.min_cell_voltage.value;
//             char serial[8];
//             snprintf(serial, sizeof serial, "%04x", message.serial);
//             if (message.mod_status1 & 0b01000000)
//                 msg.state.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
//             else if (message.charging_current > 0)
//                 msg.state.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
//             else
//                 msg.state.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
//             if (message.mod_status1 & 0b00100000 ||
//                 message.bmu_status == 0x07 ||
//                 message.bmu_status == 0x09 ||
//                 message.bmu_alarm1 & 0b10000010)
//                 msg.state.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
//             else if (message.mod_status1 & 0b00011000)
//                 msg.state.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
//             else if (message.mod_status2 & 0b11100000)
//                 msg.state.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
//             else if (message.mod_status1 & 0b10000111 ||
//                      message.mod_status2 & 0b00000001 ||
//                      message.bmu_status == 0xf0 ||
//                      message.bmu_status == 0xf1 ||
//                      message.bmu_alarm1 & 0b01111101 ||
//                      message.bmu_alarm2 & 0b00000001)
//                 msg.state.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
//             else
//                 msg.state.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
//             msg.state.voltage = message.pack_voltage * 1e-3f;
//             msg.state.current = message.pack_current * 1e-2f;
//             msg.state.charge = message.remain_capacity * 1e-2f;
//             msg.state.capacity = message.full_charge_capacity * 1e-2f;
//             msg.state.design_capacity = message.design_capacity * 1e-2f;
//             msg.state.percentage = message.rsoc * 1e-2f;
//             msg.state.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
//             msg.state.present = true;
//             msg.state.cell_voltage_length = 2;
//             msg.state.cell_voltage = cell_voltage;
//             msg.state.location = "0";
//             msg.state.serial_number = serial;
//             msg.temps[0].temperature = message.min_temp.value * 1e-1f;
//             msg.temps[1].temperature = message.max_temp.value * 1e-1f;
//             msg.temps[2].temperature = message.fet_temp * 1e-1f;
//             msg.state_of_health = message.soh;
//             pub.publish(&msg);
    //     }
    // }
private:
    // lexxauto_msgs::Battery msg;
    // sensor_msgs::Temperature temps[3];
    // ros::Publisher pub{"/sensor_set/battery", &msg};
};

}

// // vim: set expandtab shiftwidth=4:
