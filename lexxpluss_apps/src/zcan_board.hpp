/*
 * Copyright (c) 2022, LexxPluss Inc.
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
#include <zephyr/logging/log.h>
#include "can_controller.hpp"

#define CAN_ID_BOARD_TX 0x20C
#define CAN_ID_BOARD_RX 0x20F
#define CAN_TX_DATA_LENGTH_BOARD 6
#define CAN_RX_DATA_LENGTH_BOARD 4

namespace lexxhard::zcan_board {

LOG_MODULE_REGISTER(zcan_board);

char __aligned(4) msgq_ros2board_buffer[8 * sizeof (can_controller::msg_control)];
CAN_MSGQ_DEFINE(msgq_can_ros2board, 16);

class zcan_board {
public:
    void init() {
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));    //CAN(to IPC)
        if (!device_is_ready(dev)){
            LOG_ERR("CAN_2 is not ready");
            return;
        }

        static const can_filter filter_led{
            .id{CAN_ID_BOARD_RX},
            .mask{0x7ff}
        };

        can_add_rx_filter_msgq(dev, &msgq_can_ros2board, &filter_led);

        return;
    }
    void poll() {
        uint8_t packedData[CAN_TX_DATA_LENGTH_BOARD] {0};
        can_controller::msg_board message;

        while (k_msgq_get(&can_controller::msgq_board, &message, K_NO_WAIT) == 0) {
            can_frame frame{
                .id = CAN_ID_BOARD_TX,
                .dlc = CAN_TX_DATA_LENGTH_BOARD,
                .data = {0}
            };

            packedData[0] = 0;

            if (message.bumper_switch_asserted) {
                packedData[0] |= 0b11000000;
            }
            if (message.emergency_switch_asserted) {
                packedData[0] |= 0b00110000;
            }
            if (message.wait_shutdown_state) {
                packedData[0] |= 0b00001000;
            }
            if (message.emergency_state) {
                packedData[0] |= 0b00000100;
            }
            if (message.safety_lidar_asserted) {
                packedData[0] |= 0b00000010;
            }

            if (message.auto_charging_status){
                packedData[1] = 0x01;
            } else if (message.manual_charging_status) {
                packedData[1] = 0x02;
            } else {
                packedData[1] = 0x00;
            }

            packedData[2] = message.shutdown_reason;
            packedData[3] = message.charge_heartbeat_delay;

            uint16_t scaled_voltage = static_cast<uint16_t>(message.charge_connector_voltage); //unit is mv

            packedData[4] = (uint8_t)(scaled_voltage >> 8);   // Upper Byte
            packedData[5] = (uint8_t)(scaled_voltage & 0xFF); // Lower Byte

            memcpy(frame.data, packedData, CAN_TX_DATA_LENGTH_BOARD);
            can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
        }

        can_frame frame;
        while (k_msgq_get(&msgq_can_ros2board, &frame, K_NO_WAIT) == 0) {
            can_controller::msg_control msg;

            msg.emergency_stop = frame.data[0] & 0x01;
            msg.power_off = frame.data[1] & 0x01;
            msg.wheel_power_off = frame.data[2] & 0x01;
            msg.heart_beat = frame.data[3] & 0x01;
            msg.lockdown = frame.data[4] & 0x01;

            if(prev_msg.emergency_stop != msg.emergency_stop) {
                LOG_INF("Emergency Stop: %d", msg.emergency_stop);
                prev_msg.emergency_stop = msg.emergency_stop;
            }
            if (prev_msg.power_off != msg.power_off) {
                LOG_INF("Power Off: %d", msg.power_off);
                prev_msg.power_off = msg.power_off;
            }
            if (prev_msg.wheel_power_off != msg.wheel_power_off) {
                LOG_INF("Wheel Power Off: %d", msg.wheel_power_off);
                prev_msg.wheel_power_off = msg.wheel_power_off;
            }
            if (prev_msg.heart_beat != msg.heart_beat) {
                LOG_INF("Heart Beat: %d", msg.heart_beat);
                prev_msg.heart_beat = msg.heart_beat;
            }
            if (prev_msg.lockdown != msg.lockdown) {
                LOG_INF("Lockdown: %d", msg.lockdown);
                prev_msg.lockdown = msg.lockdown;
            }

            while (k_msgq_put(&can_controller::msgq_control, &msg, K_NO_WAIT) != 0)
                k_msgq_purge(&can_controller::msgq_control);
        }

        return;
    }

private:
    const device *dev{nullptr};
    can_controller::msg_control prev_msg{0};
};

}

// vim: set expandtab shiftwidth=4:
