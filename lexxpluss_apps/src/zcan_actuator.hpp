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
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "actuator_controller.hpp"

#define CAN_ID_ACTUATOR_CONTROL 0x208 // based on saito-san's CAN ID assignment
#define CAN_ID_ACTUATOR_ENCODER 0x209 // based on saito-san's CAN ID assignment
#define CAN_ID_ACTUATOR_CURRENT 0x20a // based on saito-san's CAN ID assignment
#define CAN_DATALENGTH_ACTUATOR_ENCODER 6
#define CAN_DATALENGTH_ACTUATOR_CURRENT 8

namespace lexxhard::zcan_actuator {

LOG_MODULE_REGISTER(zcan_actuator);
char __aligned(4) msgq_can_actuator_control_buffer[8 * sizeof(actuator_controller::can_format_control)];
// k_msgq msgq_can_actuator_control;
CAN_MSGQ_DEFINE(msgq_can_actuator_control, 16);

class zcan_actuator {
public:
    void init() {
        // can device bind
        k_msgq_init(&msgq_can_actuator_control, msgq_can_actuator_control_buffer, sizeof(actuator_controller::can_format_control), 8);
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)){
            LOG_INF("CAN_2 is not ready");
            return;
        }

        // setup can filter
        static const can_filter filter_actuator_control{
            .id{CAN_ID_ACTUATOR_CONTROL},
            .mask{0x7ff}
        };
        // can_attach_msgq(dev, &msgq_can_actuator_control, &filter_actuator_control);
        can_add_rx_filter_msgq(dev, &msgq_can_actuator_control, &filter_actuator_control);
        return;
    }
    void poll() {
        // send to IPC of sensor informations
        actuator_controller::msg message;
        while (k_msgq_get(&actuator_controller::msgq, &message, K_NO_WAIT) == 0) {
            // ROS:[center,left,right], ROBOT:[left,center,right]
            can_frame can_frame_actuator_encoder{
                .id = CAN_ID_ACTUATOR_ENCODER,
                .dlc = CAN_DATALENGTH_ACTUATOR_ENCODER
            };
            actuator_controller::can_format_encoder tmp_enc = actuator_controller::can_format_encoder(message.encoder_count[0], message.encoder_count[1], message.encoder_count[2]);
            memcpy(can_frame_actuator_encoder.data, &tmp_enc, CAN_DATALENGTH_ACTUATOR_ENCODER);

            can_frame can_frame_actuator_current{
                .id = CAN_ID_ACTUATOR_CURRENT,
                .dlc = CAN_DATALENGTH_ACTUATOR_CURRENT
            };
            actuator_controller::can_format_current tmp_cur = actuator_controller::can_format_current(message.current[0], message.current[1], message.current[2], message.connect);
            memcpy(can_frame_actuator_current.data, &tmp_cur, CAN_DATALENGTH_ACTUATOR_CURRENT);

            can_send(dev, &can_frame_actuator_encoder, K_MSEC(100), nullptr, nullptr);
            can_send(dev, &can_frame_actuator_current, K_MSEC(100), nullptr, nullptr);
        }

        // receive from IPC of motion control
        while (k_msgq_get(&msgq_can_actuator_control, &frame_cntl, K_NO_WAIT) == 0) {
            msg_cntl = actuator_controller::msg_control(frame_cntl);
            while (k_msgq_put(&actuator_controller::msgq_control, &msg_cntl, K_NO_WAIT) != 0)
                k_msgq_purge(&actuator_controller::msgq_control);
        }
    }
private:
    actuator_controller::can_format_control frame_cntl;
    actuator_controller::msg_control msg_cntl;
    const device *dev{nullptr};
};

}

// vim: set expandtab shiftwidth=4:
