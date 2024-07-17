/*
 * Copyright (c) 2023, LexxPlgpio Inc.
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
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "gpio_controller.hpp"

#define CAN_ID_GPIO_OUT 0x211
#define CAN_ID_GPIO_IN 0x212
#define CAN_DATA_LENGTH_GPIO_OUT 1
#define CAN_DATA_LENGTH_GPIO_IN 1

namespace lexxhard::zcan_gpio {

LOG_MODULE_REGISTER(zcan_gpio);

char __aligned(4) msgq_led_buffer[8 * sizeof (struct can_frame)];

CAN_MSGQ_DEFINE(msgq_can_gpio, 16);

class zcan_gpio {
public:
    void init() {
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)){
            LOG_ERR("CAN_2 is not ready");
            return;
        }

        static const can_filter filter{
            .id{CAN_ID_GPIO_OUT},
            .mask{0x7ff}
        };
        can_add_rx_filter_msgq(dev, &msgq_can_gpio, &filter);
    }
    void poll() {
        gpio_controller::msg message;
        while (k_msgq_get(&gpio_controller::msgq, &message, K_NO_WAIT) == 0) {
            uint8_t packedData[CAN_DATA_LENGTH_GPIO_IN] {
                pack_gpio_input_status(message)
            };

            // set data to CAN frame
            can_frame frame{
                .id = CAN_ID_GPIO_IN,
                .dlc = CAN_DATA_LENGTH_GPIO_IN,
                .data = {0}
            };

            // copy packedData to CAN frame data
            memcpy(frame.data, packedData, CAN_DATA_LENGTH_GPIO_IN);

            can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
        }

        struct can_frame frame;
        while (k_msgq_get(&msgq_can_gpio, &frame, K_NO_WAIT) == 0) {
            if (frame.id != CAN_ID_GPIO_OUT || frame.dlc != CAN_DATA_LENGTH_GPIO_OUT) {
                LOG_INF("Unknown CAN frame received: %x %x", frame.id, frame.dlc);
                continue;
            }

            const gpio_controller::msg_control msg{unpack_gpio_output_status(frame)};
            while (k_msgq_put(&gpio_controller::msgq_control, &msg, K_NO_WAIT) != 0)
                k_msgq_purge(&gpio_controller::msgq_control);
        }
    }
private:
    uint8_t pack_gpio_input_status(gpio_controller::msg const &message) const {
        uint8_t ret{0};
        ret |= static_cast<uint8_t>(message.gpio_in_0) << 7;
        ret |= static_cast<uint8_t>(message.gpio_in_1) << 6;
        ret |= static_cast<uint8_t>(message.gpio_in_2) << 5;
        ret |= static_cast<uint8_t>(message.gpio_in_3) << 4;

        return ret;
    }

    gpio_controller::msg_control unpack_gpio_output_status(can_frame const &frame) const {
        gpio_controller::msg_control ret;
        ret.ros_gpio_out_0 = static_cast<bool>((frame.data[0] & 0b10000000) >> 7);
        ret.ros_gpio_out_1 = static_cast<bool>((frame.data[0] & 0b01000000) >> 6);
        ret.ros_gpio_out_2 = static_cast<bool>((frame.data[0] & 0b00100000) >> 5);
        ret.ros_gpio_out_3 = static_cast<bool>((frame.data[0] & 0b00010000) >> 4);

        return ret;
    }

    const device *dev{nullptr};
};

}