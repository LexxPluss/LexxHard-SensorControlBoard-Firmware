/*
 * Copyright (c) 2023, LexxPltug_encoder Inc.
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
#include "tug_encoder_controller.hpp"

#define CAN_ID_TUG_ENCODER 0x210
#define CAN_DATA_LENGTH_TUG_ENCODER 2

namespace lexxhard::zcan_tug_encoder {

LOG_MODULE_REGISTER(zcan_tug_encoder);

class zcan_tug_encoder {
public:
    void init() {
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)){
            LOG_ERR("CAN_2 is not ready");
            return;
        }
    }
    void poll() {
        tug_encoder_controller::msg message;

        while (k_msgq_get(&tug_encoder_controller::msgq, &message, K_NO_WAIT) == 0) {
            uint8_t packedData[CAN_DATA_LENGTH_TUG_ENCODER] {0};
            packedData[0] = message.angle >> 8;
            packedData[1] = message.angle & 0xFF;

            // set data to CAN frame
            can_frame frame{
                .id = CAN_ID_TUG_ENCODER,
                .dlc = CAN_DATA_LENGTH_TUG_ENCODER,
                .data = {0}
            };

            // copy packedData to CAN frame data
            memcpy(frame.data, packedData, CAN_DATA_LENGTH_TUG_ENCODER);

            can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
        }
    }
private:
    const device *dev{nullptr};
};

}

