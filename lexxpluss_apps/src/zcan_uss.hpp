/*
 * Copyright (c) 2023, LexxPluss Inc.
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
#include "uss_controller.hpp"

#define CAN_ID_USS 0x204
#define CAN_DATA_LENGTH_USS 8

namespace lexxhard::zcan_uss {

LOG_MODULE_REGISTER(zcan_uss);

class zcan_uss {
public:
    void init() {
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)){
            LOG_ERR("CAN_2 is not ready");
            return;
        }
    }
    void poll() {
        uss_controller::msg message;

        while (k_msgq_get(&uss_controller::msgq, &message, K_NO_WAIT) == 0) {
            uint8_t packedData[CAN_DATA_LENGTH_USS] {0}; 
            uint16_t data1 = (uint16_t)(message.front_left / 2);
            uint16_t data2 = (uint16_t)(message.front_right / 2);
            uint16_t data3 = (uint16_t)(message.left / 2);
            uint16_t data4 = (uint16_t)(message.right / 2);
            uint16_t data5 = (uint16_t)(message.back / 2);

            // 8-byte : | byte0 | byte1 | byte2 | byte3 | byte4 | byte5 | byte6 | byte7 |
            // 8-byte : |   data1   |   data2   |   data3   |   data4   |   data5   | x |
            packedData[0] = (data1 & 0xFF0) >> 4;                         
            packedData[1] = ((data1 & 0x00F) << 4) | (data2 >> 8);   
            packedData[2] = data2 & 0x0FF;                             
            packedData[3] = (data3 & 0xFF0) >> 4;
            packedData[4] = ((data3 & 0x00F) << 4) | (data4 >> 8);
            packedData[5] = data4 & 0x0FF;
            packedData[6] = (data5 & 0xFF0) >> 4;
            packedData[7] = (data5 & 0x00F) << 4;    

            // set data to CAN frame
            can_frame frame{
                .id = CAN_ID_USS,
                .dlc = CAN_DATA_LENGTH_USS,
                .data = {0}
            };
            
            // copy packedData to CAN frame data
            memcpy(frame.data, packedData, CAN_DATA_LENGTH_USS);

            can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
        }
    }
private:
    const device *dev{nullptr};
};

}

// vim: set expandtab shiftwidth=4:
