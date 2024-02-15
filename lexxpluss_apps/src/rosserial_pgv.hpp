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

#include <zephyr.h>
#include <cstdio>
#include <drivers/can.h>
#include "std_msgs/UInt8.h"
#include "lexxauto_msgs/PositionGuideVision.h"
#include "common.hpp"
#include "pgv_controller.hpp"

#define CAN_ID_PGV_1 0x200
#define CAN_ID_PGV_2 0x201
#define CAN_ID_PGV_3 0x202
#define CAN_DATA_LENGTH 8

namespace lexxhard {

class can_pgv {
public:
    int init()
    {
        //can device bind
        dev = device_get_binding("CAN_2");
        if (!device_is_ready(dev))
            return -1;
        ring_counter=0;
        return 0;
    }

    void poll() {
        pgv_controller::msg message;
        while (k_msgq_get(&pgv_controller::msgq, &message, K_NO_WAIT) == 0)
        {
            zcan_frame frame_pgv[3]{{
                .id = CAN_ID_PGV_1,
                .rtr = CAN_DATAFRAME,
                .id_type = CAN_STANDARD_IDENTIFIER,
                .dlc = CAN_DATA_LENGTH
            },{
                .id = CAN_ID_PGV_2,
                .rtr = CAN_DATAFRAME,
                .id_type = CAN_STANDARD_IDENTIFIER,
                .dlc = CAN_DATA_LENGTH
            },{
                .id = CAN_ID_PGV_3,
                .rtr = CAN_DATAFRAME,
                .id_type = CAN_STANDARD_IDENTIFIER,
                .dlc = CAN_DATA_LENGTH,
            }};

            // 0x200 PGV( 1~ 7Byte + counter)
            memcpy(frame_pgv[0].data,message.rawdata,7);
            frame_pgv[0].data[7] = ring_counter;
            // 0x201 PGV( 8~14Byte + counter)
            memcpy(frame_pgv[1].data,message.rawdata + 7 ,7);
            frame_pgv[1].data[7] = ring_counter;
            // 0x202 PGV(15~21Byte + counter)
            memcpy(frame_pgv[2].data,message.rawdata + 14 ,7);
            frame_pgv[2].data[7] = ring_counter;

            can_send(dev, &frame_pgv[0], K_MSEC(100), nullptr, nullptr);
            can_send(dev, &frame_pgv[1], K_MSEC(100), nullptr, nullptr);
            can_send(dev, &frame_pgv[2], K_MSEC(100), nullptr, nullptr);

            ring_counter++;
        }
    }
private:

    uint8_t ring_counter{0};

    //THIS FUNCTION SHOULD RE-WRITE WHEN USE BECAUSE STILL NOT CHANGED UNTIL ROS-SERIAL GEN
    //BUT THIS FUNCTION NOT USED
    void callback(const std_msgs::UInt8 &req) {
        switch (req.data) {
        case 0:
            snprintf(direction, sizeof direction, "No lane is selected");
            break;
        case 1:
            snprintf(direction, sizeof direction, "Right lane is selected");
            break;
        case 2:
            snprintf(direction, sizeof direction, "Left lane is selected");
            break;
        case 3:
            snprintf(direction, sizeof direction, "Straight Ahead");
            break;
        }
        pgv_controller::msg_control can2pgv;
        can2pgv.dir_command = req.data;
        while (k_msgq_put(&pgv_controller::msgq_control, &can2pgv, K_NO_WAIT) != 0)
            k_msgq_purge(&pgv_controller::msgq_control);
    }
    char direction[64]{"Straight Ahead"};
    const device *dev{nullptr};
};

}

// vim: set expandtab shiftwidth=4:
