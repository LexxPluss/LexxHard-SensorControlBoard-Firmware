/*
 * Copyright (c) 2024, LexxPluss Inc.
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

#include <algorithm>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "firmware_updater.hpp"

#define CAN_ID_DFU_DATA 0x20d
#define CAN_ID_DFU_RESP 0x20e

namespace lexxhard::zcan_dfu {

LOG_MODULE_REGISTER(zcan_dfu);

char __aligned(4) msgq_dfu_buffer[8 * (sizeof (struct can_frame))];

CAN_MSGQ_DEFINE(msgq_can_dfu, 16);

class zcan_dfu {
public:
    int init() {
        k_msgq_init(&msgq_can_dfu, msgq_dfu_buffer, sizeof (struct can_frame), 8);

        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)) {
            LOG_ERR("CAN_2 is not ready");
            return -1;
	}

        static const can_filter filter{
            .id{CAN_ID_DFU_DATA},
            .mask{0x7ff}
        };
        can_add_rx_filter_msgq(dev, &msgq_can_dfu, &filter);
        return 0;
    }


    void poll()
    {
    	if (can_frame frame; k_msgq_get(&msgq_can_dfu, &frame, K_NO_WAIT) == 0) {
    	    if (frame.id == CAN_ID_DFU_DATA && frame.dlc == sizeof (firmware_updater::command_packet)) {
    	        while (k_msgq_put(&firmware_updater::msgq_command, frame.data, K_NO_WAIT) != 0)
    	            k_msgq_purge(&firmware_updater::msgq_command);
    	    }
    	}
    	if (firmware_updater::response_packet response;
    	    k_msgq_get(&firmware_updater::msgq_response, &response, K_NO_WAIT) == 0) {
    	    can_frame frame{
    	        .id{CAN_ID_DFU_RESP},
    	        .dlc{sizeof response}
    	    };
    	    std::copy_n(reinterpret_cast<uint8_t*>(&response), sizeof response, frame.data);
    	    can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
    	}
    }
private:
    const device *dev{nullptr};
};

}
