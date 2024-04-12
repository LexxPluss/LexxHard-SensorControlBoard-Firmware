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
#include "bmu_controller.hpp"

namespace lexxhard::zcan_bmu {

LOG_MODULE_REGISTER(zcan_bmu);

class zcan_bmu {
public:
    void init() {
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));    //CAN(to IPC)
        if (!device_is_ready(dev)){
            LOG_INF("CAN_2 is not ready");
            return;
        }

        return;
    }
    void poll() {
        bmu_controller::msg_can_bmu message;

        while (k_msgq_get(&bmu_controller::msgq_rawframe_bmu, &message, K_NO_WAIT) == 0) {
            can_frame frame_bmu;

            frame_bmu.id = message.can_id;
            frame_bmu.dlc = BMU_CAN_DATA_LENGTH;
            frame_bmu.data[0] = message.frame[0];
            frame_bmu.data[1] = message.frame[1];
            frame_bmu.data[2] = message.frame[2];
            frame_bmu.data[3] = message.frame[3];
            frame_bmu.data[4] = message.frame[4];
            frame_bmu.data[5] = message.frame[5];
            frame_bmu.data[6] = message.frame[6];
            frame_bmu.data[7] = message.frame[7];
                       
            can_send(dev, &frame_bmu, K_MSEC(100), nullptr, nullptr);
        }
    }
private:
    const device *dev{nullptr};
};

}

// // vim: set expandtab shiftwidth=4:
