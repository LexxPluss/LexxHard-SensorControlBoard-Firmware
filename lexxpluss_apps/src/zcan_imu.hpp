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
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "imu_controller.hpp"

#define CAN_ID_ACCL 0x206
#define CAN_ID_GYRO 0x207
#define CAN_DATA_LENGTH_IMU 7

namespace lexxhard::zcan_imu {

LOG_MODULE_REGISTER(zcan_imu);

class zcan_imu {
public:
    void init() {
        dev = device_get_binding("CAN_2");  //CAN(to IPC)
        if (!device_is_ready(dev)){
            LOG_INF("CAN_2 is not ready");
            return;
        }

        return;
    }
    void poll() {
        imu_controller::msg message;

        while (k_msgq_get(&imu_controller::msgq, &message, K_NO_WAIT) == 0) {
            zcan_frame frame_imu[2]{
                {
                    .id = CAN_ID_ACCL,
                    .rtr = CAN_DATAFRAME,
                    .id_type = CAN_STANDARD_IDENTIFIER,
                    .dlc = CAN_DATA_LENGTH_IMU,
                    .data{
                        message.accel_data_upper[0],
                        message.accel_data_lower[0],
                        message.accel_data_upper[1],
                        message.accel_data_lower[1],
                        message.accel_data_upper[2],
                        message.accel_data_lower[2],
                        message.counter
                    }
                },
                {
                    .id = CAN_ID_GYRO,
                    .rtr = CAN_DATAFRAME,
                    .id_type = CAN_STANDARD_IDENTIFIER,
                    .dlc = CAN_DATA_LENGTH_IMU,
                    .data{
                        message.gyro_data_upper[0],
                        message.gyro_data_lower[0],
                        message.gyro_data_upper[1],
                        message.gyro_data_lower[1],
                        message.gyro_data_upper[2],
                        message.gyro_data_lower[2],
                        message.counter
                    }
                }
            };

            can_send(dev, &frame_imu[0], K_MSEC(100), nullptr, nullptr);    //accel
            can_send(dev, &frame_imu[1], K_MSEC(100), nullptr, nullptr);    //gyro
        }
    }
private:
    const device *dev{nullptr};
};
}

// vim: set expandtab shiftwidth=4:
