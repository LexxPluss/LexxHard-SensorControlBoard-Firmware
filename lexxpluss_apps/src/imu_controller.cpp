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

#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <shell/shell.h>
#include "imu_controller.hpp"
#include "runaway_detector.hpp"
#include <limits>

namespace lexxhard::imu_controller {

LOG_MODULE_REGISTER(imu);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

class {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
        dev = device_get_binding("ADIS16470");
        if (!device_is_ready(dev))
            return -1;
        for (int i{0}; i < 3; ++i) {
            message.accel[i] = 0;
            message.gyro[i] = 0;
            message.delta_ang[i] = 0;
            message.delta_vel[i] = 0;
            message.accel_max[i] = std::numeric_limits<float>::min();
            message.accel_min[i] = std::numeric_limits<float>::max();
        }
        message.temp = 0;
        LOG_INF("IMU controller for LexxPluss board. (%p)", dev);
        return 0;
    }
    void run() {
        if (!device_is_ready(dev)){
            LOG_INF("IMU Not Ready");
            return;
        }
        while (true) {
            // if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL) == 0) {
            //     message.accel[0] = get_sensor_value_as_float(SENSOR_CHAN_ACCEL_X);
            //     message.accel[1] = get_sensor_value_as_float(SENSOR_CHAN_ACCEL_Y);
            //     message.accel[2] = get_sensor_value_as_float(SENSOR_CHAN_ACCEL_Z);
            //     message.gyro[0] = get_sensor_value_as_float(SENSOR_CHAN_GYRO_X);
            //     message.gyro[1] = get_sensor_value_as_float(SENSOR_CHAN_GYRO_Y);
            //     message.gyro[2] = get_sensor_value_as_float(SENSOR_CHAN_GYRO_Z);
            //     message.temp = get_sensor_value_as_float(SENSOR_CHAN_DIE_TEMP);
            //     message.delta_ang[0] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START);
            //     message.delta_ang[1] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 1);
            //     message.delta_ang[2] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 2);
            //     message.delta_vel[0] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 3);
            //     message.delta_vel[1] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 4);
            //     message.delta_vel[2] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 5);
            //     while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
            //         k_msgq_purge(&msgq);
            //     runaway_detector::msg message_runaway{
            //         .accel{message.accel[0], message.accel[1], message.accel[2]},
            //         .gyro{message.gyro[0], message.gyro[1], message.gyro[2]}
            //     };
            //     while (k_msgq_put(&runaway_detector::msgq, &message_runaway, K_NO_WAIT) != 0)
            //         k_msgq_purge(&runaway_detector::msgq);
            // }
            // k_msleep(1);

            //Initialize Variables
            for (int i{0}; i < 3; ++i) {
                message.accel[i] = 0;
                message.gyro[i] = 0;
                message.delta_ang[i] = 0;
                message.delta_vel[i] = 0;
                message.accel_max[i] = std::numeric_limits<float>::min();
                message.accel_min[i] = std::numeric_limits<float>::max();
            }
            message.temp = 0;

            int ave_num = 5; //Base freq 50Hz, if sampling freq is 500Hz(configured in adis16470_configure), ave_num = 10
            int delay_ms = int(2 * ave_num); //2ms(500Hz) * ave_num = delay_ms

            for (int ii{0}; ii < ave_num; ii++){
                if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL) == 0) {
                    float temp_acc[3] = {get_sensor_value_as_float(SENSOR_CHAN_ACCEL_X), get_sensor_value_as_float(SENSOR_CHAN_ACCEL_Y), get_sensor_value_as_float(SENSOR_CHAN_ACCEL_Z)};

                    message.accel[0] += temp_acc[0];
                    message.accel[1] += temp_acc[1];
                    message.accel[2] += temp_acc[2];
                    message.gyro[0] += get_sensor_value_as_float(SENSOR_CHAN_GYRO_X);
                    message.gyro[1] += get_sensor_value_as_float(SENSOR_CHAN_GYRO_Y);
                    message.gyro[2] += get_sensor_value_as_float(SENSOR_CHAN_GYRO_Z);
                    message.temp += get_sensor_value_as_float(SENSOR_CHAN_DIE_TEMP);
                    message.delta_ang[0] += get_sensor_value_as_float(SENSOR_CHAN_PRIV_START);
                    message.delta_ang[1] += get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 1);
                    message.delta_ang[2] += get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 2);
                    message.delta_vel[0] += get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 3);
                    message.delta_vel[1] += get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 4);
                    message.delta_vel[2] += get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 5);

                    for(int jj{0}; jj < 3; jj++){
                        //MAX
                        if(message.accel_max[jj] < temp_acc[jj]){
                            message.accel_max[jj] = temp_acc[jj];
                        }
                        //MIN
                        if(message.accel_min[jj] > temp_acc[jj]){
                            message.accel_min[jj] = temp_acc[jj];
                        }
                    }

                    //Last Data for the cycle
                    if(ii == (ave_num - 1)){
                        message.accel[0]        /= ave_num;
                        message.accel[1]        /= ave_num;
                        message.accel[2]        /= ave_num;
                        message.gyro[0]         /= ave_num;
                        message.gyro[1]         /= ave_num;
                        message.gyro[2]         /= ave_num;
                        message.temp            /= ave_num;
                        message.delta_ang[0]    /= ave_num;
                        message.delta_ang[1]    /= ave_num;
                        message.delta_ang[2]    /= ave_num;
                        message.delta_vel[0]    /= ave_num;
                        message.delta_vel[1]    /= ave_num;
                        message.delta_vel[2]    /= ave_num;
                        while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
                            k_msgq_purge(&msgq);
                        runaway_detector::msg message_runaway{
                            .accel{message.accel[0], message.accel[1], message.accel[2]},
                            .gyro{message.gyro[0], message.gyro[1], message.gyro[2]}
                        };
                        while (k_msgq_put(&runaway_detector::msgq, &message_runaway, K_NO_WAIT) != 0)
                            k_msgq_purge(&runaway_detector::msgq);
                    }
                }
                k_msleep(delay_ms); //sampling freq is 500Hz = 2ms
            }
        }
    }
    void info(const shell *shell) const {
        msg m{message};
        shell_print(shell,
                    "accel: %f %f %f (m/s/s)\n"
                    "gyro: %f %f %f (deg/s)\n"
                    "vel: %f %f %f (m/s)\n"
                    "ang: %f %f %f (deg)\n"
                    "temp: %fdeg\n"
                    "accel_max: %f %f %f (m/s/s)\n"
                    "accel_min: %f %f %f (m/s/s)\n",
                    m.accel[0], m.accel[1], m.accel[2],
                    m.gyro[0], m.gyro[1], m.gyro[2],
                    m.delta_vel[0], m.delta_vel[1], m.delta_vel[2],
                    m.delta_ang[0], m.delta_ang[1], m.delta_ang[2],
                    m.temp,
                    m.accel_max[0], m.accel_max[1], m.accel_max[2],
                    m.accel_min[0], m.accel_min[1], m.accel_min[2]);
    }
private:
    float get_sensor_value_as_float(enum sensor_channel chan, uint32_t offset) const {
        chan = static_cast<enum sensor_channel>(static_cast<uint32_t>(chan) + offset);
        return get_sensor_value_as_float(chan);
    }
    float get_sensor_value_as_float(enum sensor_channel chan) const {
        sensor_value v;
        sensor_channel_get(dev, chan, &v);
        return static_cast<float>(v.val1) + static_cast<float>(v.val2) * 1e-6f;
    }
    const device *dev{nullptr};
    msg message;
} impl;

int info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(info, NULL, "IMU information", info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(imu, &sub, "IMU commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread thread;
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
