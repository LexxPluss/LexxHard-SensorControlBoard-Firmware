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

namespace lexxhard::imu_controller {

LOG_MODULE_REGISTER(imu);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

class {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);

        dev = DEVICE_DT_GET_ONE(invensense_icm42605); //ICM-42605's register map is same as IIM-42652
        
        if (!device_is_ready(dev)) {
            LOG_ERR("IMU device not found");
            return -1;
        }

        struct sensor_value odr;
        odr.val1 = 100; // 100Hz
        odr.val2 = 0; 
        if (sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr) < 0) {
            LOG_ERR("IMU ODR Setting Fail\n");
        }

        struct sensor_value fsr_accel;
        fsr_accel.val1 = 3; // 2G Defined in the icm42605_reg.h
        fsr_accel.val2 = 0; 
        if (sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &fsr_accel) < 0) {
            LOG_ERR("IMU ODR Setting Fail\n");
        } 

        struct sensor_value fsr_gyro;
        fsr_accel.val1 = 7; // 15DPS Defined in the icm42605_reg.h
        fsr_accel.val2 = 0; 
        if (sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &fsr_gyro) < 0) {
            LOG_ERR("IMU ODR Setting Fail\n");
        } 

        for (int i{0}; i < 3; ++i) {
            message.accel[i] = 0;
            message.gyro[i] = 0;
            message.delta_ang[i] = 0;
            message.delta_vel[i] = 0;
        }
        message.temp = 0;

        LOG_INF("IMU controller for LexxPluss board. (%p)", dev);

        return 0;
    }
    void run() {
        if (!device_is_ready(dev)) {
            LOG_ERR("IMU device not found");
            return;
        }

        data_trigger = (struct sensor_trigger) {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_ALL,
	    };
        
        // data to be read from the sensor triggered by the interrupt signal
        if (sensor_trigger_set(dev, &data_trigger, handle_iim42605) < 0) {
		    printf("Cannot configure data trigger!!!\n");
		    return;
	    }

        while (true) {
            k_msleep(1);
        }
    }
    void info(const shell *shell) const {
        msg m{message};
        shell_print(shell,
                    "accel: %d %d %d (mm/s/s)\n"
                    "gyro: %d %d %d (mdeg/s)\n"
                    "vel: %d %d %d (mm/s)\n"
                    "ang: %d %d %d (mdeg)\n"
                    "temp: %fdeg",
                    m.accel[0], m.accel[1], m.accel[2],
                    m.gyro[0], m.gyro[1], m.gyro[2],
                    m.delta_vel[0], m.delta_vel[1], m.delta_vel[2],
                    m.delta_ang[0], m.delta_ang[1], m.delta_ang[2],
                    (float)m.temp / 1000);
    }
private:
    static void handle_iim42652(const struct device *dev, const struct sensor_trigger *trig) {
        int rc = process_iim42652(dev);

        if (rc != 0) {
            printf("cancelling trigger due to failure: %d\n", rc);
            (void)sensor_trigger_set(dev, trig, NULL);
            return;
        }
    }

    static int process_iim42652(const struct device *dev) {
        struct sensor_value accel[3];
        struct sensor_value gyro[3];
        struct sensor_value temperature;

        if (sensor_sample_fetch(dev) == 0) {
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
            sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
            sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
        } else {
            LOG_ERR("IMU sample fetch failed\n");
            return -1;
        }

        message.accel[0] = (int16_t)(sensor_value_to_milli(&accel[0]) * -1);        // X [mm/s/s]
        message.accel[1] = (int16_t)sensor_value_to_milli(&accel[1]);               // Y [mm/s/s]
        message.accel[2] = (int16_t)(sensor_value_to_milli(&accel[2]) * -1);        // Z [mm/s/s]
        message.gyro[0] = (int16_t)(sensor_rad_to_degrees(&gyro[0]) * -1 * 1000);   // X [mdeg/s]
        message.gyro[1] = (int16_t)sensor_rad_to_degrees(&gyro[1]);                 // Y [mdeg/s]
        message.gyro[2] = (int16_t)(sensor_rad_to_degrees(&gyro[2]) * -1 * 1000);   // Z [mdeg/s]
        message.temp = (int16_t)(sensor_value_to_milli(&temperature));              // Temperature [mdeg C]
        message.delta_ang[0] = 0;
        message.delta_ang[1] = 0;
        message.delta_ang[2] = 0;
        message.delta_vel[0] = 0;
        message.delta_vel[1] = 0;
        message.delta_vel[2] = 0;

        while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq);

        runaway_detector::msg message_runaway{
            .accel{message.accel[0], message.accel[1], message.accel[2]},
            .gyro{message.gyro[0], message.gyro[1], message.gyro[2]}
        };

        while (k_msgq_put(&runaway_detector::msgq, &message_runaway, K_NO_WAIT) != 0)
            k_msgq_purge(&runaway_detector::msgq);

        return 0;
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
