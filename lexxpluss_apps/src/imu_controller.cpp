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

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <shell/shell.h>
#include "imu_controller.hpp"
#include "runaway_detector.hpp"

namespace lexxhard::imu_controller {

LOG_MODULE_REGISTER(imu);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];
static struct sensor_trigger data_trigger;

class imu_fetcher {
public:
    int init() {
        imu_instance = this;

        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);

        dev = device_get_binding(DT_LABEL(DT_INST(0, invensense_icm42605))); //ICM-42605's register map is same as IIM-42652
        
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
        fsr_gyro.val1 = 7; // 15DPS Defined in the icm42605_reg.h
        fsr_gyro.val2 = 0; 
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
        if (sensor_trigger_set(dev, &data_trigger, &imu_fetcher::handle_iim42652) < 0) {
		    LOG_ERR("Cannot configure data trigger!!!\n");
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
    static inline float sensor_value_to_float(const struct sensor_value *val) {
        return (float)val->val1 + (float)val->val2 / 1000000;
    }

    static void handle_iim42652(const struct device *dev, struct sensor_trigger *trig) {
        int rc = imu_instance->process_iim42652(dev);

        if (rc != 0) {
            LOG_ERR("cancelling trigger due to failure: %d\n", rc);
            (void)sensor_trigger_set(dev, trig, NULL);
            return;
        }
    }

    int process_iim42652(const struct device *dev) {
        struct sensor_value accel[3];
        struct sensor_value gyro[3];
        struct sensor_value temperature;
        // msg message

        if (sensor_sample_fetch(dev) == 0) {
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
            sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
            sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
        } else {
            LOG_ERR("IMU sample fetch failed\n");
            return -1;
        }

        imu_instance->message.accel[0] = (int16_t)(sensor_value_to_double(&accel[0]) * -1 * 1000); // X [mm/s/s]
        imu_instance->message.accel[1] = (int16_t)(sensor_value_to_double(&accel[1]) * 1000);      // Y [mm/s/s]
        imu_instance->message.accel[2] = (int16_t)(sensor_value_to_double(&accel[2]) * -1 * 1000); // Z [mm/s/s]
        imu_instance->message.gyro[0] = (int16_t)(sensor_value_to_double(&gyro[0]) * -1 * 1000);   // X [mdeg/s]
        imu_instance->message.gyro[1] = (int16_t)(sensor_value_to_double(&gyro[1]) * 1000);        // Y [mdeg/s]
        imu_instance->message.gyro[2] = (int16_t)(sensor_value_to_double(&gyro[2]) * -1 * 1000);   // Z [mdeg/s]
        imu_instance->message.temp = (int16_t)(sensor_value_to_double(&temperature) * 1000);       // Temperature [mdeg C]
        imu_instance->message.delta_ang[0] = 0;
        imu_instance->message.delta_ang[1] = 0;
        imu_instance->message.delta_ang[2] = 0;
        imu_instance->message.delta_vel[0] = 0;
        imu_instance->message.delta_vel[1] = 0;
        imu_instance->message.delta_vel[2] = 0;

        while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq);

        runaway_detector::msg message_runaway{
            .accel{sensor_value_to_float(&accel[0]) * -1, sensor_value_to_float(&accel[1]), sensor_value_to_float(&accel[2]) * -1},
            .gyro{sensor_value_to_float(&gyro[0]) * -1, sensor_value_to_float(&gyro[1]), sensor_value_to_float(&gyro[2]) * -1}
        };

        while (k_msgq_put(&runaway_detector::msgq, &message_runaway, K_NO_WAIT) != 0)
            k_msgq_purge(&runaway_detector::msgq);

        return 0;
    }
    static imu_fetcher* imu_instance;
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

imu_fetcher* imu_fetcher::imu_instance = nullptr;
k_thread thread;
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
