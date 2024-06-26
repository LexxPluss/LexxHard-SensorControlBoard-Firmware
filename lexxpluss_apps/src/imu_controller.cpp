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

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "common.hpp"
#include "imu_controller.hpp"
#include "runaway_detector.hpp"

namespace lexxhard::imu_controller {

LOG_MODULE_REGISTER(imu);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];
static struct sensor_trigger data_trigger;
bool int_flag{false};

class imu_fetcher {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);

        dev = DEVICE_DT_GET(DT_NODELABEL(imu0));

        if (!device_is_ready(dev)) {
            LOG_ERR("IMU device not found");
            return -1;
        }

        struct sensor_value odr_accel;
        odr_accel.val1 = 40; // 40Hz
        odr_accel.val2 = 0;
        if (sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_accel) < 0) {
            LOG_ERR("IMU ODR ACCEL Setting Fail\n");
        }

        struct sensor_value odr_gyro;
        odr_gyro.val1 = 40; // 40Hz
        odr_gyro.val2 = 0;
        if (sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_gyro) < 0) {
            LOG_ERR("IMU ODR GYRO Setting Fail\n");
        }

        struct sensor_value fsr_accel;
        fsr_accel.val1 = 3; // 2G Defined in the icm42605_reg.h
        fsr_accel.val2 = 0;
        if (sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &fsr_accel) < 0) {
            LOG_ERR("IMU FSR ACC Setting Fail\n");
        }

        struct sensor_value fsr_gyro;
        fsr_gyro.val1 = GYRO_FS_SEL; // 1000DPS Defined in the icm42605_reg.h
        fsr_gyro.val2 = 0;
        if (sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &fsr_gyro) < 0) {
            LOG_ERR("IMU FSR GYRO Setting Fail\n");
        }

        // initialize the messsage data
        for (int i = 0; i < 3; i++) {
            message.accel_data_lower[i] = 0;
            message.accel_data_upper[i] = 0;
            message.gyro_data_lower[i] = 0;
            message.gyro_data_upper[i] = 0;
        }
        message.counter = 0;

        return 0;
    }
    void run() {
        uint8_t counter{0};

        if (!device_is_ready(dev)) {
            LOG_ERR("IMU device not found");
            return;
        }

        data_trigger = (struct sensor_trigger) {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_ALL,
        };

        // data to be read from the sensor triggered by the interrupt signal
        if (sensor_trigger_set(dev, &data_trigger, cb_func) < 0) {
            LOG_ERR("Cannot configure data trigger!!!\n");
            return;
        }

        while (true) {
            // fetch data if interrupt is triggered
            if(int_flag) {
                if (sensor_sample_fetch(dev) == 0) {
                    struct sensor_value accel[3];
                    struct sensor_value gyro[3];
                    int16_t accel_data[3];
                    int16_t gyro_data[3];

                    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
                    sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

                    //sensor -x is system y, sensor -y is systemx, sensor z is system z
                    accel_data[1] = - accel_value_to_int16_t(&accel[0]);
                    accel_data[0] = - accel_value_to_int16_t(&accel[1]);
                    accel_data[2] = - accel_value_to_int16_t(&accel[2]);
                    gyro_data[1] = - gyro_rad_to_iim42652raw_int16_t(&gyro[0], GYRO_FS_SEL);
                    gyro_data[0] = - gyro_rad_to_iim42652raw_int16_t(&gyro[1], GYRO_FS_SEL);
                    gyro_data[2] = - gyro_rad_to_iim42652raw_int16_t(&gyro[2], GYRO_FS_SEL);

                    if(imu_bias_initial_cnt < IMU_BIAS_INITIAL_CNT_MAX) {
                        bias_gyro_z = gyro_data[2];
                        gyro_data[2] = 0;
                        imu_bias_initial_cnt++;
                    } else {
                        gyro_data[2] -= bias_gyro_z;
                    }

                    //split to upper and lower bytes for CAN message
                    for(int i = 0; i < 3; i++) {
                        message.accel_data_lower[i] = (uint8_t)(accel_data[i] & 0x00FF);
                        message.accel_data_upper[i] = (uint8_t)((accel_data[i] & 0xFF00) >> 8);
                        message.gyro_data_lower[i] = (uint8_t)(gyro_data[i] & 0x00FF);
                        message.gyro_data_upper[i] = (uint8_t)((gyro_data[i] & 0xFF00) >> 8);
                    }

                    message.counter = counter++;    // 0 to 255

                    // to ZCAN module
                    while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
                        k_msgq_purge(&msgq);

                    runaway_detector::msg message_runaway{
                        .accel{sensor_value_to_float(&accel[1]), sensor_value_to_float(&accel[0]), sensor_value_to_float(&accel[2])},
                        .gyro{sensor_value_to_float(&gyro[1]), sensor_value_to_float(&gyro[0]), sensor_value_to_float(&gyro[2])}
                    };

                    // to Runaway Detector
                    while (k_msgq_put(&runaway_detector::msgq, &message_runaway, K_NO_WAIT) != 0)
                        k_msgq_purge(&runaway_detector::msgq);
                }
                int_flag = false;
            }
            k_msleep(1);
        }
    }
    void info(const shell *shell) const {
        double accel_x = (double)((int16_t)(message.accel_data_upper[0] << 8 | message.accel_data_lower[0])) / 1000.0;
        double accel_y = (double)((int16_t)(message.accel_data_upper[1] << 8 | message.accel_data_lower[1])) / 1000.0;
        double accel_z = (double)((int16_t)(message.accel_data_upper[2] << 8 | message.accel_data_lower[2])) / 1000.0;
        double gyro_x = (double)((int16_t)(message.gyro_data_upper[0] << 8 | message.gyro_data_lower[0])) / 1000.0;
        double gyro_y = (double)((int16_t)(message.gyro_data_upper[1] << 8 | message.gyro_data_lower[1])) / 1000.0;
        double gyro_z = (double)((int16_t)(message.gyro_data_upper[2] << 8 | message.gyro_data_lower[2])) / 1000.0;

        shell_print(shell,
                    "accel: %f %f %f (m/s/s)\n"
                    "gyro: %f %f %f (deg/s)\n"
                    "counter: %u\n",
                    accel_x, accel_y, accel_z,
                    gyro_x, gyro_y, gyro_z,
                    (int16_t)message.counter);
    }
    static void cb_func(const struct device *dev, const struct sensor_trigger *trig_cb) {
        ARG_UNUSED(dev);
        ARG_UNUSED(trig_cb);

        int_flag = true;
        return;
    }
private:
    float sensor_value_to_float(const struct sensor_value *val) {
        return (float)val->val1 + (float)val->val2 * 1e-6f;
    }

    int16_t accel_value_to_int16_t(const struct sensor_value *val) {
        int64_t temp_value{0};

        temp_value = (int64_t)(val->val1 * 1e3f + val->val2 * 1e-3f);

        if(temp_value > INT16_MAX) {
            temp_value = INT16_MAX;
        } else if(temp_value < INT16_MIN) {
            temp_value = INT16_MIN;
        }

        return (int16_t)temp_value;
    }

    int16_t gyro_rad_to_deg_int16_t(const struct sensor_value *val) {
        int64_t temp_value{0};

        temp_value = (int64_t)((val->val1 + val->val2 * 1e-6) * (180.0 / M_PI) * 1e3);

        if(temp_value > INT16_MAX) {
            temp_value = INT16_MAX;
        } else if(temp_value < INT16_MIN) {
            temp_value = INT16_MIN;
        }

        return (int16_t)temp_value;
    }

    int16_t gyro_rad_to_iim42652raw_int16_t(const struct sensor_value *val_rad, int gyro_fs_sel) {
        const int gyro_sensitivity_x10[8] = {164, 328, 655, 1310, 2620, 5243, 10486, 20972};
        int64_t temp_value{0};

        temp_value = (int64_t)((val_rad->val1 + val_rad->val2 * 1e-6f) * (180.0 / M_PI) * (gyro_sensitivity_x10[gyro_fs_sel] / 10.0));

        if(temp_value > INT16_MAX) {
            temp_value = INT16_MAX;
        } else if(temp_value < INT16_MIN) {
            temp_value = INT16_MIN;
        }

        return (int16_t)temp_value;
    }

    msg message;
    const device *dev{nullptr};
    int imu_bias_initial_cnt{0};
    const int IMU_BIAS_INITIAL_CNT_MAX{255};
    int16_t bias_gyro_z{0};
    const int GYRO_FS_SEL = 1; // 1000DPS iim42652
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
