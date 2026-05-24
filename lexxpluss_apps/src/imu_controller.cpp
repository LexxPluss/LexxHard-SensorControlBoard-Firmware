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

#include <algorithm>
#include <atomic>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "common.hpp"
#include "imu_calibration.hpp"
#include "imu_controller.hpp"
#include "runaway_detector.hpp"
#include "sensor/iim42652/iim42652_reg.h"
#include "sensor/iim42652/iim42652_setup.h"

namespace lexxhard::imu_controller {

LOG_MODULE_REGISTER(imu);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];
static struct sensor_trigger data_trigger;
std::atomic<bool> int_flag{false};

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

                    imu_calibration::feed_sample(accel, gyro);

                    //sensor -x is system y, sensor -y is systemx, sensor z is system z
                    accel_data[1] = - accel_value_to_int16_t(&accel[0]);
                    accel_data[0] = - accel_value_to_int16_t(&accel[1]);
                    accel_data[2] = - accel_value_to_int16_t(&accel[2]);
                    gyro_data[1] = - gyro_rad_to_iim42652raw_int16_t(&gyro[0]);
                    gyro_data[0] = - gyro_rad_to_iim42652raw_int16_t(&gyro[1]);
                    gyro_data[2] = - gyro_rad_to_iim42652raw_int16_t(&gyro[2]);

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
    void regdump(const struct shell *shell) const {
        if (!device_is_ready(dev)) {
            shell_error(shell, "IMU device not ready");
            return;
        }

        struct entry {
            const char *name;
            uint8_t bank;
            uint8_t addr;
        };
        static const entry table[] = {
            {"WHO_AM_I            (B0 0x75)", BIT_BANK_SEL_0, REG_WHO_AM_I},
            {"DEVICE_CONFIG       (B0 0x11)", BIT_BANK_SEL_0, REG_DEVICE_CONFIG},
            {"INTF_CONFIG0        (B0 0x4C)", BIT_BANK_SEL_0, REG_INTF_CONFIG0},
            {"INTF_CONFIG1        (B0 0x4D)", BIT_BANK_SEL_0, REG_INTF_CONFIG1},
            {"PWR_MGMT0           (B0 0x4E)", BIT_BANK_SEL_0, REG_PWR_MGMT0},
            {"GYRO_CONFIG0        (B0 0x4F)", BIT_BANK_SEL_0, REG_GYRO_CONFIG0},
            {"ACCEL_CONFIG0       (B0 0x50)", BIT_BANK_SEL_0, REG_ACCEL_CONFIG0},
            {"GYRO_CONFIG1        (B0 0x51)", BIT_BANK_SEL_0, REG_GYRO_CONFIG1},
            {"GYRO_ACCEL_CONFIG0  (B0 0x52)", BIT_BANK_SEL_0, REG_GYRO_ACCEL_CONFIG0},
            {"ACCEL_CONFIG1       (B0 0x53)", BIT_BANK_SEL_0, REG_ACCEL_CONFIG1},
            {"SELF_TEST_CONFIG    (B0 0x70)", BIT_BANK_SEL_0, REG_SELF_TEST_CONFIG},
            {"TEMP_DATA1          (B0 0x1D)", BIT_BANK_SEL_0, REG_TEMP_DATA1},
            {"TEMP_DATA0          (B0 0x1E)", BIT_BANK_SEL_0, REG_TEMP_DATA0},
            {"ACCEL_DATA_X1       (B0 0x1F)", BIT_BANK_SEL_0, REG_ACCEL_DATA_X1},
            {"ACCEL_DATA_X0       (B0 0x20)", BIT_BANK_SEL_0, REG_ACCEL_DATA_X0},
            {"ACCEL_DATA_Y1       (B0 0x21)", BIT_BANK_SEL_0, REG_ACCEL_DATA_Y1},
            {"ACCEL_DATA_Y0       (B0 0x22)", BIT_BANK_SEL_0, REG_ACCEL_DATA_Y0},
            {"ACCEL_DATA_Z1       (B0 0x23)", BIT_BANK_SEL_0, REG_ACCEL_DATA_Z1},
            {"ACCEL_DATA_Z0       (B0 0x24)", BIT_BANK_SEL_0, REG_ACCEL_DATA_Z0},
            {"GYRO_DATA_X1        (B0 0x25)", BIT_BANK_SEL_0, REG_GYRO_DATA_X1},
            {"GYRO_DATA_X0        (B0 0x26)", BIT_BANK_SEL_0, REG_GYRO_DATA_X0},
            {"GYRO_DATA_Y1        (B0 0x27)", BIT_BANK_SEL_0, REG_GYRO_DATA_Y1},
            {"GYRO_DATA_Y0        (B0 0x28)", BIT_BANK_SEL_0, REG_GYRO_DATA_Y0},
            {"GYRO_DATA_Z1        (B0 0x29)", BIT_BANK_SEL_0, REG_GYRO_DATA_Z1},
            {"GYRO_DATA_Z0        (B0 0x2A)", BIT_BANK_SEL_0, REG_GYRO_DATA_Z0},
            {"OFFSET_USER0        (B4 0x77)", BIT_BANK_SEL_4, REG_OFFSET_USER0},
            {"OFFSET_USER1        (B4 0x78)", BIT_BANK_SEL_4, REG_OFFSET_USER1},
            {"OFFSET_USER2        (B4 0x79)", BIT_BANK_SEL_4, REG_OFFSET_USER2},
            {"OFFSET_USER3        (B4 0x7A)", BIT_BANK_SEL_4, REG_OFFSET_USER3},
            {"OFFSET_USER4        (B4 0x7B)", BIT_BANK_SEL_4, REG_OFFSET_USER4},
            {"OFFSET_USER5        (B4 0x7C)", BIT_BANK_SEL_4, REG_OFFSET_USER5},
            {"OFFSET_USER6        (B4 0x7D)", BIT_BANK_SEL_4, REG_OFFSET_USER6},
            {"OFFSET_USER7        (B4 0x7E)", BIT_BANK_SEL_4, REG_OFFSET_USER7},
            {"OFFSET_USER8        (B4 0x7F)", BIT_BANK_SEL_4, REG_OFFSET_USER8},
        };

        shell_print(shell, "IIM-42652 register dump (sensor running, no power-cycle):");
        uint8_t accel_cfg0 = 0xFF;
        for (const auto &e : table) {
            uint8_t v = 0xFF;
            int rc = iim42652_diag_read_reg(dev, e.bank, e.addr, &v);
            if (rc == 0) {
                shell_print(shell, "  %s = 0x%02X", e.name, v);
            } else {
                shell_print(shell, "  %s = read err %d", e.name, rc);
            }
            if (e.bank == BIT_BANK_SEL_0 && e.addr == REG_ACCEL_CONFIG0 && rc == 0) {
                accel_cfg0 = v;
            }
        }

        /* Atomic 6-byte burst from ACCEL_DATA_X1..Z0 so the three axes come
         * from the same sample. Iterating single-byte reads in the table
         * above can split an axis across two ODR ticks at 50 Hz. */
        uint8_t accel_burst[6] = {0};
        int rc_burst = iim42652_diag_read_regs(dev, BIT_BANK_SEL_0,
                                               REG_ACCEL_DATA_X1,
                                               accel_burst, sizeof(accel_burst));
        if (rc_burst != 0) {
            shell_print(shell, "  raw accel burst read err %d", rc_burst);
            return;
        }

        int16_t ax = static_cast<int16_t>((accel_burst[0] << 8) | accel_burst[1]);
        int16_t ay = static_cast<int16_t>((accel_burst[2] << 8) | accel_burst[3]);
        int16_t az = static_cast<int16_t>((accel_burst[4] << 8) | accel_burst[5]);

        /* Decode FSR from ACCEL_CONFIG0[7:5] (BIT_ACCEL_FSR / SHIFT_ACCEL_FS_SEL).
         * The shift values come from DS §3.1 and match the driver's
         * iim42652_accel_sensitivity_shift[] table. */
        const char *fsr_label = "?";
        int lsb_shift = -1;
        if (accel_cfg0 != 0xFF) {
            uint8_t fs_sel = (accel_cfg0 & BIT_ACCEL_FSR) >> SHIFT_ACCEL_FS_SEL;
            switch (fs_sel) {
            case ACCEL_FS_16G: fsr_label = "16g"; lsb_shift = IIM42652_ACCEL_SENS_16G_SHIFT; break;
            case ACCEL_FS_8G:  fsr_label = "8g";  lsb_shift = IIM42652_ACCEL_SENS_8G_SHIFT; break;
            case ACCEL_FS_4G:  fsr_label = "4g";  lsb_shift = IIM42652_ACCEL_SENS_4G_SHIFT; break;
            case ACCEL_FS_2G:  fsr_label = "2g";  lsb_shift = IIM42652_ACCEL_SENS_2G_SHIFT; break;
            }
        }

        if (lsb_shift > 0) {
            float lsb_per_g = static_cast<float>(1u << lsb_shift);
            shell_print(shell,
                "  raw accel burst (FSR=%s, %.0f LSB/g): X=%6d (%.4f g)  Y=%6d (%.4f g)  Z=%6d (%.4f g)",
                fsr_label, lsb_per_g,
                ax, ax / lsb_per_g, ay, ay / lsb_per_g, az, az / lsb_per_g);
        } else {
            shell_print(shell,
                "  raw accel burst (ACCEL_CONFIG0=0x%02X, FSR unknown): X=%6d Y=%6d Z=%6d",
                accel_cfg0, ax, ay, az);
        }
    }

    void info(const shell *shell) const {
        double accel_x = from_fixed<ACCEL_SCALING>(pack_i16t(message.accel_data_upper[0], message.accel_data_lower[0]));
        double accel_y = from_fixed<ACCEL_SCALING>(pack_i16t(message.accel_data_upper[1], message.accel_data_lower[1]));
        double accel_z = from_fixed<ACCEL_SCALING>(pack_i16t(message.accel_data_upper[2], message.accel_data_lower[2]));
        double gyro_x = from_fixed<GYRO_SCALING>(pack_i16t(message.gyro_data_upper[0], message.gyro_data_lower[0]));
        double gyro_y = from_fixed<GYRO_SCALING>(pack_i16t(message.gyro_data_upper[1], message.gyro_data_lower[1]));
        double gyro_z = from_fixed<GYRO_SCALING>(pack_i16t(message.gyro_data_upper[2], message.gyro_data_lower[2]));

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
        return to_fixed<ACCEL_SCALING>(sensor_value_to_float(val));
    }

    int16_t gyro_rad_to_iim42652raw_int16_t(const struct sensor_value *val_rad) {
        return to_fixed<GYRO_SCALING>(RAD_TO_DEG * sensor_value_to_float(val_rad));
    }

    template<float SCALING>
    int16_t to_fixed(float float_value) const
    {
        const int32_t scaled_value = static_cast<int32_t>(std::round(float_value * SCALING));
        const int32_t clamped_value = std::clamp(scaled_value, INT16_MIN, INT16_MAX);
        return static_cast<int16_t>(clamped_value);
    }

    template<float SCALING>
    float from_fixed(int16_t fixed_value) const
    {
        static constexpr float inv_scaling{1.0f / SCALING};

        return fixed_value * inv_scaling;
    }

    int16_t pack_i16t(uint8_t upper, uint8_t lower) const
    {
        return static_cast<int16_t>((static_cast<uint16_t>(upper) << 8) | lower);
    }

    msg message;
    const device *dev{nullptr};
    static constexpr int GYRO_FS_SEL = 1; // 1000DPS iim42652
    static constexpr float RAD_TO_DEG{180.0f / static_cast<float>(M_PI)};
    static constexpr float GYRO_SENSITIVITY[]{16.4f, 32.8f, 65.5f, 131.0f, 262.0f, 524.3f, 1048.6f, 2097.2f};
    static constexpr float GYRO_SCALING{GYRO_SENSITIVITY[GYRO_FS_SEL]};
    static constexpr float ACCEL_SCALING{1000.0f};
} impl;

int info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

int regdump(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    impl.regdump(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(info, NULL, "IMU information", info),
    SHELL_CMD(regdump, NULL, "IIM-42652 register dump (WHO_AM_I, configs, OFFSET_USER, raw ACCEL/GYRO/TEMP)", regdump),
#ifdef LEXXHARD_IMU_CALIBRATION
    SHELL_CMD(calrun,  NULL, "Manual calibration dry-run: compute biases + would-write step values; OFFSET_USER NOT touched",
              lexxhard::imu_calibration::cmd_calrun),
    SHELL_CMD(calinfo, NULL, "Show last calrun result + current OFFSET_USER decode",
              lexxhard::imu_calibration::cmd_calinfo),
#endif
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
