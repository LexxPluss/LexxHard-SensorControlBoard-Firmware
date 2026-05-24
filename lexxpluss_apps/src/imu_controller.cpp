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

        // Startup auto-calibration: collect 200 static samples, derive accel
        // and gyro biases, then write OFFSET_USER. See plan §3.3. Best-effort:
        // any failure logs the reason and leaves OFFSET_USER=0 (uncalibrated)
        // so the rest of the boot still proceeds.
        run_auto_calibration();

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
    /* Startup auto-calibration (plan §3.3) — v2.
     *
     * Assumptions:
     *   - Robot is static and roughly horizontal at boot.
     *   - One axis (Z by current PACO/V7 assembly) carries gravity; X/Y near 0 g.
     *
     * Per [[feedback-no-physical-orientation-from-data]] we do NOT hardcode
     * which sensor axis is "up". `az_target = sign(az_mean) * G_LOCAL` is
     * derived from observation rather than asserted from assembly drawings.
     *
     * Sign convention (verified):
     *   sensor_output = sensor_raw + OFFUSER
     *   - ACCEL_Z: verified plan §3.1, PACO v71es007 2026-05-14
     *   - GYRO_X/Y/Z: verified on same hardware 2026-05-15
     *     (+320 OFFUSER step → ~+320 LSB raw shift on each gyro axis)
     * To cancel bias we write step = -bias (in step units).
     *
     * v2 scope:
     *   - Writes ACCEL_Z + all three GYRO axes.
     *   - ACCEL_X/Y still left at 0: what looks like static X/Y bias is
     *     dominated by mechanical mounting tilt vs. true horizontal, and
     *     writing that into OFFSET_USER would entrench tilt as if it were
     *     a sensor error. A six-face test (plan §4.3) is needed to separate
     *     zero-g offset from mounting tilt before enabling ax/ay auto-cal.
     *   - "Would-have-written" ax/ay step values are still logged for future
     *     comparison once tilt is characterized.
     *
     * Fail-safes (any one triggers → keep OFFSET_USER=0 and continue boot):
     *   - Sample timeout: < 200 / N samples within 6 s
     *   - Bias envelope:  |ax/ay| < 50 mG, |az_resid| < 100 mG, |gyro| < 5 dps
     *   - Motion gate:    per-axis std checks (catches vibration / movement
     *                     during boot that mean-only would average out)
     */
    void run_auto_calibration() {
        // Sensor was just turned on by sensor_trigger_set() → iim42652_turn_on_sensor(),
        // which already does k_msleep(100). The extra wait below drains any
        // transient samples that pre-date a clean static reading.
        LOG_INF("auto-cal: 100 ms transient drain");
        k_msleep(100);
        int_flag = false;

        static constexpr int N = 200;
        static constexpr int TIMEOUT_MS = 6000;  /* 4s nominal + slack */
        double sum_ax = 0, sum_ay = 0, sum_az = 0;
        double sum_gx = 0, sum_gy = 0, sum_gz = 0;
        double sum_ax2 = 0, sum_ay2 = 0, sum_az2 = 0;
        double sum_gx2 = 0, sum_gy2 = 0, sum_gz2 = 0;
        int collected = 0;
        int waited_ms = 0;
        LOG_INF("auto-cal: collecting %d samples (~4 s @ 50 Hz)", N);
        while (collected < N) {
            if (waited_ms > TIMEOUT_MS) {
                LOG_ERR("auto-cal: timeout after %d / %d samples; keeping OFFSET=0",
                        collected, N);
                return;
            }
            if (int_flag) {
                if (sensor_sample_fetch(dev) == 0) {
                    struct sensor_value a[3], g[3];
                    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, a);
                    sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, g);
                    const double ax = sensor_value_to_float(&a[0]);
                    const double ay = sensor_value_to_float(&a[1]);
                    const double az = sensor_value_to_float(&a[2]);
                    const double gx = sensor_value_to_float(&g[0]);
                    const double gy = sensor_value_to_float(&g[1]);
                    const double gz = sensor_value_to_float(&g[2]);
                    sum_ax += ax; sum_ax2 += ax * ax;
                    sum_ay += ay; sum_ay2 += ay * ay;
                    sum_az += az; sum_az2 += az * az;
                    sum_gx += gx; sum_gx2 += gx * gx;
                    sum_gy += gy; sum_gy2 += gy * gy;
                    sum_gz += gz; sum_gz2 += gz * gz;
                    collected++;
                }
                int_flag = false;
            }
            k_msleep(1);
            waited_ms++;
        }

        const double ax_mean = sum_ax / N;            // m/s², sensor frame
        const double ay_mean = sum_ay / N;
        const double az_mean = sum_az / N;
        const double gx_mean_rad = sum_gx / N;        // rad/s, sensor frame
        const double gy_mean_rad = sum_gy / N;
        const double gz_mean_rad = sum_gz / N;

        /* Biased variance is fine for N=200 + a static envelope check. Clamp
         * to >=0 to guard against tiny negative values from FP cancellation. */
        auto var_of = [](double sum2, double mean, int n) {
            return std::max(0.0, sum2 / n - mean * mean);
        };
        const double ax_std = std::sqrt(var_of(sum_ax2, ax_mean, N));  // m/s²
        const double ay_std = std::sqrt(var_of(sum_ay2, ay_mean, N));
        const double az_std = std::sqrt(var_of(sum_az2, az_mean, N));
        const double gx_std_rad = std::sqrt(var_of(sum_gx2, gx_mean_rad, N));  // rad/s
        const double gy_std_rad = std::sqrt(var_of(sum_gy2, gy_mean_rad, N));
        const double gz_std_rad = std::sqrt(var_of(sum_gz2, gz_mean_rad, N));

        constexpr double G_LOCAL = 9.80665;
        const double az_target = (az_mean >= 0.0) ? +G_LOCAL : -G_LOCAL;

        const double ax_bias_mg  = ax_mean / G_LOCAL * 1000.0;
        const double ay_bias_mg  = ay_mean / G_LOCAL * 1000.0;
        const double az_bias_mg  = (az_mean - az_target) / G_LOCAL * 1000.0;
        const double gx_bias_dps = gx_mean_rad * (180.0 / M_PI);
        const double gy_bias_dps = gy_mean_rad * (180.0 / M_PI);
        const double gz_bias_dps = gz_mean_rad * (180.0 / M_PI);

        const double ax_std_mg  = ax_std / G_LOCAL * 1000.0;
        const double ay_std_mg  = ay_std / G_LOCAL * 1000.0;
        const double az_std_mg  = az_std / G_LOCAL * 1000.0;
        const double gx_std_dps = gx_std_rad * (180.0 / M_PI);
        const double gy_std_dps = gy_std_rad * (180.0 / M_PI);
        const double gz_std_dps = gz_std_rad * (180.0 / M_PI);

        LOG_INF("auto-cal: accel bias mG ax=%.2f ay=%.2f az=%.2f; std mG ax=%.2f ay=%.2f az=%.2f",
                ax_bias_mg, ay_bias_mg, az_bias_mg,
                ax_std_mg, ay_std_mg, az_std_mg);
        LOG_INF("auto-cal: gyro bias dps gx=%.3f gy=%.3f gz=%.3f; std dps gx=%.3f gy=%.3f gz=%.3f",
                gx_bias_dps, gy_bias_dps, gz_bias_dps,
                gx_std_dps, gy_std_dps, gz_std_dps);

        /* Envelope gate (bias). Rejects tilt, half-stuck assembly, dangling
         * IMU during boot, etc. */
        constexpr double CAL_MAX_AX_AY_MG = 50.0;
        constexpr double CAL_MAX_AZ_MG    = 100.0;
        constexpr double CAL_MAX_G_DPS    = 5.0;
        if (std::fabs(ax_bias_mg)  > CAL_MAX_AX_AY_MG ||
            std::fabs(ay_bias_mg)  > CAL_MAX_AX_AY_MG ||
            std::fabs(az_bias_mg)  > CAL_MAX_AZ_MG    ||
            std::fabs(gx_bias_dps) > CAL_MAX_G_DPS    ||
            std::fabs(gy_bias_dps) > CAL_MAX_G_DPS    ||
            std::fabs(gz_bias_dps) > CAL_MAX_G_DPS) {
            LOG_ERR("auto-cal: bias outside envelope (|ax/ay|<%.0fmG, |az|<%.0fmG, |g*|<%.1fdps); skipping",
                    CAL_MAX_AX_AY_MG, CAL_MAX_AZ_MG, CAL_MAX_G_DPS);
            return;
        }

        /* Motion gate (std). Rejects vibration / movement during boot that
         * the bias envelope alone cannot detect (mean stays near zero even
         * with large oscillations). Thresholds set ~3-5× the static noise
         * floor observed on PACO/V7 (accel std ≈ 1.2 mG, gyro std ≈ 0.04 dps). */
        constexpr double MOTION_MAX_ACCEL_STD_MG = 5.0;
        constexpr double MOTION_MAX_GYRO_STD_DPS = 0.1;
        if (ax_std_mg > MOTION_MAX_ACCEL_STD_MG ||
            ay_std_mg > MOTION_MAX_ACCEL_STD_MG ||
            az_std_mg > MOTION_MAX_ACCEL_STD_MG ||
            gx_std_dps > MOTION_MAX_GYRO_STD_DPS ||
            gy_std_dps > MOTION_MAX_GYRO_STD_DPS ||
            gz_std_dps > MOTION_MAX_GYRO_STD_DPS) {
            LOG_ERR("auto-cal: motion detected (accel std > %.1fmG or gyro std > %.2fdps); skipping",
                    MOTION_MAX_ACCEL_STD_MG, MOTION_MAX_GYRO_STD_DPS);
            return;
        }

        constexpr double ACCEL_STEP_MG  = 0.5;        /* DS §18 */
        constexpr double GYRO_STEP_MDPS = 1000.0 / 32.0;  /* 31.25 mdps/step */
        auto clamp_step = [](double s) -> int16_t {
            long si = std::lround(s);
            if (si < -2048) si = -2048;
            if (si >  2047) si =  2047;
            return static_cast<int16_t>(si);
        };
        const int16_t az_step = clamp_step(-az_bias_mg  / ACCEL_STEP_MG);
        const int16_t gx_step = clamp_step(-gx_bias_dps * 1000.0 / GYRO_STEP_MDPS);
        const int16_t gy_step = clamp_step(-gy_bias_dps * 1000.0 / GYRO_STEP_MDPS);
        const int16_t gz_step = clamp_step(-gz_bias_dps * 1000.0 / GYRO_STEP_MDPS);

        /* Logged for future comparison once mounting tilt is separated from
         * sensor zero-g offset via six-face test. Not written this iteration. */
        const int16_t ax_step_pending = clamp_step(-ax_bias_mg  / ACCEL_STEP_MG);
        const int16_t ay_step_pending = clamp_step(-ay_bias_mg  / ACCEL_STEP_MG);

        LOG_INF("auto-cal: writing gyro=(%d,%d,%d) az=%d (v2: ax/ay still 0, six-face test needed)",
                gx_step, gy_step, gz_step, az_step);
        LOG_INF("auto-cal: pending (not written) ax=%d ay=%d",
                ax_step_pending, ay_step_pending);

        int rc = iim42652_set_offset_user(dev,
                                          gx_step, gy_step, gz_step,  /* gyro: sign verified */
                                          0, 0, az_step);             /* accel: Z only */
        if (rc) {
            LOG_ERR("auto-cal: set_offset_user failed rc=%d (OFFSET state uncertain)", rc);
            return;
        }
        LOG_INF("auto-cal: applied (gyro X/Y/Z + ACCEL_Z)");

        /* set_offset_user briefly powered the sensor down; the trigger may
         * have queued an IRQ from the post-restore state. Reset so the
         * main loop's first fetch is a deliberate fresh one. */
        int_flag = false;
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
