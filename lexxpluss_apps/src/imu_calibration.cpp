/*
 * Copyright (c) 2026, LexxPluss Inc.
 *
 * Manual IMU calibration shell - DRY-RUN ONLY (PR #85).
 *
 * Behaviour summary (see imu_calibration.hpp for concurrency model):
 *   `imu calrun`  - one-shot: drain transients, accumulate N_SAMPLES samples
 *                   off the running IMU fetcher loop, finalize means / stds /
 *                   would-write OFFSET_USER step values, run envelope + motion
 *                   gates, print result. Never writes hardware.
 *   `imu calinfo` - print the last calrun result plus the chip's current
 *                   OFFSET_USER registers decoded into per-axis step / mG / dps.
 *
 * Algorithm is identical to the PACO-verified startup auto-cal v2; only the
 * trigger has changed (boot path -> manual shell). The actual `OFFSET_USER`
 * write moves to a follow-up PR (`imu calrun --write`).
 */
#include "imu_calibration.hpp"

#ifdef LEXXHARD_IMU_CALIBRATION

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "sensor/iim42652/iim42652_reg.h"
#include "sensor/iim42652/iim42652_setup.h"

LOG_MODULE_DECLARE(imu, CONFIG_SENSOR_LOG_LEVEL);

namespace lexxhard::imu_calibration {

namespace {

enum class state_t : int {
    IDLE = 0,
    RUNNING,
    DONE,
    FAILED,
};

constexpr int N_SAMPLES = 200;           /* ~4 s @ 50 Hz, same as v2 */
constexpr int DRAIN_SAMPLES = 5;         /* ~100 ms transient drain */
constexpr int TIMEOUT_MS = 6000;         /* 4 s nominal + slack */
constexpr int POLL_INTERVAL_MS = 50;

constexpr double G_LOCAL = 9.80665;
constexpr double PI_D = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI_D;
constexpr double ACCEL_STEP_MG = 0.5;            /* DS Sec.18, 0.5 mG/step */
constexpr double GYRO_STEP_MDPS = 1000.0 / 32.0; /* 31.25 mdps/step */

/* Gates - same as PACO-verified v2 (~3-5x static noise floor). */
constexpr double ENV_AX_AY_MG_LIMIT = 50.0;
constexpr double ENV_AZ_MG_LIMIT    = 100.0;
constexpr double ENV_GYRO_DPS_LIMIT = 5.0;
constexpr double MOTION_ACCEL_STD_MG = 5.0;
constexpr double MOTION_GYRO_STD_DPS = 0.1;

std::atomic<int> g_state{static_cast<int>(state_t::IDLE)};

struct accumulator_t {
    int drain_remaining;
    int collected;
    double sum_ax, sum_ay, sum_az;
    double sum_ax2, sum_ay2, sum_az2;
    double sum_gx, sum_gy, sum_gz;
    double sum_gx2, sum_gy2, sum_gz2;
};
accumulator_t g_acc{};

struct diag_t {
    bool ever_ran;
    state_t final_state;
    int collected;
    /* sensor frame means + stds */
    double ax_bias_mg, ay_bias_mg, az_bias_mg;
    double gx_bias_dps, gy_bias_dps, gz_bias_dps;
    double az_target_mps2;
    double ax_std_mg, ay_std_mg, az_std_mg;
    double gx_std_dps, gy_std_dps, gz_std_dps;
    /* would-write step values (NOT applied in dry-run) */
    int16_t ax_step, ay_step, az_step;
    int16_t gx_step, gy_step, gz_step;
    /* gate outcomes */
    bool envelope_ok;
    bool motion_ok;
    bool timeout;
    const char *fail_reason;
};
diag_t g_diag{};

double sv_to_d(const struct sensor_value *v) {
    return static_cast<double>(v->val1) + static_cast<double>(v->val2) * 1e-6;
}

int16_t clamp_step(double s) {
    long si = std::lround(s);
    if (si < -2048) si = -2048;
    if (si >  2047) si =  2047;
    return static_cast<int16_t>(si);
}

void finalize() {
    constexpr int N = N_SAMPLES;
    const double ax_mean = g_acc.sum_ax / N;
    const double ay_mean = g_acc.sum_ay / N;
    const double az_mean = g_acc.sum_az / N;
    const double gx_mean = g_acc.sum_gx / N;
    const double gy_mean = g_acc.sum_gy / N;
    const double gz_mean = g_acc.sum_gz / N;

    auto var_of = [](double sum2, double mean, int n) {
        return std::max(0.0, sum2 / n - mean * mean);
    };
    const double ax_std = std::sqrt(var_of(g_acc.sum_ax2, ax_mean, N));
    const double ay_std = std::sqrt(var_of(g_acc.sum_ay2, ay_mean, N));
    const double az_std = std::sqrt(var_of(g_acc.sum_az2, az_mean, N));
    const double gx_std = std::sqrt(var_of(g_acc.sum_gx2, gx_mean, N));
    const double gy_std = std::sqrt(var_of(g_acc.sum_gy2, gy_mean, N));
    const double gz_std = std::sqrt(var_of(g_acc.sum_gz2, gz_mean, N));

    /* az target sign derived from observed gravity, not hardcoded. */
    const double az_target = (az_mean >= 0.0) ? +G_LOCAL : -G_LOCAL;

    g_diag.az_target_mps2 = az_target;
    g_diag.ax_bias_mg  = ax_mean / G_LOCAL * 1000.0;
    g_diag.ay_bias_mg  = ay_mean / G_LOCAL * 1000.0;
    g_diag.az_bias_mg  = (az_mean - az_target) / G_LOCAL * 1000.0;
    g_diag.gx_bias_dps = gx_mean * RAD_TO_DEG;
    g_diag.gy_bias_dps = gy_mean * RAD_TO_DEG;
    g_diag.gz_bias_dps = gz_mean * RAD_TO_DEG;

    g_diag.ax_std_mg = ax_std / G_LOCAL * 1000.0;
    g_diag.ay_std_mg = ay_std / G_LOCAL * 1000.0;
    g_diag.az_std_mg = az_std / G_LOCAL * 1000.0;
    g_diag.gx_std_dps = gx_std * RAD_TO_DEG;
    g_diag.gy_std_dps = gy_std * RAD_TO_DEG;
    g_diag.gz_std_dps = gz_std * RAD_TO_DEG;

    g_diag.ax_step = clamp_step(-g_diag.ax_bias_mg  / ACCEL_STEP_MG);
    g_diag.ay_step = clamp_step(-g_diag.ay_bias_mg  / ACCEL_STEP_MG);
    g_diag.az_step = clamp_step(-g_diag.az_bias_mg  / ACCEL_STEP_MG);
    g_diag.gx_step = clamp_step(-g_diag.gx_bias_dps * 1000.0 / GYRO_STEP_MDPS);
    g_diag.gy_step = clamp_step(-g_diag.gy_bias_dps * 1000.0 / GYRO_STEP_MDPS);
    g_diag.gz_step = clamp_step(-g_diag.gz_bias_dps * 1000.0 / GYRO_STEP_MDPS);

    g_diag.envelope_ok =
        std::fabs(g_diag.ax_bias_mg)  <= ENV_AX_AY_MG_LIMIT &&
        std::fabs(g_diag.ay_bias_mg)  <= ENV_AX_AY_MG_LIMIT &&
        std::fabs(g_diag.az_bias_mg)  <= ENV_AZ_MG_LIMIT    &&
        std::fabs(g_diag.gx_bias_dps) <= ENV_GYRO_DPS_LIMIT &&
        std::fabs(g_diag.gy_bias_dps) <= ENV_GYRO_DPS_LIMIT &&
        std::fabs(g_diag.gz_bias_dps) <= ENV_GYRO_DPS_LIMIT;
    g_diag.motion_ok =
        g_diag.ax_std_mg <= MOTION_ACCEL_STD_MG &&
        g_diag.ay_std_mg <= MOTION_ACCEL_STD_MG &&
        g_diag.az_std_mg <= MOTION_ACCEL_STD_MG &&
        g_diag.gx_std_dps <= MOTION_GYRO_STD_DPS &&
        g_diag.gy_std_dps <= MOTION_GYRO_STD_DPS &&
        g_diag.gz_std_dps <= MOTION_GYRO_STD_DPS;
    g_diag.timeout = false;
    g_diag.collected = g_acc.collected;
    g_diag.ever_ran = true;

    state_t final_state;
    if (g_diag.envelope_ok && g_diag.motion_ok) {
        final_state = state_t::DONE;
        g_diag.fail_reason = nullptr;
    } else {
        final_state = state_t::FAILED;
        g_diag.fail_reason = !g_diag.envelope_ok ? "envelope (bias too large)"
                                                 : "motion (std too large)";
    }
    g_diag.final_state = final_state;

    /* Release store so shell's acquire load sees a fully written g_diag. */
    g_state.store(static_cast<int>(final_state), std::memory_order_release);
}

}  /* anonymous namespace */

void feed_sample(const struct sensor_value accel[3],
                 const struct sensor_value gyro[3]) {
    if (g_state.load(std::memory_order_acquire) != static_cast<int>(state_t::RUNNING)) {
        return;
    }
    if (g_acc.drain_remaining > 0) {
        --g_acc.drain_remaining;
        return;
    }
    if (g_acc.collected >= N_SAMPLES) {
        return;
    }
    const double ax = sv_to_d(&accel[0]);
    const double ay = sv_to_d(&accel[1]);
    const double az = sv_to_d(&accel[2]);
    const double gx = sv_to_d(&gyro[0]);
    const double gy = sv_to_d(&gyro[1]);
    const double gz = sv_to_d(&gyro[2]);
    g_acc.sum_ax += ax; g_acc.sum_ax2 += ax * ax;
    g_acc.sum_ay += ay; g_acc.sum_ay2 += ay * ay;
    g_acc.sum_az += az; g_acc.sum_az2 += az * az;
    g_acc.sum_gx += gx; g_acc.sum_gx2 += gx * gx;
    g_acc.sum_gy += gy; g_acc.sum_gy2 += gy * gy;
    g_acc.sum_gz += gz; g_acc.sum_gz2 += gz * gz;
    if (++g_acc.collected >= N_SAMPLES) {
        finalize();
    }
}

namespace {

void print_diag(const struct shell *shell) {
    if (!g_diag.ever_ran) {
        shell_print(shell, "No calibration run yet. Run `imu calrun`.");
        return;
    }

    const char *state_name =
        g_diag.final_state == state_t::DONE   ? "DONE" :
        g_diag.final_state == state_t::FAILED ? "FAILED" : "?";
    shell_print(shell, "Last calrun: %s (collected %d / %d)",
                state_name, g_diag.collected, N_SAMPLES);
    if (g_diag.final_state != state_t::DONE) {
        shell_print(shell, "  fail reason: %s",
                    g_diag.fail_reason ? g_diag.fail_reason : "(none)");
    }

    shell_print(shell, "[Sensor frame] bias:");
    shell_print(shell, "  accel mG:  ax=%+.2f ay=%+.2f az=%+.2f (az_target=%+.4f m/s^2)",
                g_diag.ax_bias_mg, g_diag.ay_bias_mg, g_diag.az_bias_mg,
                g_diag.az_target_mps2);
    shell_print(shell, "  gyro  dps: gx=%+.4f gy=%+.4f gz=%+.4f",
                g_diag.gx_bias_dps, g_diag.gy_bias_dps, g_diag.gz_bias_dps);
    shell_print(shell, "[Sensor frame] std:");
    shell_print(shell, "  accel mG:  ax=%.3f ay=%.3f az=%.3f",
                g_diag.ax_std_mg, g_diag.ay_std_mg, g_diag.az_std_mg);
    shell_print(shell, "  gyro  dps: gx=%.4f gy=%.4f gz=%.4f",
                g_diag.gx_std_dps, g_diag.gy_std_dps, g_diag.gz_std_dps);

    /* ROS-frame view: applies the firmware CAN transform (X<->Y swap +
     * negate-all) AND the SCBDriver receiver's extra accel.y flip, so the
     * numbers match what `/driver/internal/sensor_set/imu` publishes. */
    const double ros_ax = -g_diag.ay_bias_mg;
    const double ros_ay = +g_diag.ax_bias_mg;
    const double ros_az = -g_diag.az_bias_mg;
    const double ros_gx = -g_diag.gy_bias_dps;
    const double ros_gy = -g_diag.gx_bias_dps;
    const double ros_gz = -g_diag.gz_bias_dps;
    shell_print(shell, "[ROS frame, /driver/internal/sensor_set/imu] bias:");
    shell_print(shell, "  accel mG:  x=%+.2f y=%+.2f z=%+.2f", ros_ax, ros_ay, ros_az);
    shell_print(shell, "  gyro  dps: x=%+.4f y=%+.4f z=%+.4f", ros_gx, ros_gy, ros_gz);

    shell_print(shell, "Would-write OFFSET_USER step (DRY-RUN, NOT applied):");
    shell_print(shell, "  gyro:  gx=%+5d gy=%+5d gz=%+5d",
                g_diag.gx_step, g_diag.gy_step, g_diag.gz_step);
    shell_print(shell,
                "  accel: ax=%+5d ay=%+5d az=%+5d  (ax/ay would NOT be written "
                "even with --write; six-face test pending)",
                g_diag.ax_step, g_diag.ay_step, g_diag.az_step);

    shell_print(shell, "Gates: envelope=%s motion=%s",
                g_diag.envelope_ok ? "OK" : "FAIL",
                g_diag.motion_ok   ? "OK" : "FAIL");
}

void print_current_offset(const struct shell *shell) {
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(imu0));
    if (!device_is_ready(dev)) {
        shell_warn(shell, "Cannot read current OFFSET_USER: IMU device not ready");
        return;
    }
    uint8_t buf[9];
    int rc = iim42652_diag_read_regs(dev, BIT_BANK_SEL_4, REG_OFFSET_USER0, buf, 9);
    if (rc) {
        shell_warn(shell, "Read OFFSET_USER0..8 failed rc=%d", rc);
        return;
    }
    auto unpack12 = [](uint8_t lo_byte, uint8_t high_nibble) -> int16_t {
        uint16_t u = (static_cast<uint16_t>(high_nibble & 0x0F) << 8) | lo_byte;
        if (u & 0x800) {
            u |= 0xF000;  /* sign-extend 12-bit -> 16-bit */
        }
        return static_cast<int16_t>(u);
    };
    /* DS Sec.18 OFFSET_USER packing (verified on PACO 2026-05-15 with
     * az=-56 step -> bytes 00,00,00,00,00,00,00,F0,C8). */
    const int16_t gx = unpack12(buf[0], (buf[1] >> 4) & 0x0F);
    const int16_t gy = unpack12(buf[2],  buf[1]       & 0x0F);
    const int16_t gz = unpack12(buf[3], (buf[4] >> 4) & 0x0F);
    const int16_t ax = unpack12(buf[5],  buf[4]       & 0x0F);
    const int16_t ay = unpack12(buf[6], (buf[7] >> 4) & 0x0F);
    const int16_t az = unpack12(buf[8],  buf[7]       & 0x0F);

    shell_print(shell, "Current OFFSET_USER (raw bytes 0..8):");
    shell_print(shell, "  %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                buf[0], buf[1], buf[2], buf[3], buf[4],
                buf[5], buf[6], buf[7], buf[8]);
    shell_print(shell, "Current OFFSET_USER (decoded, sensor frame):");
    shell_print(shell,
                "  gyro:  gx=%+5d (%+.3f dps)  gy=%+5d (%+.3f dps)  gz=%+5d (%+.3f dps)",
                gx, gx * (GYRO_STEP_MDPS / 1000.0),
                gy, gy * (GYRO_STEP_MDPS / 1000.0),
                gz, gz * (GYRO_STEP_MDPS / 1000.0));
    shell_print(shell,
                "  accel: ax=%+5d (%+.2f mG)  ay=%+5d (%+.2f mG)  az=%+5d (%+.2f mG)",
                ax, ax * ACCEL_STEP_MG,
                ay, ay * ACCEL_STEP_MG,
                az, az * ACCEL_STEP_MG);
}

}  /* anonymous namespace */

int cmd_calrun(const struct shell *shell, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    /* Accept restart from IDLE / DONE / FAILED; reject if RUNNING. */
    auto try_acquire = []() -> bool {
        for (auto from : {state_t::IDLE, state_t::DONE, state_t::FAILED}) {
            int expected = static_cast<int>(from);
            if (g_state.compare_exchange_strong(
                    expected,
                    static_cast<int>(state_t::RUNNING),
                    std::memory_order_acq_rel)) {
                return true;
            }
        }
        return false;
    };
    if (!try_acquire()) {
        shell_error(shell, "Calibration already running. Wait for it to finish.");
        return 1;
    }

    /* Reset accumulator. State has just transitioned to RUNNING, so the
     * fetcher loop may already pass the gate; the first DRAIN_SAMPLES are
     * discarded anyway. */
    g_acc = accumulator_t{};
    g_acc.drain_remaining = DRAIN_SAMPLES;

    shell_print(shell,
                "calrun: dry-run, collecting %d samples (~%d s @ 50 Hz). Keep robot static.",
                N_SAMPLES, (N_SAMPLES + DRAIN_SAMPLES) / 50);

    const int64_t deadline = k_uptime_get() + TIMEOUT_MS;
    while (k_uptime_get() < deadline) {
        int s = g_state.load(std::memory_order_acquire);
        if (s == static_cast<int>(state_t::DONE) ||
            s == static_cast<int>(state_t::FAILED)) {
            break;
        }
        k_msleep(POLL_INTERVAL_MS);
    }

    if (g_state.load(std::memory_order_acquire) == static_cast<int>(state_t::RUNNING)) {
        /* Timed out. Mark diag and transition to FAILED so future calls retry. */
        g_diag.ever_ran = true;
        g_diag.timeout = true;
        g_diag.envelope_ok = false;
        g_diag.motion_ok = false;
        g_diag.final_state = state_t::FAILED;
        g_diag.collected = g_acc.collected;
        g_diag.fail_reason = "timeout (insufficient samples)";
        g_state.store(static_cast<int>(state_t::FAILED), std::memory_order_release);
    }

    print_diag(shell);
    return g_diag.final_state == state_t::DONE ? 0 : 1;
}

int cmd_calinfo(const struct shell *shell, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    print_diag(shell);
    shell_print(shell, "");
    print_current_offset(shell);
    return 0;
}

}  /* namespace lexxhard::imu_calibration */

#endif  /* LEXXHARD_IMU_CALIBRATION */
