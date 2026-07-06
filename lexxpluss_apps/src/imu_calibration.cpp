/*
 * Copyright (c) 2026, LexxPluss Inc.
 *
 * Manual IMU calibration shell (PR #86).
 *
 * Behaviour summary (see imu_calibration.hpp for concurrency model):
 *   `imu calrun`         - dry-run: drain transients, accumulate N_SAMPLES
 *                          samples off the running IMU fetcher loop, finalize
 *                          means / stds / would-write OFFSET_USER step values,
 *                          run envelope + motion gates, print result. Never
 *                          writes hardware.
 *   `imu calrun --write` - same collection, then (only if both gates pass)
 *                          writes GYRO_X/Y/Z + ACCEL_Z OFFSET_USER. ACCEL_X/Y
 *                          are read-merged (current register values preserved,
 *                          NOT overwritten and NOT forced to 0) because a
 *                          single static pose cannot separate sensor zero-g
 *                          offset from mounting tilt — six-face test pending.
 *   `imu calinfo`        - print the last calrun result plus the chip's current
 *                          OFFSET_USER registers decoded into step / mG / dps.
 *
 * Collection algorithm is identical to the PACO-verified startup auto-cal v2;
 * only the trigger changed (boot path -> manual shell). The write path reuses
 * the driver's iim42652_set_offset_user(), verified on PACO 2026-05-15.
 *
 * Safety invariants for --write:
 *   - default (no arg) never writes;
 *   - only `imu calrun --write` (exactly) requests a write; any other arg is
 *     rejected before sampling starts;
 *   - write happens only when final_state == DONE (envelope + motion gates
 *     both pass); gate fail / timeout => no write;
 *   - if reading the current OFFSET_USER (for ACCEL_X/Y merge) fails, --write
 *     aborts without writing — it never degrades to ax/ay = 0.
 */
#include "imu_calibration.hpp"

#ifdef LEXXHARD_IMU_CALIBRATION

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

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

/* State machine, with two intermediate states to make all shared-data
 * writes single-writer:
 *
 *   IDLE / DONE / FAILED   - terminal-ish; shell may start a new run
 *
 *      shell CAS {IDLE,DONE,FAILED} -> STARTING
 *      shell exclusively owns g_acc + g_diag (clears both)
 *      shell release-store STARTING -> RUNNING
 *
 *   RUNNING                - fetcher exclusively writes g_acc; nobody
 *                            writes g_diag
 *
 *      whoever CAS-wins RUNNING -> FINALIZING owns g_diag exclusively:
 *        - fetcher hits N samples in feed_sample() -> finalize()
 *        - shell timeout in cmd_calrun()
 *      Winner writes g_diag, then release-store FINALIZING -> DONE/FAILED.
 *      Loser of the CAS does nothing.
 *
 *   FINALIZING             - exclusive g_diag write phase; readers must
 *                            spin until DONE/FAILED.
 *
 * feed_sample() only acts when state == RUNNING. It ignores STARTING and
 * FINALIZING so the shell side has uncontended access to g_acc / g_diag
 * during those phases.
 */
enum class state_t : int {
    IDLE = 0,
    STARTING,
    RUNNING,
    FINALIZING,
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
    int64_t t_run_start_ms;    /* shell sets at RUNNING store */
    int64_t t_first_sample_ms; /* fetcher sets on first post-drain sample */
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
    /* timing (ms); -1 = not measured for this run */
    int64_t t_collect_done_ms; /* finalize() timestamp, for shell poll_lag calc */
    int drain_ms;
    int sample_ms;
    int collect_total_ms;
};
diag_t g_diag{};

double sv_to_d(const struct sensor_value *v) {
    return static_cast<double>(v->val1) + static_cast<double>(v->val2) * 1e-6;
}

/* DS Sec.18 OFFSET_USER 12-bit signed unpack (shared by readback + ax/ay merge). */
int16_t unpack12(uint8_t lo_byte, uint8_t high_nibble) {
    uint16_t u = (static_cast<uint16_t>(high_nibble & 0x0F) << 8) | lo_byte;
    if (u & 0x800) {
        u |= 0xF000;  /* sign-extend 12-bit -> 16-bit */
    }
    return static_cast<int16_t>(u);
}

struct offset_steps_t {
    int16_t gx, gy, gz, ax, ay, az;
};

/* Decode the 9-byte OFFSET_USER0..8 image into six 12-bit signed steps.
 * MUST mirror iim42652_set_offset_user()'s packing exactly (the encoder is
 * the single source of truth):
 *   USER1 = GY[11:8]<<4 | GX[11:8]   (high nibble = GY, low nibble = GX)
 *   USER4 = AX[11:8]<<4 | GZ[11:8]   (high nibble = AX, low nibble = GZ)
 *   USER7 = AZ[11:8]<<4 | AY[11:8]   (high nibble = AZ, low nibble = AY)
 * Cross-check: PACO 2026-05-15 az=-56 step (0xFC8) -> bytes ...,F0,C8, i.e.
 * AZ[11:8]=0xF lives in USER7's HIGH nibble -> az = unpack12(buf[8], buf[7]>>4). */
offset_steps_t decode_offset_user(const uint8_t buf[9]) {
    return offset_steps_t{
        /* gx */ unpack12(buf[0],  buf[1]       & 0x0F),
        /* gy */ unpack12(buf[2], (buf[1] >> 4) & 0x0F),
        /* gz */ unpack12(buf[3],  buf[4]       & 0x0F),
        /* ax */ unpack12(buf[5], (buf[4] >> 4) & 0x0F),
        /* ay */ unpack12(buf[6],  buf[7]       & 0x0F),
        /* az */ unpack12(buf[8], (buf[7] >> 4) & 0x0F),
    };
}

int16_t clamp_step(double s) {
    long si = std::lround(s);
    if (si < -2048) si = -2048;
    if (si >  2047) si =  2047;
    return static_cast<int16_t>(si);
}

void finalize() {
    /* Claim exclusive g_diag write right via RUNNING -> FINALIZING CAS.
     * If the shell's timeout path won the race we bail; whatever it wrote
     * (or zeroed) into g_diag is now authoritative. */
    int expected = static_cast<int>(state_t::RUNNING);
    if (!g_state.compare_exchange_strong(
            expected,
            static_cast<int>(state_t::FINALIZING),
            std::memory_order_acq_rel)) {
        return;
    }

    const int64_t t_collect_done = k_uptime_get();

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

    g_diag.t_collect_done_ms = t_collect_done;
    g_diag.drain_ms = static_cast<int>(g_acc.t_first_sample_ms - g_acc.t_run_start_ms);
    g_diag.sample_ms = static_cast<int>(t_collect_done - g_acc.t_first_sample_ms);
    g_diag.collect_total_ms = static_cast<int>(t_collect_done - g_acc.t_run_start_ms);

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
    if (g_acc.collected == 0) {
        g_acc.t_first_sample_ms = k_uptime_get();  /* first post-drain sample */
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

    shell_print(shell, "Correction step (DRY-RUN; --write ADDS this to current OFFSET_USER):");
    shell_print(shell, "  gyro:  gx=%+5d gy=%+5d gz=%+5d",
                g_diag.gx_step, g_diag.gy_step, g_diag.gz_step);
    shell_print(shell,
                "  accel: ax=%+5d ay=%+5d az=%+5d  (ax/ay correction NOT applied "
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
    const offset_steps_t o = decode_offset_user(buf);

    shell_print(shell, "Current OFFSET_USER (raw bytes 0..8):");
    shell_print(shell, "  %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                buf[0], buf[1], buf[2], buf[3], buf[4],
                buf[5], buf[6], buf[7], buf[8]);
    shell_print(shell, "Current OFFSET_USER (decoded, sensor frame):");
    shell_print(shell,
                "  gyro:  gx=%+5d (%+.3f dps)  gy=%+5d (%+.3f dps)  gz=%+5d (%+.3f dps)",
                o.gx, o.gx * (GYRO_STEP_MDPS / 1000.0),
                o.gy, o.gy * (GYRO_STEP_MDPS / 1000.0),
                o.gz, o.gz * (GYRO_STEP_MDPS / 1000.0));
    shell_print(shell,
                "  accel: ax=%+5d (%+.2f mG)  ay=%+5d (%+.2f mG)  az=%+5d (%+.2f mG)",
                o.ax, o.ax * ACCEL_STEP_MG,
                o.ay, o.ay * ACCEL_STEP_MG,
                o.az, o.az * ACCEL_STEP_MG);
}

/* Apply the last calrun's correction ON TOP OF the current OFFSET_USER.
 *
 * g_diag.*_step is a CORRECTION (= -measured_output_bias), not an absolute
 * target. Because OFFSET_USER is applied in hardware before the data
 * registers, calrun always measures the already-compensated output, so the
 * correction must accumulate:
 *     final = current + correction        (gx/gy/gz/az)
 *     final = current                     (ax/ay preserved, correction shown
 *                                          but not applied; six-face pending)
 * This makes repeated --write convergent (a second run sees ~0 residual ->
 * ~0 correction -> stays put) instead of overwriting good compensation.
 *
 * Returns 0 on success, negative on failure (nothing written). Aborts before
 * writing if the current-OFFSET read fails or any final gx/gy/gz/az would
 * exceed the 12-bit signed range [-2048, 2047] (no silent clamp). write_ms is
 * the wall time of the set_offset_user() call only. */
int apply_offset_write(const struct shell *shell, int *write_ms) {
    *write_ms = -1;
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(imu0));
    if (!device_is_ready(dev)) {
        shell_error(shell, "--write aborted: IMU device not ready (nothing written)");
        return -ENODEV;
    }
    /* Read current OFFSET_USER so we can accumulate onto it and preserve
     * ACCEL_X/Y. If this read fails we abort rather than guess. */
    uint8_t buf[9];
    int rc = iim42652_diag_read_regs(dev, BIT_BANK_SEL_4, REG_OFFSET_USER0, buf, 9);
    if (rc) {
        shell_error(shell,
                    "--write aborted: read current OFFSET_USER failed rc=%d "
                    "(nothing written)", rc);
        return rc;
    }
    const offset_steps_t cur = decode_offset_user(buf);

    /* final = current + correction for gx/gy/gz/az; ax/ay held at current. */
    const int final_gx = cur.gx + g_diag.gx_step;
    const int final_gy = cur.gy + g_diag.gy_step;
    const int final_gz = cur.gz + g_diag.gz_step;
    const int final_az = cur.az + g_diag.az_step;
    const int16_t final_ax = cur.ax;
    const int16_t final_ay = cur.ay;

    /* Range check the accumulated values; abort (no silent clamp) if any
     * gx/gy/gz/az would saturate the 12-bit signed OFFSET_USER field. */
    auto out_of_range = [](int v) { return v < -2048 || v > 2047; };
    if (out_of_range(final_gx) || out_of_range(final_gy) ||
        out_of_range(final_gz) || out_of_range(final_az)) {
        shell_error(shell,
                    "--write aborted: accumulated OFFSET out of [-2048,2047] "
                    "(final gx=%d gy=%d gz=%d az=%d); nothing written",
                    final_gx, final_gy, final_gz, final_az);
        return -ERANGE;
    }

    shell_print(shell, "OFFSET_USER write:");
    shell_print(shell, "  current:    gx=%+d gy=%+d gz=%+d ax=%+d ay=%+d az=%+d",
                cur.gx, cur.gy, cur.gz, cur.ax, cur.ay, cur.az);
    shell_print(shell,
                "  correction: gx=%+d gy=%+d gz=%+d ax=%+d ay=%+d az=%+d  "
                "(ax/ay correction NOT applied; six-face pending)",
                g_diag.gx_step, g_diag.gy_step, g_diag.gz_step,
                g_diag.ax_step, g_diag.ay_step, g_diag.az_step);
    shell_print(shell, "  final:      gx=%+d gy=%+d gz=%+d ax=%+d ay=%+d az=%+d",
                final_gx, final_gy, final_gz, final_ax, final_ay, final_az);

    const int64_t tw0 = k_uptime_get();
    rc = iim42652_set_offset_user(dev,
                                  static_cast<int16_t>(final_gx),
                                  static_cast<int16_t>(final_gy),
                                  static_cast<int16_t>(final_gz),
                                  final_ax, final_ay,
                                  static_cast<int16_t>(final_az));
    *write_ms = static_cast<int>(k_uptime_get() - tw0);
    if (rc) {
        shell_error(shell, "OFFSET_USER write failed rc=%d", rc);
        return rc;
    }
    return 0;
}

}  /* anonymous namespace */

int cmd_calrun(const struct shell *shell, size_t argc, char **argv) {
    /* Strict arg parse BEFORE any state change: only `imu calrun` or
     * `imu calrun --write`. Reject anything else without starting sampling. */
    bool want_write = false;
    if (argc == 2 && std::strcmp(argv[1], "--write") == 0) {
        want_write = true;
    } else if (argc != 1) {
        shell_error(shell, "usage: imu calrun [--write]");
        return -EINVAL;
    }

    /* Acquire phase: CAS {IDLE,DONE,FAILED} -> STARTING. While STARTING,
     * feed_sample() bails out, so we have exclusive access to g_acc and
     * g_diag and can reset them safely. */
    auto try_acquire_starting = []() -> bool {
        for (auto from : {state_t::IDLE, state_t::DONE, state_t::FAILED}) {
            int expected = static_cast<int>(from);
            if (g_state.compare_exchange_strong(
                    expected,
                    static_cast<int>(state_t::STARTING),
                    std::memory_order_acq_rel)) {
                return true;
            }
        }
        return false;
    };
    if (!try_acquire_starting()) {
        shell_error(shell, "Calibration already running. Wait for it to finish.");
        return 1;
    }

    /* Single-writer phase: nobody else touches g_acc or g_diag. */
    g_acc = accumulator_t{};
    g_acc.drain_remaining = DRAIN_SAMPLES;
    g_acc.t_run_start_ms = k_uptime_get();
    g_diag = diag_t{};  /* clear stale stats from previous run */

    /* Release-store STARTING -> RUNNING. The fetcher's next acquire-load
     * sees both the new state AND the cleared accumulator. */
    g_state.store(static_cast<int>(state_t::RUNNING), std::memory_order_release);

    shell_print(shell,
                "calrun: %s, collecting %d samples (~%d s @ 50 Hz). Keep robot static.",
                want_write ? "WILL WRITE on gate pass" : "dry-run",
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

    /* Timeout path. Try to win RUNNING -> FINALIZING; only the winner
     * gets to write g_diag. If fetcher just finalized concurrently,
     * we lose the CAS and its g_diag is authoritative. */
    {
        int expected = static_cast<int>(state_t::RUNNING);
        if (g_state.compare_exchange_strong(
                expected,
                static_cast<int>(state_t::FINALIZING),
                std::memory_order_acq_rel)) {
            g_diag.ever_ran = true;
            g_diag.timeout = true;
            g_diag.envelope_ok = false;
            g_diag.motion_ok = false;
            g_diag.final_state = state_t::FAILED;
            g_diag.collected = g_acc.collected;
            g_diag.fail_reason = "timeout (insufficient samples)";
            g_state.store(static_cast<int>(state_t::FAILED), std::memory_order_release);
        } else {
            /* Fetcher won the race. Wait briefly for it to publish DONE/FAILED. */
            while (g_state.load(std::memory_order_acquire) ==
                   static_cast<int>(state_t::FINALIZING)) {
                k_yield();
            }
        }
    }

    const int64_t t_observed = k_uptime_get();
    print_diag(shell);

    /* --write: only on a clean DONE (both gates passed). gate fail / timeout
     * => skip, never write. */
    int write_ms = -1;
    int write_rc = 0;
    if (want_write) {
        if (g_diag.final_state == state_t::DONE) {
            write_rc = apply_offset_write(shell, &write_ms);
            shell_print(shell, "");
            print_current_offset(shell);
        } else {
            shell_print(shell, "--write skipped: gates did not pass (nothing written)");
        }
    }

    /* Timing block. poll_lag = shell wakeup latency after finalize; kept
     * separate from print time. total spans run-start to end of write/print. */
    const int64_t t_end = k_uptime_get();
    if (g_diag.timeout) {
        shell_print(shell, "calrun timing: timed out after %d ms (no valid collection)",
                    static_cast<int>(t_end - g_acc.t_run_start_ms));
    } else {
        const int poll_lag_ms = static_cast<int>(t_observed - g_diag.t_collect_done_ms);
        const int total_ms = static_cast<int>(t_end - g_acc.t_run_start_ms);
        if (write_ms >= 0) {
            shell_print(shell,
                        "calrun timing: drain_ms=%d sample_ms=%d collect_total_ms=%d "
                        "poll_lag_ms=%d write_ms=%d total_ms=%d",
                        g_diag.drain_ms, g_diag.sample_ms, g_diag.collect_total_ms,
                        poll_lag_ms, write_ms, total_ms);
        } else {
            shell_print(shell,
                        "calrun timing: drain_ms=%d sample_ms=%d collect_total_ms=%d "
                        "poll_lag_ms=%d write_ms=n/a total_ms=%d",
                        g_diag.drain_ms, g_diag.sample_ms, g_diag.collect_total_ms,
                        poll_lag_ms, total_ms);
        }
    }

    /* Calibration gate failure (or timeout) => non-zero. A clean cal that then
     * failed to write (--write) must also surface as non-zero so the shell
     * layer does not treat "collected OK but write failed" as success. */
    if (g_diag.final_state != state_t::DONE) {
        return 1;
    }
    return write_rc;  /* 0 on dry-run / successful write, non-zero on write failure */
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
