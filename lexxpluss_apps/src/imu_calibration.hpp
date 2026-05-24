/*
 * Copyright (c) 2026, LexxPluss Inc.
 *
 * Manual IMU calibration shell (PR #85, dry-run).
 *
 * Gated by LEXXHARD_IMU_CALIBRATION (default OFF). When the flag is off,
 * feed_sample() collapses to a no-op so the IMU fetcher loop is unchanged.
 * When on, the shell registers `imu calrun` and `imu calinfo` subcommands.
 *
 * Concurrency model (state machine, see imu_calibration.cpp for details):
 *
 *   IDLE/DONE/FAILED   shell CAS -> STARTING
 *   STARTING           shell exclusively writes g_acc + g_diag, then
 *                      release-store -> RUNNING
 *   RUNNING            fetcher exclusively writes g_acc via feed_sample()
 *   FINALIZING         whoever wins RUNNING -> FINALIZING CAS exclusively
 *                      writes g_diag (fetcher in finalize(), or shell on
 *                      timeout). Loser does nothing.
 *   DONE/FAILED        terminal; g_diag is stable and safe to read after
 *                      an acquire-load observes one of these.
 *
 * feed_sample() only acts when state == RUNNING; STARTING and FINALIZING
 * make it bail, so the shell side has uncontended access to shared data
 * during those phases.
 */
#pragma once

#include <zephyr/drivers/sensor.h>

#ifdef LEXXHARD_IMU_CALIBRATION

#include <zephyr/shell/shell.h>

namespace lexxhard::imu_calibration {

void feed_sample(const struct sensor_value accel[3],
                 const struct sensor_value gyro[3]);

int cmd_calrun(const struct shell *shell, size_t argc, char **argv);
int cmd_calinfo(const struct shell *shell, size_t argc, char **argv);

}  // namespace lexxhard::imu_calibration

#else  /* LEXXHARD_IMU_CALIBRATION */

namespace lexxhard::imu_calibration {

inline void feed_sample(const struct sensor_value[3],
                        const struct sensor_value[3]) {}

}  // namespace lexxhard::imu_calibration

#endif  /* LEXXHARD_IMU_CALIBRATION */
