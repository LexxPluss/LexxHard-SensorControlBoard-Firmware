/*
 * Copyright (c) 2026, LexxPluss Inc.
 *
 * Manual IMU calibration shell (PR #85, dry-run).
 *
 * Gated by LEXXHARD_IMU_CALIBRATION (default OFF). When the flag is off,
 * feed_sample() collapses to a no-op so the IMU fetcher loop is unchanged.
 * When on, the shell registers `imu calrun` and `imu calinfo` subcommands.
 *
 * Concurrency model:
 *   - The IMU fetcher thread (imu_controller.cpp) calls feed_sample() once
 *     per sample. Only that thread mutates the accumulator.
 *   - The shell thread (cmd_calrun) initiates a run via an atomic state
 *     transition IDLE -> RUNNING, then polls the same atomic. It never
 *     touches the accumulator or the sensor directly.
 *   - finalize() runs on the fetcher thread when the sample count reaches
 *     N_SAMPLES, then publishes the final state with a release store so
 *     the shell sees a consistent g_diag on its next acquire load.
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
