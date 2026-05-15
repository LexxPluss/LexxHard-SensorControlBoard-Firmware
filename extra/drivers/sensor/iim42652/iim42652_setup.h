/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * CHANGELOG:
 * 2024-04-02: created IIM42652 driver based on ICM42605 driver by Takuro Tsujikawa (takuro.tsujikawa@lexxpluss.com)
 * 	- changed definition from ICM42605 to IIM42652
 * 	- changed filename from icm42605_setup.h to iim42652_setup.h
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_IIM42652_IIM42652_SETUP_H_
#define ZEPHYR_DRIVERS_SENSOR_IIM42652_IIM42652_SETUP_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

int iim42652_sensor_init(const struct device *dev);
int iim42652_turn_on_fifo(const struct device *dev);
int iim42652_turn_off_fifo(const struct device *dev);
int iim42652_turn_off_sensor(const struct device *dev);
int iim42652_turn_on_sensor(const struct device *dev);
int iim42652_set_odr(const struct device *dev, int a_rate, int g_rate);

/* Diagnostic register reads. `bank` is one of BIT_BANK_SEL_{0..4}. Both
 * variants take the driver's bus mutex, select the requested bank, perform
 * the read, then restore Bank 0. Sensor power state is not touched — the
 * caller is responsible for any sensor-off requirements dictated by
 * DS §12.9 for registers that need it (e.g. AAF, OFFSET_USER).
 *
 * The mutex serializes against sample_fetch(); without it a 50 Hz fetch
 * could land mid-sequence and read the FIFO from the wrong bank.
 */
int iim42652_diag_read_reg(const struct device *dev, uint8_t bank, uint8_t addr,
			   uint8_t *val);
int iim42652_diag_read_regs(const struct device *dev, uint8_t bank, uint8_t addr,
			    uint8_t *buf, size_t len);

/* Write all six OFFSET_USER channels (gyro X/Y/Z + accel X/Y/Z) in one
 * sensor-off cycle. Each step is a 12-bit signed value:
 *   gyro:  ±2048 step = ±64 dps   (1/32 dps/step ≈ 31.25 mdps/step)
 *   accel: ±2048 step = ±1 g      (0.5 mG/step)
 * Returns -EINVAL if any step is outside [-2048, 2047].
 *
 * Sign convention (verified on PACO v71es007):
 *   sensor_output = sensor_raw + OFFUSER
 *   - ACCEL_Z: 2026-05-14, plan §3.1 (+40/-56 step experiments)
 *   - GYRO X/Y/Z: 2026-05-15 (+320 step per axis → ~+320 LSB sensor-frame
 *     shift on the matching GYRO_DATA register; all three axes same sign
 *     as accel)
 * So to cancel a measured bias, write step = -bias_observed.
 *
 * All 9 OFFSET_USER bytes are rewritten in this call — sibling axes are
 * NOT preserved. If you only want partial update, read back current
 * OFFSET_USER0..8 via iim42652_diag_read_regs() first and merge.
 */
int iim42652_set_offset_user(const struct device *dev,
			     int16_t gx_step, int16_t gy_step, int16_t gz_step,
			     int16_t ax_step, int16_t ay_step, int16_t az_step);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_IIM42652_IIM42652_SETUP__ */
