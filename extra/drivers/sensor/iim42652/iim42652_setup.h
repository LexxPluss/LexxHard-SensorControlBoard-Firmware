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

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_IIM42652_IIM42652_SETUP__ */
