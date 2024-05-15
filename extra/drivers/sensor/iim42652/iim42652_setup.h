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

#include <zephyr/device.h>

int iim42652_sensor_init(const struct device *dev);
int iim42652_turn_on_fifo(const struct device *dev);
int iim42652_turn_off_fifo(const struct device *dev);
int iim42652_turn_off_sensor(const struct device *dev);
int iim42652_turn_on_sensor(const struct device *dev);
int iim42652_set_odr(const struct device *dev, int a_rate, int g_rate);

#endif /* __SENSOR_IIM42652_IIM42652_SETUP__ */
