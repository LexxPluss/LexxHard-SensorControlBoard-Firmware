/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_IIM42652_IIM42652_SETUP_H_
#define ZEPHYR_DRIVERS_SENSOR_IIM42652_IIM42652_SETUP_H_

#include <device.h>

int iim42652_sensor_init(const struct device *dev);
int iim42652_turn_on_fifo(const struct device *dev);
int iim42652_turn_off_fifo(const struct device *dev);
int iim42652_turn_off_sensor(const struct device *dev);
int iim42652_turn_on_sensor(const struct device *dev);
int iim42652_set_odr(const struct device *dev, int a_rate, int g_rate);

#endif /* __SENSOR_IIM42652_IIM42652_SETUP__ */
