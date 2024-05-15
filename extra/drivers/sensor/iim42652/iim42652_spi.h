/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * CHANGELOG:
 * 2024-04-02: created IIM42652 driver based on ICM42605 driver by Takuro Tsujikawa (takuro.tsujikawa@lexxpluss.com)
 * 	- changed definition from ICM42605 to IIM42652
 * 	- changed filename from icm42605_spi.h to iim42652_spi.h
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_IIM42652_IIM42652_SPI_H_
#define ZEPHYR_DRIVERS_SENSOR_IIM42652_IIM42652_SPI_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

int inv_spi_single_write(const struct spi_dt_spec *bus, uint8_t reg, uint8_t *data);
int inv_spi_read(const struct spi_dt_spec *bus, uint8_t reg, uint8_t *data, size_t len);

#endif /* __SENSOR_IIM42652_IIM42652_SPI__ */
