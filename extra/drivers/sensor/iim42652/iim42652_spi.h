/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * CHANGELOG:
 * 2024-04-02: created IIM42625 driver based on ICM42605 driver by Takuro Tsujikawa (takuro.tsujikawa@lexxpluss.com)
 * 	- changed definition from ICM42605 to IIM42652
 * 	- changed filename from icm42605_spi.h to iim42652_spi.h
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_IIM42652_IIM42652_SPI_H_
#define ZEPHYR_DRIVERS_SENSOR_IIM42652_IIM42652_SPI_H_

#include <device.h>

int inv_spi_single_write(uint8_t reg, uint8_t *data);
int inv_spi_read(uint8_t reg, uint8_t *data, size_t len);
int iim42652_spi_init(const struct device *spi_device,
		      const struct spi_config *spi_config);

#endif /* __SENSOR_IIM42652_IIM42652_SPI__ */
