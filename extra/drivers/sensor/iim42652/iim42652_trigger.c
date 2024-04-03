/*
 * Copyright (c) 2016 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * CHANGELOG:
 * 2024-04-02: created IIM42625 driver based on ICM42605 driver by Takuro Tsujikawa (takuro.tsujikawa@lexxpluss.com)
 * 	- changed definition from ICM42605 to IIM42652
 * 	- changed filename from icm42605_trigger.c to iim42652_trigger.c
 *  - Fixed the issue based on https://github.com/zephyrproject-rtos/zephyr/commit/0c6cf9a84ee9f677149e257ce00ddd141865c469
 */

#include <device.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include "iim42652.h"
#include "iim42652_setup.h"

LOG_MODULE_DECLARE(IIM42652, CONFIG_SENSOR_LOG_LEVEL);

int iim42652_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct iim42652_data *drv_data = dev->data;
	const struct iim42652_config *cfg = dev->config;

	if (trig->type != SENSOR_TRIG_DATA_READY
	    && trig->type != SENSOR_TRIG_TAP
	    && trig->type != SENSOR_TRIG_DOUBLE_TAP) {
		return -ENOTSUP;
	}

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_DISABLE);

	if (handler == NULL) {
		iim42652_turn_off_sensor(dev);
		return 0;
	}

	if (trig->type == SENSOR_TRIG_DATA_READY) {
		drv_data->data_ready_handler = handler;
		drv_data->data_ready_trigger = *trig;
	} else if (trig->type == SENSOR_TRIG_TAP) {
		drv_data->tap_handler = handler;
		drv_data->tap_trigger = *trig;
		drv_data->tap_en = true;
	} else if (trig->type == SENSOR_TRIG_DOUBLE_TAP) {
		drv_data->double_tap_handler = handler;
		drv_data->double_tap_trigger = *trig;
		drv_data->tap_en = true;
	} else {
		return -ENOTSUP;
	}

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

	iim42652_turn_on_sensor(dev);

	return 0;
}

static void iim42652_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb, uint32_t pins)
{
	struct iim42652_data *drv_data =
		CONTAINER_OF(cb, struct iim42652_data, gpio_cb);
	const struct iim42652_config *cfg = drv_data->dev->config;

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_DISABLE);

	k_sem_give(&drv_data->gpio_sem);
}

static void iim42652_thread_cb(const struct device *dev)
{
	struct iim42652_data *drv_data = dev->data;
	const struct iim42652_config *cfg = dev->config;

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev,
					     &drv_data->data_ready_trigger);
	}

	if (drv_data->tap_handler != NULL ||
	    drv_data->double_tap_handler != NULL) {
		iim42652_tap_fetch(dev);
	}

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);
}

static void iim42652_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct iim42652_data *drv_data = p1;

	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		iim42652_thread_cb(drv_data->dev);
	}
}

int iim42652_init_interrupt(const struct device *dev)
{
	struct iim42652_data *drv_data = dev->data;
	const struct iim42652_config *cfg = dev->config;
	int result = 0;

	/* setup data ready gpio interrupt */
	drv_data->gpio = device_get_binding(cfg->int_label);
	if (drv_data->gpio == NULL) {
		LOG_INF("Failed to get pointer to %s device",
			cfg->int_label);
		return -ENODEV;
	}

	drv_data->dev = dev;

	gpio_pin_configure(drv_data->gpio, cfg->int_pin,
			   GPIO_INPUT | cfg->int_flags);

	gpio_init_callback(&drv_data->gpio_cb,
			   iim42652_gpio_callback,
			   BIT(cfg->int_pin));

	result = gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb);
	if (result < 0) {
		LOG_INF("Failed to set gpio callback");
		return result;
	}

	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);
	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_IIM42652_THREAD_STACK_SIZE, iim42652_thread, drv_data, NULL, NULL,
			K_PRIO_COOP(CONFIG_IIM42652_THREAD_PRIORITY), 0, K_NO_WAIT);

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_EDGE_TO_INACTIVE);

	return 0;
}
