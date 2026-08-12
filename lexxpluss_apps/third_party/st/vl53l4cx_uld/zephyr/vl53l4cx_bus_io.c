/*
 * Copyright (c) 2024-2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * There is exactly one real I2C path in this port, and it is in
 * vl53lx_platform.c. The BSP IO block still has to be filled, because
 * VL53L4CX_RegisterBusIO fails when IO.Init is null and VL53L4CX_DeInit calls
 * IO.DeInit with no null check at all, so omitting either is a failure or a
 * null-pointer jump rather than a shortcut.
 *
 * WriteReg and ReadReg are therefore placeholders that fail loudly and count
 * themselves. Any traffic arriving here means a second transport path has appeared
 * behind the port's back, which is exactly what the tests forbid.
 */

#include "vl53l4cx_bus_io.h"

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(vl53l4cx_bus_io, CONFIG_LOG_DEFAULT_LEVEL);

static uint32_t placeholder_calls;

static int32_t io_init(void)
{
	/* The bus, its speed and the enable chain belong to the chain controller. */
	return 0;
}

static int32_t io_deinit(void)
{
	return 0;
}

static int32_t io_get_tick(void)
{
	/* Real, not a stub: vl53l4cx.c polls with IO.GetTick directly. */
	return (int32_t)k_uptime_get_32();
}

static int32_t io_write_reg_placeholder(uint16_t addr, uint8_t *data, uint16_t len)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	placeholder_calls++;
	LOG_ERR("VL53L4CX IO.WriteReg is a placeholder and must never carry traffic");
	return -1;
}

static int32_t io_read_reg_placeholder(uint16_t addr, uint8_t *data, uint16_t len)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	placeholder_calls++;
	LOG_ERR("VL53L4CX IO.ReadReg is a placeholder and must never carry traffic");
	return -1;
}

void vl53l4cx_bus_io_fill(VL53L4CX_IO_t *pIO, uint16_t address_7bit)
{
	pIO->Init = io_init;
	pIO->DeInit = io_deinit;
	/* ST's 8-bit wire convention, which is what the ULD itself stores here: see the
	 * address note in vl53lx_platform.c. Taking a 7-bit argument and shifting once,
	 * in one place, is what keeps the convention from being guessed at downstream. */
	pIO->Address = (uint16_t)(address_7bit << 1);
	pIO->WriteReg = io_write_reg_placeholder;
	pIO->ReadReg = io_read_reg_placeholder;
	pIO->GetTick = io_get_tick;
}

uint32_t vl53l4cx_bus_io_placeholder_calls(void)
{
	return placeholder_calls;
}

void vl53l4cx_bus_io_reset_placeholder_calls(void)
{
	placeholder_calls = 0;
}
