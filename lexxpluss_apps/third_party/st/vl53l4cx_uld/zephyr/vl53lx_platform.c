/*
 * Copyright (c) 2024-2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Zephyr platform layer for the vendored VL53L4CX ULD.
 *
 * This replaces upstream/porting/vl53lx_platform.c, which is kept in the snapshot
 * only as provenance. That file carries one file-scope _I2CBuffer[256] shared by
 * every device plus its own tracing, so it is not reentrant across the four cliff
 * sensors on our single bus and does not fit our locking or test boundaries.
 *
 * There is exactly ONE real I2C path, here. The bus handle is resolved at file
 * scope, which is safe only because all four VL53L4CX sit on the same immutable
 * i2c2 controller and are distinguished by VL53L4CX_Object_t::IO.Address. The BSP
 * IO callbacks cannot carry per-device context - their signature is
 * (uint16_t address, uint8_t *data, uint16_t size) with no handle - so moving any
 * L4 to a second bus would silently address the wrong controller. If that ever
 * happens, this file must gain an explicit per-device bus binding first.
 *
 * The register index is 16-bit big-endian and precedes the payload. It is sent as a
 * separate i2c_msg segment rather than memcpy'd into a bounce buffer, which is what
 * makes concurrent use across sensors safe.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include "vl53lx_platform.h"

#define VL53L4CX_I2C_NODE DT_NODELABEL(i2c2)

/* One erroneous transfer must not look like a vacant address: the enumeration
 * depends on telling a clean NACK from a transport failure, so the mapping from
 * Zephyr errno to the ULD's error space is deliberate and tested. */
static VL53LX_Error vl53lx_map_errno(int err)
{
	switch (err) {
	case 0:
		return VL53LX_ERROR_NONE;
	case -ENXIO:
	case -ENODEV:
		return VL53LX_ERROR_CONTROL_INTERFACE;
	case -EIO:
	case -ETIMEDOUT:
	case -EBUSY:
		return VL53LX_ERROR_CONTROL_INTERFACE;
	default:
		return VL53LX_ERROR_CONTROL_INTERFACE;
	}
}

static const struct device *vl53lx_bus(void)
{
	static const struct device *bus;

	if (bus == NULL) {
		bus = DEVICE_DT_GET(VL53L4CX_I2C_NODE);
	}
	return bus;
}

static uint16_t vl53lx_addr(VL53LX_DEV Dev)
{
	/* The ULD hands us the object; the 7-bit address lives in its IO block. The
	 * ST convention stores the 8-bit wire address, so shift when it is even and
	 * larger than the 7-bit space. */
	uint16_t addr = Dev->IO.Address;

	return (addr > 0x7FU) ? (uint16_t)(addr >> 1) : addr;
}

VL53LX_Error VL53LX_WriteMulti(VL53LX_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	uint8_t idx[2] = {(uint8_t)(index >> 8), (uint8_t)(index & 0xFFU)};
	struct i2c_msg msg[2] = {
		{.buf = idx, .len = sizeof(idx), .flags = I2C_MSG_WRITE},
		{.buf = pdata, .len = (uint32_t)count, .flags = I2C_MSG_WRITE | I2C_MSG_STOP},
	};

	if (count == 0U) {
		msg[0].flags |= I2C_MSG_STOP;
		return vl53lx_map_errno(i2c_transfer(vl53lx_bus(), msg, 1, vl53lx_addr(Dev)));
	}
	return vl53lx_map_errno(i2c_transfer(vl53lx_bus(), msg, 2, vl53lx_addr(Dev)));
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	uint8_t idx[2] = {(uint8_t)(index >> 8), (uint8_t)(index & 0xFFU)};

	/* i2c_write_read issues the index write then a repeated start for the read,
	 * which is what the part expects and what the tests pin. */
	return vl53lx_map_errno(
		i2c_write_read(vl53lx_bus(), vl53lx_addr(Dev), idx, sizeof(idx), pdata, count));
}

VL53LX_Error VL53LX_WrByte(VL53LX_DEV Dev, uint16_t index, uint8_t data)
{
	return VL53LX_WriteMulti(Dev, index, &data, 1);
}

VL53LX_Error VL53LX_WrWord(VL53LX_DEV Dev, uint16_t index, uint16_t data)
{
	uint8_t buf[2];

	sys_put_be16(data, buf);
	return VL53LX_WriteMulti(Dev, index, buf, sizeof(buf));
}

VL53LX_Error VL53LX_WrDWord(VL53LX_DEV Dev, uint16_t index, uint32_t data)
{
	uint8_t buf[4];

	sys_put_be32(data, buf);
	return VL53LX_WriteMulti(Dev, index, buf, sizeof(buf));
}

VL53LX_Error VL53LX_RdByte(VL53LX_DEV Dev, uint16_t index, uint8_t *pdata)
{
	return VL53LX_ReadMulti(Dev, index, pdata, 1);
}

VL53LX_Error VL53LX_RdWord(VL53LX_DEV Dev, uint16_t index, uint16_t *pdata)
{
	uint8_t buf[2];
	VL53LX_Error status = VL53LX_ReadMulti(Dev, index, buf, sizeof(buf));

	if (status == VL53LX_ERROR_NONE) {
		*pdata = sys_get_be16(buf);
	}
	return status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_DEV Dev, uint16_t index, uint32_t *pdata)
{
	uint8_t buf[4];
	VL53LX_Error status = VL53LX_ReadMulti(Dev, index, buf, sizeof(buf));

	if (status == VL53LX_ERROR_NONE) {
		*pdata = sys_get_be32(buf);
	}
	return status;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_DEV Dev, int32_t wait_us)
{
	ARG_UNUSED(Dev);
	if (wait_us <= 0) {
		return VL53LX_ERROR_NONE;
	}
	/* Short waits must not yield: the ULD uses them inside register sequences. */
	k_busy_wait((uint32_t)wait_us);
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_DEV Dev, int32_t wait_ms)
{
	ARG_UNUSED(Dev);
	if (wait_ms <= 0) {
		return VL53LX_ERROR_NONE;
	}
	k_msleep(wait_ms);
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTickCount(VL53LX_DEV Dev, uint32_t *ptick_count_ms)
{
	ARG_UNUSED(Dev);
	*ptick_count_ms = k_uptime_get_32();
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	*ptimer_freq_hz = 1000;
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
	*ptimer_count = (int32_t)k_uptime_get_32();
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(VL53LX_DEV Dev, uint32_t timeout_ms, uint16_t index,
				    uint8_t value, uint8_t mask, uint32_t poll_delay_ms)
{
	uint32_t start = k_uptime_get_32();

	for (;;) {
		uint8_t reg = 0;
		VL53LX_Error status = VL53LX_RdByte(Dev, index, &reg);

		if (status != VL53LX_ERROR_NONE) {
			return status;
		}
		if ((reg & mask) == value) {
			return VL53LX_ERROR_NONE;
		}
		if ((k_uptime_get_32() - start) >= timeout_ms) {
			return VL53LX_ERROR_TIME_OUT;
		}
		(void)VL53LX_WaitMs(Dev, (int32_t)poll_delay_ms);
	}
}

/*
 * Communication lifetime is owned outside the ULD: the bus, its speed and the
 * enable chain all belong to the chain controller, so these are checked no-ops
 * rather than successful lies. They must not be null - VL53L4CX_RegisterBusIO
 * fails when IO.Init is null, and VL53L4CX_DeInit calls IO.DeInit with no null
 * check at all, so a missing DeInit is a null-pointer jump rather than an error.
 */
VL53LX_Error VL53LX_CommsInitialise(VL53LX_DEV Dev, uint8_t comms_type,
				    uint16_t comms_speed_khz)
{
	ARG_UNUSED(comms_type);
	ARG_UNUSED(comms_speed_khz);

	if (Dev == NULL) {
		return VL53LX_ERROR_INVALID_PARAMS;
	}
	if (!device_is_ready(vl53lx_bus())) {
		return VL53LX_ERROR_CONTROL_INTERFACE;
	}
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_CommsClose(VL53LX_DEV Dev)
{
	return (Dev == NULL) ? VL53LX_ERROR_INVALID_PARAMS : VL53LX_ERROR_NONE;
}

/*
 * Every GPIO hook fails loudly. The enable line is a distributed shift register
 * driven by the chain controller, and the contract forbids toggling an L4's enable
 * during acquisition: dropping it returns the device to address 0x29 and destroys
 * the whole chain's addressing. So there is no correct implementation of these, and
 * a successful no-op would let a future caller believe it had reset a sensor.
 */
VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
	ARG_UNUSED(pin);
	ARG_UNUSED(mode);
	return VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
}

VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
	ARG_UNUSED(pin);
	ARG_UNUSED(value);
	return VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
}

VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
	ARG_UNUSED(pin);
	ARG_UNUSED(pvalue);
	return VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
}

VL53LX_Error VL53LX_GpioXshutdown(uint8_t value)
{
	ARG_UNUSED(value);
	return VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
}

VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value)
{
	ARG_UNUSED(value);
	return VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
}

VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value)
{
	ARG_UNUSED(value);
	return VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
}
