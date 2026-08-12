/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Port-level tests: the wire shape and the errno path of zephyr/vl53lx_platform.c.
 *
 * These pin properties the ULD cannot check for us and a device cannot show us cheaply:
 * the register index is 16-bit big-endian, a write is two segments with no STOP between
 * them, a read is an index write followed by a repeated start and one final STOP, and
 * every Zephyr errno both maps into the ULD's error space and survives raw in the
 * sticky record. What is deliberately NOT here is any assertion about a sequence of
 * ULD registers: that would freeze vendor internals into our tests.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/ztest.h>

#include "fake_i2c.h"
#include "vl53l4cx_bus_io.h"
#include "vl53l4cx_port.h"
#include "vl53lx_platform.h"

#define TEST_ADDR7 0x29

static VL53L4CX_Object_t obj;

static void *setup(void)
{
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&obj, 0, sizeof(obj));
	obj.IO.Address = TEST_ADDR7 << 1; /* ST's 8-bit wire convention */
	fake_i2c_reset();
	vl53l4cx_port_sticky_reset();
}

ZTEST_SUITE(tof_cliff_port, NULL, setup, before, NULL, NULL);

ZTEST(tof_cliff_port, test_write_sends_be16_index_then_payload_without_intermediate_stop)
{
	uint8_t payload[3] = {0xDE, 0xAD, 0xBE};

	zassert_equal(VL53LX_WriteMulti(&obj, 0xABCD, payload, sizeof(payload)),
		      VL53LX_ERROR_NONE);
	zassert_equal(fake_i2c_count, 1);

	const struct fake_i2c_xfer *x = &fake_i2c_log[0];

	zassert_equal(x->addr, TEST_ADDR7);
	zassert_equal(x->num_segs, 2, "the index must be its own segment, not a bounce buffer");

	/* Index: big-endian, high byte first. */
	zassert_equal(x->segs[0].len, 2);
	zassert_equal(x->segs[0].data[0], 0xAB);
	zassert_equal(x->segs[0].data[1], 0xCD);

	/* No STOP between index and payload: a STOP here would end the transaction and
	 * the part would treat the payload as a new register index. */
	zassert_equal(x->segs[0].flags & I2C_MSG_STOP, 0U);
	zassert_equal(x->segs[0].flags & I2C_MSG_READ, 0U);

	/* Payload continues the same write and terminates the transaction. */
	zassert_equal(x->segs[1].len, sizeof(payload));
	zassert_equal(x->segs[1].flags & I2C_MSG_READ, 0U);
	zassert_not_equal(x->segs[1].flags & I2C_MSG_STOP, 0U);
	zassert_equal(x->segs[1].flags & I2C_MSG_RESTART, 0U,
		      "a write payload must not begin a new transaction");
	zassert_mem_equal(x->segs[1].data, payload, sizeof(payload));
}

ZTEST(tof_cliff_port, test_zero_length_write_is_one_segment_with_stop)
{
	zassert_equal(VL53LX_WriteMulti(&obj, 0x0102, NULL, 0), VL53LX_ERROR_NONE);
	zassert_equal(fake_i2c_count, 1);
	zassert_equal(fake_i2c_log[0].num_segs, 1);
	zassert_not_equal(fake_i2c_log[0].segs[0].flags & I2C_MSG_STOP, 0U);
}

ZTEST(tof_cliff_port, test_read_uses_repeated_start_and_one_final_stop)
{
	uint8_t buf[2] = {0, 0};
	uint8_t reply[2] = {0xEE, 0xAA};

	fake_i2c_set_read_data(reply, sizeof(reply));
	zassert_equal(VL53LX_ReadMulti(&obj, 0x0102, buf, sizeof(buf)), VL53LX_ERROR_NONE);
	zassert_equal(fake_i2c_count, 1);

	const struct fake_i2c_xfer *x = &fake_i2c_log[0];

	zassert_equal(x->num_segs, 2);
	/* Index write first, no STOP. */
	zassert_equal(x->segs[0].len, 2);
	zassert_equal(x->segs[0].data[0], 0x01);
	zassert_equal(x->segs[0].data[1], 0x02);
	zassert_equal(x->segs[0].flags & I2C_MSG_READ, 0U);
	zassert_equal(x->segs[0].flags & I2C_MSG_STOP, 0U);
	/* Then a repeated start into the read, which is the only STOP. */
	zassert_not_equal(x->segs[1].flags & I2C_MSG_READ, 0U);
	zassert_not_equal(x->segs[1].flags & I2C_MSG_RESTART, 0U);
	zassert_not_equal(x->segs[1].flags & I2C_MSG_STOP, 0U);
	zassert_mem_equal(buf, reply, sizeof(reply));
}

ZTEST(tof_cliff_port, test_multibyte_accessors_are_big_endian_on_the_wire)
{
	uint8_t reply[4] = {0x11, 0x22, 0x33, 0x44};
	uint16_t w = 0;
	uint32_t d = 0;

	zassert_equal(VL53LX_WrWord(&obj, 0x0000, 0x1234), VL53LX_ERROR_NONE);
	zassert_equal(fake_i2c_log[0].segs[1].data[0], 0x12);
	zassert_equal(fake_i2c_log[0].segs[1].data[1], 0x34);

	zassert_equal(VL53LX_WrDWord(&obj, 0x0000, 0x89ABCDEFU), VL53LX_ERROR_NONE);
	zassert_equal(fake_i2c_log[1].segs[1].data[0], 0x89);
	zassert_equal(fake_i2c_log[1].segs[1].data[3], 0xEF);

	fake_i2c_set_read_data(reply, sizeof(reply));
	zassert_equal(VL53LX_RdWord(&obj, 0x0000, &w), VL53LX_ERROR_NONE);
	zassert_equal(w, 0x1122);
	zassert_equal(VL53LX_RdDWord(&obj, 0x0000, &d), VL53LX_ERROR_NONE);
	zassert_equal(d, 0x11223344U);
}

ZTEST(tof_cliff_port, test_st_eight_bit_address_is_halved_for_the_zephyr_api)
{
	uint8_t v = 0;

	/* IO.Address is ST's 8-bit wire address - VL53LX_SetDeviceAddress writes
	 * DeviceAddress / 2 to the part and VL53L4CX_SetAddress copies the 8-bit value
	 * back into this field - so 0x52 must reach Zephyr as 0x29. Getting this wrong
	 * addresses a neighbouring sensor rather than failing. */
	obj.IO.Address = 0x52;
	zassert_equal(VL53LX_WrByte(&obj, 0x0000, 0x00), VL53LX_ERROR_NONE);
	zassert_equal(VL53LX_RdByte(&obj, 0x0000, &v), VL53LX_ERROR_NONE);
	zassert_true(fake_i2c_all_addressed(0x29));
}

ZTEST(tof_cliff_port, test_a_seven_bit_address_left_in_the_field_is_refused_not_guessed)
{
	uint8_t v = 0;

	/* The two conventions overlap in 0x00..0x7F, so no heuristic can separate them.
	 * An odd value is the signature of a 7-bit address stored by mistake, and it must
	 * fail rather than address half the chain one bit off. */
	obj.IO.Address = 0x29;
	zassert_equal(VL53LX_WrByte(&obj, 0x0000, 0x00), VL53LX_ERROR_INVALID_PARAMS);
	zassert_equal(VL53LX_RdByte(&obj, 0x0000, &v), VL53LX_ERROR_INVALID_PARAMS);
	zassert_equal(fake_i2c_count, 0, "a malformed address must not reach the bus");
	zassert_equal(vl53l4cx_port_sticky_errno(), -EINVAL);

	/* Out of range likewise. */
	obj.IO.Address = 0x100;
	zassert_equal(VL53LX_WrByte(&obj, 0x0000, 0x00), VL53LX_ERROR_INVALID_PARAMS);
	zassert_equal(fake_i2c_count, 0);
}

ZTEST(tof_cliff_port, test_every_errno_maps_to_control_interface_and_is_kept_raw)
{
	static const int errnos[] = {-ENXIO, -ENODEV, -EIO, -ETIMEDOUT, -EBUSY, -EPERM};

	for (size_t i = 0; i < ARRAY_SIZE(errnos); i++) {
		fake_i2c_reset();
		vl53l4cx_port_sticky_reset();
		fake_i2c_fail_on(1, errnos[i]);

		zassert_equal(VL53LX_WrByte(&obj, 0x0000, 0x00),
			      VL53LX_ERROR_CONTROL_INTERFACE,
			      "a transport failure must never look like a vacant address");
		/* The mapping is lossy on purpose - the ULD has one code for every
		 * transport problem - so the raw value has to survive somewhere. */
		zassert_equal(vl53l4cx_port_sticky_errno(), errnos[i]);
	}
}

ZTEST(tof_cliff_port, test_sticky_keeps_the_first_errno_across_later_traffic)
{
	uint8_t v = 0;

	/* Middle transaction fails. Two succeed after it. */
	fake_i2c_fail_on(2, -ETIMEDOUT);
	zassert_equal(VL53LX_WrByte(&obj, 0x0000, 0x00), VL53LX_ERROR_NONE);
	zassert_equal(VL53LX_WrByte(&obj, 0x0001, 0x00), VL53LX_ERROR_CONTROL_INTERFACE);
	fake_i2c_fail_on(0, 0);
	zassert_equal(VL53LX_RdByte(&obj, 0x0002, &v), VL53LX_ERROR_NONE);
	zassert_equal(VL53LX_RdByte(&obj, 0x0003, &v), VL53LX_ERROR_NONE);

	/* A later success must not clear the record: this is the whole reason the
	 * adapter can trust the sticky value over a ULD return code. */
	zassert_equal(vl53l4cx_port_sticky_errno(), -ETIMEDOUT);

	vl53l4cx_port_sticky_reset();
	zassert_equal(vl53l4cx_port_sticky_errno(), 0);
}

ZTEST(tof_cliff_port, test_read_leaves_the_buffer_untouched_when_the_transfer_fails)
{
	uint8_t buf[2] = {0x5A, 0x5A};

	fake_i2c_fail_on(1, -EIO);
	zassert_equal(VL53LX_ReadMulti(&obj, 0x0000, buf, sizeof(buf)),
		      VL53LX_ERROR_CONTROL_INTERFACE);
	zassert_equal(buf[0], 0x5A, "a failed read must not fabricate data");
	zassert_equal(buf[1], 0x5A);
}

ZTEST(tof_cliff_port, test_word_accessors_do_not_write_through_on_failure)
{
	uint16_t w = 0xBEEF;
	uint32_t d = 0xDEADBEEFU;

	fake_i2c_fail_on(1, -EIO);
	zassert_equal(VL53LX_RdWord(&obj, 0x0000, &w), VL53LX_ERROR_CONTROL_INTERFACE);
	zassert_equal(w, 0xBEEF);

	fake_i2c_reset();
	fake_i2c_fail_on(1, -EIO);
	zassert_equal(VL53LX_RdDWord(&obj, 0x0000, &d), VL53LX_ERROR_CONTROL_INTERFACE);
	zassert_equal(d, 0xDEADBEEFU);
}

ZTEST(tof_cliff_port, test_gpio_hooks_all_refuse)
{
	uint8_t v = 0;

	/* There is no correct implementation: the enable line is the chain's addressing
	 * mechanism. A successful no-op would let a caller believe it had reset a part. */
	zassert_equal(VL53LX_GpioSetMode(0, 0), VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
	zassert_equal(VL53LX_GpioSetValue(0, 0), VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
	zassert_equal(VL53LX_GpioGetValue(0, &v), VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
	zassert_equal(VL53LX_GpioXshutdown(0), VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
	zassert_equal(VL53LX_GpioCommsSelect(0), VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
	zassert_equal(VL53LX_GpioPowerEnable(0), VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
	zassert_equal(fake_i2c_count, 0, "a refused GPIO hook must not touch the bus");
}

ZTEST(tof_cliff_port, test_bus_io_block_has_real_callbacks_and_failing_placeholders)
{
	VL53L4CX_IO_t io;

	vl53l4cx_bus_io_fill(&io, TEST_ADDR7);

	/* One place converts the 7-bit address a caller thinks in to the 8-bit form the
	 * ULD stores, so no other layer has to know the convention. */
	zassert_equal(io.Address, TEST_ADDR7 << 1);

	/* VL53L4CX_RegisterBusIO fails when Init is null, and VL53L4CX_DeInit calls
	 * DeInit with no null check at all, so neither may be omitted. */
	zassert_not_null(io.Init);
	zassert_not_null(io.DeInit);

	/* Init is a real check, not an unconditional success: RegisterBusIO returns
	 * whatever it returns, so a controller that never initialised must fail at open
	 * with a stage attached rather than at the first transfer of the first read. */
	zassert_true(vl53l4cx_port_bus_ready(), "the emulated controller must be ready");
	zassert_equal(io.Init(), 0);
	zassert_equal(io.DeInit(), 0);

	/* GetTick has to be real: upstream/vl53l4cx.c polls with it directly. */
	zassert_not_null(io.GetTick);
	(void)io.GetTick();

	/* WriteReg and ReadReg are placeholders precisely because there is one real I2C
	 * path. Their only caller anywhere in the snapshot is the upstream example layer,
	 * which is not compiled, so traffic through them means a second transport path
	 * appeared behind the port's back. */
	zassert_not_null(io.WriteReg);
	zassert_not_null(io.ReadReg);
	zassert_equal(vl53l4cx_bus_io_placeholder_calls(), 0);
	zassert_not_equal(io.WriteReg(TEST_ADDR7, NULL, 0), 0);
	zassert_not_equal(io.ReadReg(TEST_ADDR7, NULL, 0), 0);
	zassert_equal(vl53l4cx_bus_io_placeholder_calls(), 2,
		      "the counter is the defence in depth behind the missing enable API");
	vl53l4cx_bus_io_reset_placeholder_calls();
	zassert_equal(vl53l4cx_bus_io_placeholder_calls(), 0);
}
