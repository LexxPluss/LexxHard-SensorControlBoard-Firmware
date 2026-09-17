/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The L7 platform's observable contract, through Zephyr's real i2c_transfer path. No ST register
 * sequence appears here: what is ours is the shape of one transaction, the proven 328-byte bound,
 * address conversion, and preservation of the original transport error.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/ztest.h>

#include "fake_i2c.h"
#include "platform.h"
#include "vl53l7cx_api.h"
#include "vl53l7cx_port.h"

#define TEST_ADDR7 0x29

static VL53L7CX_Platform platform;

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&platform, 0, sizeof(platform));
	platform.address = TEST_ADDR7 << 1; /* ST's 8-bit wire convention */
	fake_i2c_reset();
	vl53l7cx_port_clear_error();
}

ZTEST_SUITE(tof_l7_port, NULL, NULL, before, NULL, NULL);

ZTEST(tof_l7_port, test_write_is_be16_index_then_payload_with_one_final_stop)
{
	uint8_t payload[] = {0xDE, 0xAD, 0xBE};

	zassert_equal(VL53L7CX_WrMulti(&platform, 0xABCD, payload, sizeof(payload)),
		      VL53L7CX_STATUS_OK);
	zassert_equal(fake_i2c_count, 1);

	const struct fake_i2c_xfer *x = &fake_i2c_log[0];
	zassert_equal(x->addr, TEST_ADDR7);
	zassert_equal(x->num_segs, 2);
	zassert_equal(x->segs[0].len, 2);
	zassert_equal(x->segs[0].data[0], 0xAB);
	zassert_equal(x->segs[0].data[1], 0xCD);
	zassert_equal(x->segs[0].flags & I2C_MSG_STOP, 0U);
	zassert_equal(x->segs[0].flags & I2C_MSG_READ, 0U);
	zassert_equal(x->segs[1].len, sizeof(payload));
	zassert_equal(x->segs[1].flags & I2C_MSG_READ, 0U);
	zassert_equal(x->segs[1].flags & I2C_MSG_RESTART, 0U);
	zassert_not_equal(x->segs[1].flags & I2C_MSG_STOP, 0U);
	zassert_mem_equal(x->segs[1].data, payload, sizeof(payload));
}

ZTEST(tof_l7_port, test_read_is_repeated_start_and_one_final_stop)
{
	uint8_t reply[] = {0x12, 0x34};
	uint8_t data[sizeof(reply)] = {0};

	fake_i2c_set_read_data(reply, sizeof(reply));
	zassert_equal(VL53L7CX_RdMulti(&platform, 0x0102, data, sizeof(data)),
		      VL53L7CX_STATUS_OK);
	zassert_equal(fake_i2c_count, 1);

	const struct fake_i2c_xfer *x = &fake_i2c_log[0];
	zassert_equal(x->num_segs, 2);
	zassert_equal(x->segs[0].data[0], 0x01);
	zassert_equal(x->segs[0].data[1], 0x02);
	zassert_equal(x->segs[0].flags & I2C_MSG_STOP, 0U);
	zassert_not_equal(x->segs[1].flags & I2C_MSG_READ, 0U);
	zassert_not_equal(x->segs[1].flags & I2C_MSG_RESTART, 0U);
	zassert_not_equal(x->segs[1].flags & I2C_MSG_STOP, 0U);
	zassert_mem_equal(data, reply, sizeof(reply));
}

ZTEST(tof_l7_port, test_st_eight_bit_address_is_converted_once)
{
	uint8_t byte = 0;

	platform.address = 0x52;
	zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_OK);
	zassert_equal(VL53L7CX_RdByte(&platform, 0, &byte), VL53L7CX_STATUS_OK);
	zassert_true(fake_i2c_all_addressed(0x29));
}

ZTEST(tof_l7_port, test_odd_or_out_of_range_address_is_refused_before_the_bus)
{
	uint8_t byte = 0;

	platform.address = 0x29; /* a 7-bit address mistakenly left in the ST field */
	zassert_equal(VL53L7CX_RdByte(&platform, 0, &byte), VL53L7CX_STATUS_INVALID_PARAM);
	zassert_equal(fake_i2c_count, 0);
	zassert_equal(vl53l7cx_port_error(), -EINVAL);

	vl53l7cx_port_clear_error();
	platform.address = 0x100;
	zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_INVALID_PARAM);
	zassert_equal(fake_i2c_count, 0);
	zassert_equal(vl53l7cx_port_error(), -EINVAL);
}

ZTEST(tof_l7_port, test_large_transfer_is_split_at_the_proven_bound_and_reindexed)
{
	uint8_t payload[VL53L7CX_PORT_MAX_TRANSFER + 1];
	memset(payload, 0xA5, sizeof(payload));

	zassert_equal(VL53L7CX_WrMulti(&platform, 0x0100, payload, sizeof(payload)),
		      VL53L7CX_STATUS_OK);
	zassert_equal(fake_i2c_count, 2);
	zassert_equal(fake_i2c_log[0].segs[0].data[0], 0x01);
	zassert_equal(fake_i2c_log[0].segs[0].data[1], 0x00);
	zassert_equal(fake_i2c_log[0].segs[1].len, VL53L7CX_PORT_MAX_TRANSFER);
	/* 0x0100 + 328 = 0x0248: each piece is a complete indexed transaction. */
	zassert_equal(fake_i2c_log[1].segs[0].data[0], 0x02);
	zassert_equal(fake_i2c_log[1].segs[0].data[1], 0x48);
	zassert_equal(fake_i2c_log[1].segs[1].len, 1);
}

ZTEST(tof_l7_port, test_register_range_overflow_is_refused_without_a_partial_write)
{
	uint8_t payload[VL53L7CX_PORT_MAX_TRANSFER];

	zassert_equal(VL53L7CX_WrMulti(&platform, 0xFF00, payload, sizeof(payload)),
		      VL53L7CX_STATUS_INVALID_PARAM);
	zassert_equal(vl53l7cx_port_error(), -EOVERFLOW);
	zassert_equal(fake_i2c_count, 0, "an impossible operation must not leave a partial write");
}

ZTEST(tof_l7_port, test_segment_failure_stops_the_operation_and_preserves_errno)
{
	uint8_t payload[VL53L7CX_PORT_MAX_TRANSFER * 2 + 1] = {0};

	fake_i2c_fail_on(2, -ETIMEDOUT);
	zassert_equal(VL53L7CX_WrMulti(&platform, 0x0100, payload, sizeof(payload)),
		      VL53L7CX_STATUS_ERROR);
	zassert_equal(fake_i2c_count, 2, "no transaction may follow the failed middle piece");
	zassert_equal(vl53l7cx_port_error(), -ETIMEDOUT);
}

ZTEST(tof_l7_port, test_every_transport_errno_is_preserved_behind_the_uld_status)
{
	static const int errnos[] = {-ENXIO, -ENODEV, -EIO, -ETIMEDOUT, -EBUSY, -EPERM};

	for (size_t i = 0; i < ARRAY_SIZE(errnos); ++i) {
		fake_i2c_reset();
		vl53l7cx_port_clear_error();
		fake_i2c_fail_on(1, errnos[i]);

		zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_ERROR);
		zassert_equal(vl53l7cx_port_error(), errnos[i]);
	}
}

ZTEST(tof_l7_port, test_sticky_error_keeps_the_first_transport_failure)
{
	fake_i2c_fail_on(1, -EIO);
	zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_ERROR);

	fake_i2c_fail_on(2, -ETIMEDOUT);
	zassert_equal(VL53L7CX_WrByte(&platform, 1, 0), VL53L7CX_STATUS_ERROR);
	zassert_equal(vl53l7cx_port_error(), -EIO);

	vl53l7cx_port_clear_error();
	zassert_equal(vl53l7cx_port_error(), 0);
}

ZTEST(tof_l7_port, test_failed_read_does_not_fabricate_bytes)
{
	uint8_t data[] = {0x5A, 0x5A};

	fake_i2c_fail_on(1, -EIO);
	zassert_equal(VL53L7CX_RdMulti(&platform, 0, data, sizeof(data)), VL53L7CX_STATUS_ERROR);
	zassert_equal(data[0], 0x5A);
	zassert_equal(data[1], 0x5A);
}

ZTEST(tof_l7_port, test_zero_length_rules_are_explicit)
{
	uint8_t byte = 0;

	/* The ULD uses a write with only an index in a few probe paths; a zero-byte read has no useful
	 * meaning and is rejected rather than handed to a controller with driver-specific behaviour. */
	zassert_equal(VL53L7CX_WrMulti(&platform, 0x1234, NULL, 0), VL53L7CX_STATUS_OK);
	zassert_equal(fake_i2c_count, 1);
	zassert_equal(fake_i2c_log[0].num_segs, 1);
	zassert_not_equal(fake_i2c_log[0].segs[0].flags & I2C_MSG_STOP, 0U);

	fake_i2c_reset();
	zassert_equal(VL53L7CX_RdMulti(&platform, 0, &byte, 0), VL53L7CX_STATUS_INVALID_PARAM);
	zassert_equal(fake_i2c_count, 0);
}

ZTEST(tof_l7_port, test_swap_buffer_matches_the_uld_word_conversion)
{
	uint8_t data[] = {0x11, 0x22, 0x33, 0x44};

	VL53L7CX_SwapBuffer(data, sizeof(data));
	static const uint8_t expected[] = {0x44, 0x33, 0x22, 0x11};
	zassert_mem_equal(data, expected, sizeof(data));
	zassert_equal(vl53l7cx_port_error(), 0);

	VL53L7CX_SwapBuffer(data, 3);
	zassert_equal(vl53l7cx_port_error(), -EINVAL);
}
