/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The L7 platform's observable contract, through Zephyr's real i2c_transfer path. No ST register
 * sequence appears here: what is ours is the shape of one transaction, the proven 328-byte bound,
 * address conversion, and preservation of the original transport error.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/ztest.h>

#include "fake_i2c.h"
#include "platform.h"
#include "vl53l7cx_api.h"
#include "vl53l7cx_port.h"

#define TEST_ADDR7 0x29

/* The port no longer compiles a bus in, so the test names one -- which is the behaviour under
 * test as much as it is setup. */
static struct vl53l7cx_port_context ctx;
static VL53L7CX_Platform platform;

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&ctx, 0, sizeof(ctx));
	ctx.bus = DEVICE_DT_GET(DT_NODELABEL(i2c2));
	memset(&platform, 0, sizeof(platform));
	platform.port = &ctx;
	platform.address = TEST_ADDR7 << 1; /* ST's 8-bit wire convention */
	fake_i2c_reset();
	vl53l7cx_port_clear_error(&ctx);
	vl53l7cx_port_clear_argument_fault();
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
	zassert_equal(vl53l7cx_port_error(&ctx), -EINVAL);

	vl53l7cx_port_clear_error(&ctx);
	platform.address = 0x100;
	zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_INVALID_PARAM);
	zassert_equal(fake_i2c_count, 0);
	zassert_equal(vl53l7cx_port_error(&ctx), -EINVAL);
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
	zassert_equal(vl53l7cx_port_error(&ctx), -EOVERFLOW);
	zassert_equal(fake_i2c_count, 0, "an impossible operation must not leave a partial write");
}

ZTEST(tof_l7_port, test_segment_failure_stops_the_operation_and_preserves_errno)
{
	uint8_t payload[VL53L7CX_PORT_MAX_TRANSFER * 2 + 1] = {0};

	fake_i2c_fail_on(2, -ETIMEDOUT);
	zassert_equal(VL53L7CX_WrMulti(&platform, 0x0100, payload, sizeof(payload)),
		      VL53L7CX_STATUS_ERROR);
	zassert_equal(fake_i2c_count, 2, "no transaction may follow the failed middle piece");
	zassert_equal(vl53l7cx_port_error(&ctx), -ETIMEDOUT);
}

ZTEST(tof_l7_port, test_every_transport_errno_is_preserved_behind_the_uld_status)
{
	static const int errnos[] = {-ENXIO, -ENODEV, -EIO, -ETIMEDOUT, -EBUSY, -EPERM};

	for (size_t i = 0; i < ARRAY_SIZE(errnos); ++i) {
		fake_i2c_reset();
		vl53l7cx_port_clear_error(&ctx);
		fake_i2c_fail_on(1, errnos[i]);

		zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_ERROR);
		zassert_equal(vl53l7cx_port_error(&ctx), errnos[i]);
	}
}

ZTEST(tof_l7_port, test_sticky_error_keeps_the_first_transport_failure)
{
	fake_i2c_fail_on(1, -EIO);
	zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_ERROR);

	fake_i2c_fail_on(2, -ETIMEDOUT);
	zassert_equal(VL53L7CX_WrByte(&platform, 1, 0), VL53L7CX_STATUS_ERROR);
	zassert_equal(vl53l7cx_port_error(&ctx), -EIO);

	vl53l7cx_port_clear_error(&ctx);
	zassert_equal(vl53l7cx_port_error(&ctx), 0);
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
	zassert_equal(vl53l7cx_port_argument_fault(), 0);

	/* A misaligned length is still refused, but it is now recorded as an image-wide argument
	 * fault rather than as a transport error on whichever device happened to be in scope.
	 * SwapBuffer is the one ST callback with no platform pointer, so there is no device to
	 * attribute it to and pretending otherwise was the bug. */
	VL53L7CX_SwapBuffer(data, 3);
	zassert_equal(vl53l7cx_port_argument_fault(), -EINVAL);
	zassert_equal(vl53l7cx_port_error(&ctx), 0, "no device did this");
}

/* ---- how many transactions completed, which the error alone cannot say ---- */

/* WHY THE PORT COUNTS AT ALL. The recovery pass has to tell an address where nothing answered --
 * the ordinary cold boot -- from a survivor that acknowledged part of an exchange and NACKed the
 * rest. The first error cannot say: both arrive as -ENXIO. The ULD's own alive flag cannot either,
 * because it is set only when BOTH identity bytes match, so a device that answers one read and
 * refuses the other reports zero exactly like an empty address. Only "did anything complete"
 * separates them, and only the port knows. */
ZTEST(tof_l7_port, test_completed_transfers_counts_what_got_through)
{
	uint8_t byte = 0;

	zassert_equal(vl53l7cx_port_completed_transfers(&ctx), 0U, "cleared in the fixture");

	zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_OK);
	zassert_equal(vl53l7cx_port_completed_transfers(&ctx), 1U);
	zassert_equal(VL53L7CX_RdByte(&platform, 0, &byte), VL53L7CX_STATUS_OK);
	zassert_equal(vl53l7cx_port_completed_transfers(&ctx), 2U);

	vl53l7cx_port_clear_error(&ctx);
	zassert_equal(vl53l7cx_port_completed_transfers(&ctx), 0U,
		      "the count belongs to one ULD operation, so the clear resets it with the error");
}

/* A TRANSACTION THAT FAILED DID NOT COMPLETE. This is the direction that matters: a non-zero count
 * is evidence that something on the bus answered, so counting attempts instead of completions would
 * make every cold boot look like a partial exchange. */
ZTEST(tof_l7_port, test_a_failed_transaction_is_not_counted)
{
	fake_i2c_fail_on(1, -ENXIO);
	zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_ERROR);
	zassert_equal(vl53l7cx_port_completed_transfers(&ctx), 0U, "a NACK is not a completion");
	zassert_equal(vl53l7cx_port_error(&ctx), -ENXIO);
}

/* AND THE PARTIAL EXCHANGE, which is the whole case this exists for: one transaction through, the
 * next refused. The error says -ENXIO and the count says something answered, and the pass needs
 * both to reach the right conclusion. */
ZTEST(tof_l7_port, test_a_partial_exchange_shows_both_an_error_and_a_completion)
{
	uint8_t byte = 0;

	zassert_equal(VL53L7CX_WrByte(&platform, 0, 0), VL53L7CX_STATUS_OK);
	fake_i2c_fail_on(2, -ENXIO);
	zassert_equal(VL53L7CX_RdByte(&platform, 1, &byte), VL53L7CX_STATUS_ERROR);

	zassert_equal(vl53l7cx_port_error(&ctx), -ENXIO, "the refusal is recorded");
	zassert_equal(vl53l7cx_port_completed_transfers(&ctx), 1U, "and so is what got through");
}

/* A segmented operation that fails midway counts the pieces that landed, not the operation. The
 * recovery pass only asks whether the count is zero, but a caller reading it as "operations" would
 * be wrong, so the meaning is pinned. */
ZTEST(tof_l7_port, test_segments_are_counted_individually)
{
	uint8_t payload[VL53L7CX_PORT_MAX_TRANSFER * 2 + 1] = {0};

	fake_i2c_fail_on(2, -ETIMEDOUT);
	zassert_equal(VL53L7CX_WrMulti(&platform, 0x0100, payload, sizeof(payload)),
		      VL53L7CX_STATUS_ERROR);
	zassert_equal(vl53l7cx_port_completed_transfers(&ctx), 1U,
		      "the first segment landed, the second did not, and the rest never ran");
}


/* ---- what the per-device context is for -------------------------------------------------- */

/* THE STATE IS PER DEVICE, NOT PER IMAGE. Before this, one atomic held the first errno for every
 * L7 in the build, so a failure on one bus was indistinguishable from a failure on another and a
 * recovery pass asking "did anything answer" could be answered about the wrong sensor. */
ZTEST(tof_l7_port, test_two_contexts_do_not_share_error_or_completion_state)
{
	struct vl53l7cx_port_context other = {.bus = DEVICE_DT_GET(DT_NODELABEL(i2c2))};
	VL53L7CX_Platform other_platform = {.address = TEST_ADDR7 << 1, .port = &other};
	uint8_t byte = 0;

	fake_i2c_fail_on(1, -ENXIO);
	zassert_equal(VL53L7CX_RdByte(&platform, 0x0010, &byte), VL53L7CX_STATUS_ERROR);

	fake_i2c_fail_on(0, 0);
	zassert_equal(VL53L7CX_RdByte(&other_platform, 0x0010, &byte), VL53L7CX_STATUS_OK);

	zassert_equal(vl53l7cx_port_error(&ctx), -ENXIO, "the failing device keeps its errno");
	zassert_equal(vl53l7cx_port_completed_transfers(&ctx), 0U, "and nothing completed for it");
	zassert_equal(vl53l7cx_port_error(&other), 0, "the healthy device is not blamed for it");
	zassert_equal(vl53l7cx_port_completed_transfers(&other), 1U, "and its own work is counted");
}

/* THE BUS COMES FROM THE CONTEXT. A context naming no controller must refuse before any traffic:
 * if the port still reached a compiled-in bus, the fake would record a transfer here. */
ZTEST(tof_l7_port, test_a_context_without_a_bus_refuses_before_any_traffic)
{
	struct vl53l7cx_port_context busless = {.bus = NULL};
	VL53L7CX_Platform busless_platform = {.address = TEST_ADDR7 << 1, .port = &busless};
	uint8_t byte = 0;

	zassert_equal(VL53L7CX_RdByte(&busless_platform, 0x0010, &byte), VL53L7CX_STATUS_ERROR);
	zassert_equal(fake_i2c_count, 0, "no controller was named, so nothing may have been driven");
	zassert_equal(vl53l7cx_port_error(&busless), -ENODEV);

	zassert_equal(VL53L7CX_RdByte(&platform, 0x0010, &byte), VL53L7CX_STATUS_OK,
		      "a context that does name one still works");
	zassert_equal(fake_i2c_count, 1);
}

/* A PLATFORM WITH NO CONTEXT AT ALL is a programming error, and must not fall back to anything. */
ZTEST(tof_l7_port, test_a_platform_without_a_context_refuses_before_any_traffic)
{
	VL53L7CX_Platform orphan = {.address = TEST_ADDR7 << 1, .port = NULL};
	uint8_t byte = 0;

	zassert_equal(VL53L7CX_RdByte(&orphan, 0x0010, &byte), VL53L7CX_STATUS_ERROR);
	zassert_equal(fake_i2c_count, 0);
	zassert_equal(vl53l7cx_port_error(NULL), -ENODEV, "a question that could not be asked");
}

/* THE ONE FAULT WITH NO DEVICE TO BLAME stays out of every device's errno. Folding it into one
 * would report an image-wide bug as a transport failure on whichever sensor ran last. */
ZTEST(tof_l7_port, test_swap_buffer_argument_fault_is_not_a_device_error)
{
	struct vl53l7cx_port_context other = {.bus = DEVICE_DT_GET(DT_NODELABEL(i2c2))};

	VL53L7CX_SwapBuffer(NULL, 4);

	zassert_equal(vl53l7cx_port_argument_fault(), -EINVAL, "latched where a test can see it");
	zassert_equal(vl53l7cx_port_error(&ctx), 0, "and attributed to no device");
	zassert_equal(vl53l7cx_port_error(&other), 0);

	vl53l7cx_port_clear_argument_fault();
	zassert_equal(vl53l7cx_port_argument_fault(), 0);
}
