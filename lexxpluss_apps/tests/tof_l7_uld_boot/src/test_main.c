/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The MCU boot wait, and that a boot nobody saw is not a success.
 *
 * WHY THIS SUITE EXISTS. Auditing what vl53l7cx_init() treats as success found 0002's defect in a
 * second place: _vl53l7cx_poll_for_mcu_boot() polled GO2 status 0 for bit 7 up to 500 times and, on
 * running out, fell through to `return status` without setting anything -- so with every read
 * succeeding it reported VL53L7CX_STATUS_OK for a boot that never happened. It also OR-ed the
 * DEVICE's status 1 byte into the status, which is the same collision 0002 removed from
 * vl53l7cx_stop_ranging(): the ULD's own codes live in those 8 bits.
 *
 * The function is `static`, so it is reached by INCLUDING the patched translation unit rather than
 * by driving init() -- getting init that far means satisfying two firmware-access polls and an
 * 86 KB download first, which would make the fake larger than the thing under test.
 *
 * THE STATUS CONSTANTS ARE NOT DISJOINT BIT FLAGS, as the sibling suite also had to say:
 * VL53L7CX_STATUS_ERROR is 0xFF and contains TIMEOUT_ERROR's 0x01, so `rc & TIMEOUT_ERROR` is true
 * of the generic failure too. Every assertion below is on the EXACT value.
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/ztest.h>

/* The patched vendor source, included rather than linked: see the CMakeLists. */
#include "vl53l7cx_api.c"

#define REG_G02_STATUS_0 0x0006
#define REG_G02_STATUS_1 0x0007

/* -1 = never */
static int bit7_after_polls;
static int bit0_after_polls;
static uint8_t status1_value;
static int fail_reads;
static int polls_seen;
static int waits_seen;

static void bus_reset(void)
{
	bit7_after_polls = -1;
	bit0_after_polls = -1;
	status1_value = 0x00;
	fail_reads = 0;
	polls_seen = 0;
	waits_seen = 0;
}

/* MODELS THE REAL PORT'S FAILURE: read_chunk() in zephyr/platform.c hands the caller's buffer
 * straight to i2c_transfer() and returns its error, so a failed read leaves those bytes exactly as
 * they were -- and returns VL53L7CX_STATUS_ERROR, never 0x01. */
uint8_t VL53L7CX_RdByte(VL53L7CX_Platform *platform, uint16_t reg, uint8_t *value)
{
	(void)platform;
	if (fail_reads) {
		return VL53L7CX_STATUS_ERROR;   /* value deliberately untouched */
	}
	if (reg == REG_G02_STATUS_0) {
		++polls_seen;
		uint8_t answer = 0x00;
		if (bit7_after_polls >= 0 && polls_seen >= bit7_after_polls) {
			answer |= 0x80;
		}
		if (bit0_after_polls >= 0 && polls_seen >= bit0_after_polls) {
			answer |= 0x01;
		}
		*value = answer;
		return VL53L7CX_STATUS_OK;
	}
	if (reg == REG_G02_STATUS_1) {
		*value = status1_value;
		return VL53L7CX_STATUS_OK;
	}
	*value = 0x00;
	return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size)
{
	(void)platform;
	(void)reg;
	if (fail_reads) {
		return VL53L7CX_STATUS_ERROR;
	}
	memset(values, 0, size);
	return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_WrByte(VL53L7CX_Platform *platform, uint16_t reg, uint8_t value)
{
	(void)platform;
	(void)reg;
	(void)value;
	return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size)
{
	(void)platform;
	(void)reg;
	(void)values;
	(void)size;
	return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform *platform, uint32_t time_ms)
{
	(void)platform;
	(void)time_ms;
	++waits_seen;
	return VL53L7CX_STATUS_OK;
}

void VL53L7CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
	(void)buffer;
	(void)size;
}

static VL53L7CX_Configuration dev;

static void before(void *unused)
{
	(void)unused;
	bus_reset();
	memset(&dev, 0, sizeof dev);
	dev.platform.address = 0x52;
}

ZTEST_SUITE(tof_l7_uld_boot, NULL, NULL, before, NULL, NULL);

ZTEST(tof_l7_uld_boot, test_a_boot_that_never_answers_is_a_timeout_not_a_success)
{
	/* THE DEFECT. Before the patch this returned VL53L7CX_STATUS_OK: every read succeeded, nothing
	 * set an error, and the loop simply ran out. init() then went on to the next step, where the
	 * failure surfaced two seconds later as a UI command that did not answer -- the right verdict
	 * for the wrong reason, which is the misattribution this driver's bench rounds keep removing. */
	const uint8_t rc = _vl53l7cx_poll_for_mcu_boot(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR,
		      "a boot wait that ran out must say so itself, not leave it to a later step");
	zassert_equal(polls_seen, 500, "the whole poll budget was spent: %d", polls_seen);
	zassert_equal(waits_seen, 500, "%d", waits_seen);
}

ZTEST(tof_l7_uld_boot, test_a_boot_that_answers_is_still_ok)
{
	bit7_after_polls = 1;
	status1_value = 0x00;

	const uint8_t rc = _vl53l7cx_poll_for_mcu_boot(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_OK);
	zassert_equal(polls_seen, 1, "it stopped as soon as the device answered");
}

ZTEST(tof_l7_uld_boot, test_a_boot_that_answers_late_is_also_ok)
{
	bit7_after_polls = 400;

	const uint8_t rc = _vl53l7cx_poll_for_mcu_boot(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_OK);
	zassert_equal(polls_seen, 400);
}

ZTEST(tof_l7_uld_boot, test_a_device_status_1_becomes_a_named_error_rather_than_its_own_byte)
{
	/* `status |= go2_status1` put a DEVICE byte where the ULD's codes live. 0x01 is
	 * TIMEOUT_ERROR's value, so a device reporting 0x01 was indistinguishable from a boot that ran
	 * out -- the two have different next steps. */
	bit7_after_polls = 1;
	status1_value = 0x01;

	const uint8_t rc = _vl53l7cx_poll_for_mcu_boot(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_ERROR, "a non-zero status 1 is a failure");
	zassert_not_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR,
			  "and must not borrow the timeout's value from the device");
	zassert_equal(polls_seen, 1, "it did not spend the budget, so the timeout would be a lie");
}

ZTEST(tof_l7_uld_boot, test_an_unrecognised_status_1_is_the_same_named_error)
{
	bit7_after_polls = 1;
	status1_value = 0x5A;

	zassert_equal(_vl53l7cx_poll_for_mcu_boot(&dev), VL53L7CX_STATUS_ERROR);
}

ZTEST(tof_l7_uld_boot, test_the_bit_zero_exit_keeps_its_upstream_meaning)
{
	/* Upstream leaves the loop when GO2 status 0 bit 0 is set, with status untouched. What that bit
	 * means is not established here and is NOT changed by the patch: the only thing the patch adds
	 * is that this exit is not mistaken for running out of iterations. */
	bit0_after_polls = 3;

	const uint8_t rc = _vl53l7cx_poll_for_mcu_boot(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_OK, "an early exit is not a timeout");
	zassert_equal(polls_seen, 3);
}

ZTEST(tof_l7_uld_boot, test_a_bit_zero_exit_on_the_last_iteration_is_not_a_timeout)
{
	/* THE CASE THE FLAG EXISTS FOR. A post-loop `timeout >= 500` test would have called this a
	 * timeout, because the counter reaches 500 on the very iteration that leaves early. The two
	 * early exits mark themselves instead, so only running out of iterations becomes a timeout. */
	bit0_after_polls = 500;

	const uint8_t rc = _vl53l7cx_poll_for_mcu_boot(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_OK,
		      "it left on the bit, not on the budget, and the two are different events");
	zassert_equal(polls_seen, 500);
}

ZTEST(tof_l7_uld_boot, test_a_transport_error_is_not_overwritten_by_the_timeout)
{
	/* The ordering 0002 settled, in this function too: an error already recorded is kept, so the
	 * layer that failed is still readable. */
	fail_reads = 1;

	const uint8_t rc = _vl53l7cx_poll_for_mcu_boot(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_ERROR, "the transport failure stays the status");
	zassert_not_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR);
}
