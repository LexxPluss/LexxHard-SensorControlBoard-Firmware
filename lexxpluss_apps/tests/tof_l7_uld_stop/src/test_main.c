/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * vl53l7cx_stop_ranging() against a bus the test controls, with the REAL vendor function.
 *
 * WHY THIS SUITE HAD TO EXIST. tests/tof_l7_sensor substitutes the whole ULD, which is the right
 * shape for testing the adapter and is precisely why it cannot see this: the behaviour in question
 * is inside vl53l7cx_stop_ranging() itself, and a suite that replaces that function is testing its
 * own replacement. The defect it hides is not hypothetical -- on timeout the upstream function does
 * `status |= tmp`, where tmp is the last byte polled and the loop can only still be running because
 * that byte's bit 7 was clear. A device answering 0x00 therefore leaves the status untouched and a
 * five-second wait returns VL53L7CX_STATUS_OK.
 *
 * That answer is the one the boot-time recovery pass must never be given: it turns an unrecovered
 * survivor into a clean boot record, and the next failure is attributed somewhere else.
 *
 * WHAT IS FAKE HERE, STATED PLAINLY. The bus is. The platform entry points below are a small
 * register map, not our Zephyr port -- that port has its own suite in tests/tof_l7_port, and mixing
 * the two would leave a failure here ambiguous between the vendor's logic and our transport. What
 * is real is the vendor source under test, compiled from the patched copy exactly as the product
 * compiles it.
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "vl53l7cx_api.h"

/* ---- the bus ---- */

/* G02 status 0. Bit 7 set is "the MCU stopped"; the ULD polls this and nothing else to decide. */
#define REG_G02_STATUS_0 0x0006
#define REG_G02_STATUS_1 0x0007
#define REG_AUTO_STOP    0x2FFC

static uint8_t status0_before;   /* what 0x6 reads until confirm_after_polls polls have happened */
static uint8_t status0_after;    /* and after */
static uint8_t status1_value;    /* what 0x7 reads, consulted only when 0x6 has bit 7 set */
static int confirm_after_polls;  /* -1 = never confirms */
static int polls_seen;
static int waits_seen;

static void bus_reset(void)
{
	status0_before = 0x00;
	status0_after = 0x80;
	status1_value = 0x84;
	confirm_after_polls = -1;
	polls_seen = 0;
	waits_seen = 0;
}

static uint8_t read_register(uint16_t reg, uint8_t *dst, uint32_t len)
{
	memset(dst, 0, len);

	if (reg == REG_AUTO_STOP) {
		/* Anything other than 0x4FF, so a zeroed configuration takes the provoke-MCU-stop
		 * path -- which is the path the recovery pass always takes, because the
		 * configuration it hands the ULD is freshly zeroed every time. */
		return VL53L7CX_STATUS_OK;
	}
	if (reg == REG_G02_STATUS_0 && len >= 1U) {
		++polls_seen;
		const bool confirmed =
			confirm_after_polls >= 0 && polls_seen >= confirm_after_polls;
		dst[0] = confirmed ? status0_after : status0_before;
		return VL53L7CX_STATUS_OK;
	}
	if (reg == REG_G02_STATUS_1 && len >= 1U) {
		dst[0] = status1_value;
		return VL53L7CX_STATUS_OK;
	}
	return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_RdByte(VL53L7CX_Platform *platform, uint16_t reg, uint8_t *value)
{
	(void)platform;
	return read_register(reg, value, 1U);
}

uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size)
{
	(void)platform;
	return read_register(reg, values, size);
}

uint8_t VL53L7CX_WrByte(VL53L7CX_Platform *platform, uint16_t reg, uint8_t value)
{
	(void)platform; (void)reg; (void)value;
	return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size)
{
	(void)platform; (void)reg; (void)values; (void)size;
	return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform *platform, uint32_t time_ms)
{
	(void)platform; (void)time_ms;
	++waits_seen;
	return VL53L7CX_STATUS_OK;
}

void VL53L7CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
	for (uint16_t i = 0; i < size; i += 4U) {
		uint32_t tmp;

		memcpy(&tmp, &buffer[i], sizeof(tmp));
		tmp = __builtin_bswap32(tmp);
		memcpy(&buffer[i], &tmp, sizeof(tmp));
	}
}

/* ---- the device object, exactly as the recovery pass builds one ---- */

static VL53L7CX_Configuration dev;

static void before(void *unused)
{
	(void)unused;
	bus_reset();
	/* Zeroed, with only the address filled in. That is the whole state the recovery pass has
	 * after a reset, and is_auto_stop_enabled being zero is what selects the polling path. */
	memset(&dev, 0, sizeof dev);
	dev.platform.address = 0x52;
}

ZTEST_SUITE(tof_l7_uld_stop, NULL, NULL, before, NULL, NULL);

/* THE ONE THIS SUITE EXISTS FOR. Without patch 0002 this returns VL53L7CX_STATUS_OK after five
 * seconds of polling, and a caller that believes it reports a sensor it never stopped. */
ZTEST(tof_l7_uld_stop, test_a_stop_that_never_confirms_is_a_failure_not_a_timeout_shaped_success)
{
	confirm_after_polls = -1;
	status0_before = 0x00;

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_not_equal(rc, VL53L7CX_STATUS_OK,
			  "an unconfirmed stop must not read as a stopped sensor");
	zassert_true((rc & VL53L7CX_STATUS_TIMEOUT_ERROR) != 0,
		     "and it must say it timed out rather than carrying some other code");
	zassert_true(polls_seen > 500, "the full poll budget was spent: %d", polls_seen);
}

/* The byte a silent device produces through a zeroing port is 0x00, and that is also the ordinary
 * "not stopped yet" reading -- which is exactly why `status |= tmp` could not see it. Pinned with a
 * second value that is equally invisible to an OR. */
ZTEST(tof_l7_uld_stop, test_a_zero_reading_is_the_case_the_or_could_not_detect)
{
	confirm_after_polls = -1;
	status0_before = 0x00;
	zassert_equal(vl53l7cx_stop_ranging(&dev) & ~VL53L7CX_STATUS_TIMEOUT_ERROR, 0U,
		      "nothing but the timeout bit distinguishes this case");
}

/* A device that does stop is still reported as stopped: the patch must not turn every stop into a
 * failure, which is the obvious way to 'fix' this and would make the recovery pass useless. */
ZTEST(tof_l7_uld_stop, test_a_stop_that_confirms_immediately_is_still_ok)
{
	confirm_after_polls = 1;
	status1_value = 0x84;

	zassert_equal(vl53l7cx_stop_ranging(&dev), VL53L7CX_STATUS_OK);
	zassert_true(polls_seen <= 3, "it stopped polling as soon as it was told: %d", polls_seen);
}

/* And one that takes a while. The timeout is five seconds for a reason; a stop that confirms on the
 * hundredth poll is a slow stop, not a failed one. */
ZTEST(tof_l7_uld_stop, test_a_stop_that_confirms_late_is_still_ok)
{
	confirm_after_polls = 100;
	status1_value = 0x85;

	zassert_equal(vl53l7cx_stop_ranging(&dev), VL53L7CX_STATUS_OK);
	zassert_true(polls_seen >= 100, "it waited: %d", polls_seen);
	zassert_true(polls_seen < 500, "but not to the budget");
}

/* A device that stops and then reports a status 1 the ULD does not recognise is a failure, and it
 * was one before this patch too. Kept so the patch is not credited with it. */
ZTEST(tof_l7_uld_stop, test_an_unrecognised_status_1_still_fails_on_its_own)
{
	confirm_after_polls = 1;
	status1_value = 0x42;

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_not_equal(rc, VL53L7CX_STATUS_OK);
	zassert_true((rc & VL53L7CX_STATUS_TIMEOUT_ERROR) == 0,
		     "this one is not a timeout and must not be labelled as one");
}
