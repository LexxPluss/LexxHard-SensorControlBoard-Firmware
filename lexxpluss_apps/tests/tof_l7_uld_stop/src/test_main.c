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

static int fail_reads;
static uint8_t identity_device_id;
static uint8_t identity_revision_id;

static void bus_reset(void)
{
	status0_before = 0x00;
	status0_after = 0x80;
	status1_value = 0x84;
	confirm_after_polls = -1;
	polls_seen = 0;
	waits_seen = 0;
	fail_reads = 0;
	identity_device_id = 0;
	identity_revision_id = 0;
}

/* MODELS THE REAL PORT'S FAILURE, which the rest of this fake does not: read_chunk() in
 * zephyr/platform.c hands the caller's buffer straight to i2c_transfer() and returns its error, so
 * a read that fails leaves those bytes EXACTLY as they were. The zeroing below is convenience for
 * the success paths and would hide anything that reads an unwritten buffer. */

static uint8_t read_register(uint16_t reg, uint8_t *dst, uint32_t len)
{
	if (fail_reads) {
		(void)reg;
		(void)len;
		return VL53L7CX_STATUS_ERROR;   /* dst deliberately untouched */
	}

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
	/* The two identity registers vl53l7cx_is_alive() reads. */
	if (reg == 0U && len >= 1U) {
		dst[0] = identity_device_id;
		return VL53L7CX_STATUS_OK;
	}
	if (reg == 1U && len >= 1U) {
		dst[0] = identity_revision_id;
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

/* THE ULD'S STATUS CONSTANTS ARE NOT DISJOINT BIT FLAGS, which matters for how these cases assert.
 * VL53L7CX_STATUS_ERROR is 0xFF and therefore contains VL53L7CX_STATUS_TIMEOUT_ERROR's 0x01, so
 * `rc & TIMEOUT_ERROR` is true of the generic failure as well and cannot tell the two apart. The
 * discriminators used below are the EXACT value and the poll count -- a timeout is the only outcome
 * that spends the five-second budget, and that is a property of the run rather than of a bit. */

/* THE ONE THIS SUITE EXISTS FOR. Without patch 0002 this returns VL53L7CX_STATUS_OK after five
 * seconds of polling, and a caller that believes it reports a sensor it never stopped. */
ZTEST(tof_l7_uld_stop, test_a_stop_that_never_confirms_is_a_failure_not_a_timeout_shaped_success)
{
	confirm_after_polls = -1;
	status0_before = 0x00;

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_not_equal(rc, VL53L7CX_STATUS_OK,
			  "an unconfirmed stop must not read as a stopped sensor");
	zassert_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR,
		      "and it says exactly that it timed out: the device contributed 0x00, so nothing "
		      "else is folded in");
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

/* THE DEVICE BYTE IS NOT CARRIED, and this case is where that was decided.
 *
 * It used to assert `rc == 0x42` -- "where the device gave a code, that code is what comes back".
 * That promise cannot be kept, because the caller classifies this value by EXACT MATCH and the
 * ULD's own codes share the 8 bits with the device byte: 0x42 is also VL53L7CX_MCU_ERROR, which is
 * precisely why testing this value proved nothing about whether a device code survived. A status 1
 * of 0x01 would have come back as 0x01 and been classified as a TIMEOUT by a branch whose own
 * comment says it must not be. So the classification wins and the device byte is dropped. */
ZTEST(tof_l7_uld_stop, test_an_unrecognised_status_1_reports_the_generic_failure_not_its_own_byte)
{
	confirm_after_polls = 1;
	status1_value = 0x42;

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_ERROR,
		      "an unaccepted status 1 is the generic failure, whatever byte carried it");
	zassert_not_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR,
			  "this one is not a timeout and must not be labelled as one");
}

/* THE THREE VALUES THAT COLLIDE WITH ULD CODES, which the suite had no case for and which are the
 * whole reason the OR had to go. Each of these, folded into the status, would have been classified
 * as something it is not: 0x01 as a timeout, 0x02 as a corrupted frame, 0x7F as caller misuse. */
ZTEST(tof_l7_uld_stop, test_a_status_1_that_collides_with_a_uld_code_is_not_classified_as_that_code)
{
	const uint8_t colliding[] = {
		VL53L7CX_STATUS_TIMEOUT_ERROR,   /* 0x01 -- answered promptly, not a timeout */
		VL53L7CX_STATUS_CORRUPTED_FRAME, /* 0x02 */
		VL53L7CX_STATUS_INVALID_PARAM,   /* 0x7F -- would read as caller misuse */
	};

	for (size_t i = 0; i < ARRAY_SIZE(colliding); ++i) {
		before(NULL);
		confirm_after_polls = 1;
		status1_value = colliding[i];

		const uint8_t rc = vl53l7cx_stop_ranging(&dev);

		zassert_equal(rc, VL53L7CX_STATUS_ERROR,
			      "status 1 0x%02x came back as 0x%02x instead of the generic failure",
			      colliding[i], rc);
		zassert_true(polls_seen <= 3,
			     "the device stopped promptly, so no timeout budget was spent: %d",
			     polls_seen);
	}
}

/* And the same collision on the timeout path. A device still answering 0x02 when the budget runs
 * out timed out; before this it came back as 0x03 and was classified as a generic I/O error, with
 * the one fact the caller needed -- that five seconds were spent -- lost. */
ZTEST(tof_l7_uld_stop, test_a_timeout_is_a_timeout_whatever_the_device_was_last_answering)
{
	const uint8_t last_byte[] = {0x00, 0x01, 0x02, 0x7F};

	for (size_t i = 0; i < ARRAY_SIZE(last_byte); ++i) {
		before(NULL);
		confirm_after_polls = -1;
		status0_before = last_byte[i];

		const uint8_t rc = vl53l7cx_stop_ranging(&dev);

		zassert_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR,
			      "last status 0 byte 0x%02x gave 0x%02x instead of the timeout status",
			      last_byte[i], rc);
		zassert_true(polls_seen > 500, "the full poll budget was spent: %d", polls_seen);
	}
}

/* THE SECOND PLACE THE SAME MISTAKE LIVED, and the one the case above walked straight past.
 *
 * When the device HAS stopped, the ULD reads G02 status 1 and treats anything other than 0x84 or
 * 0x85 as wrong -- then records it with `status |= tmp`. A status 1 of 0x00 is not 0x84 and not
 * 0x85, so it takes the failure branch, contributes nothing, and the function returns OK for a stop
 * whose own reported state the ULD had just rejected. Testing 0x42 proved the branch was reached;
 * it could not prove the branch reported anything. */
ZTEST(tof_l7_uld_stop, test_a_zero_status_1_is_a_failure_and_not_a_timeout)
{
	confirm_after_polls = 1;
	status1_value = 0x00;

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_not_equal(rc, VL53L7CX_STATUS_OK,
			  "the ULD rejected the device's own reported state: that is not success");
	zassert_equal(rc, VL53L7CX_STATUS_ERROR,
		      "with nothing from the device to carry, it is the generic failure");
	zassert_not_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR,
			  "and it is NOT the timeout status: the device answered promptly");
	zassert_true(polls_seen <= 3,
		     "which the run shows too -- it stopped, so the budget was not spent: %d",
		     polls_seen);
}

/* Both accepted values, explicitly, so the substitution cannot creep into the success path. 0x85
 * is already exercised by the late-confirm case; it is asserted here too because that case is about
 * timing and this one is about the acceptance set. */
ZTEST(tof_l7_uld_stop, test_the_two_accepted_status_1_values_are_still_success)
{
	confirm_after_polls = 1;

	status1_value = 0x84;
	zassert_equal(vl53l7cx_stop_ranging(&dev), VL53L7CX_STATUS_OK, "0x84");

	before(NULL);
	confirm_after_polls = 1;
	status1_value = 0x85;
	zassert_equal(vl53l7cx_stop_ranging(&dev), VL53L7CX_STATUS_OK, "0x85");
}

/* ---- is_alive must not answer out of memory nobody wrote ---- */

/* Leaves 0xF0 and 0x02 adjacent on the stack, which is what a real sensor's identity reads put
 * there. Not marked inline or static-noinline-dependent: what matters is that it runs and returns
 * before the call under test, at a comparable depth, the way one loop iteration precedes the next
 * in the recovery pass. */
static volatile uint8_t sink;

static void leave_an_identity_on_the_stack(void)
{
	uint8_t crumbs[64];
	for (size_t i = 0; i + 1 < sizeof crumbs; i += 2) {
		crumbs[i] = 0xF0U;
		crumbs[i + 1] = 0x02U;
	}
	sink = crumbs[0];
}

/* PATCH 0003. device_id and revision_id are plain locals, the function issues all four of its
 * transfers with no early return, and the port does not clear a buffer whose read failed -- so
 * *p_is_alive was computed by comparing whatever was on the stack against 0xF0 and 0x02.
 *
 * That matters because the recovery pass probes six addresses in a loop through the same frame: a
 * sensor that IS there leaves those two values behind, and the next empty address can read its
 * answer out of the previous one's leftovers and be reported alive. The pass's census is evidence,
 * so a false positive argues for a hypothesis nobody tested.
 *
 * WHAT THIS CASE DOES NOT DO, measured rather than assumed: it does NOT fail when patch 0003 is
 * removed. That was tried -- unregistering the patch leaves this suite fully green -- because the
 * priming below does not reliably land 0xF0 and 0x02 in the two slots the compiler happens to give
 * those locals. Reading uninitialised memory cannot be pinned by a test that must produce a
 * specific wrong answer to fail, so the patch rests on the code: the locals are uninitialised, all
 * four transfers are issued regardless, and the port leaves a failed read's buffer untouched.
 *
 * What this case does pin is the property the patch guarantees and the deterministic half of the
 * pair below it: with nothing acknowledged the probe says not alive, and with a real identity it
 * still says alive. The priming stays because it costs nothing and makes the mechanism legible. */
ZTEST(tof_l7_uld_stop, test_is_alive_reports_not_alive_when_no_identity_was_read)
{
	fail_reads = 1;
	leave_an_identity_on_the_stack();

	uint8_t answered = 0xFFU;
	const uint8_t rc = vl53l7cx_is_alive(&dev, &answered);

	zassert_not_equal(rc, VL53L7CX_STATUS_OK, "every transfer failed, so the status must say so");
	zassert_equal(answered, 0U,
		      "the probe answered %u from bytes no read ever wrote", answered);
}

/* And a device that does identify is still reported alive, so the initialisation did not simply
 * pin the answer to zero. */
ZTEST(tof_l7_uld_stop, test_is_alive_still_recognises_a_real_identity)
{
	/* The fake's success path zeroes, so drive the two identity registers explicitly. */
	fail_reads = 0;
	identity_device_id = 0xF0U;
	identity_revision_id = 0x02U;

	uint8_t answered = 0U;
	const uint8_t rc = vl53l7cx_is_alive(&dev, &answered);

	zassert_equal(rc, VL53L7CX_STATUS_OK, "");
	zassert_equal(answered, 1U, "a real L7 identity must still read as alive");

	identity_device_id = 0U;
	identity_revision_id = 0U;
}
