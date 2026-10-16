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
static int fail_read_reg;        /* -1 = none */
/* Status 0 reads succeed this many times and then start failing. -1 = never. Lets a case put a
 * transport fault PART WAY through the poll, which is the only shape that can tell a recorded
 * value apart from a stale one. */
static int fail_status0_after;
/* THE I2C CALL SEQUENCE, not just its length.
 *
 * "The observer changes nothing" is a claim about WHICH transfers happen in WHICH order, and two
 * runs can agree on the count while disagreeing on the order or the register. Counting was what the
 * first version of that case compared, so it would have passed a record that read 0x07 instead of
 * 0x06 somewhere. Every fake appends here, including the waits, because a wait moved or dropped is
 * as much a behaviour change as a transfer. */
enum trace_kind { TRACE_RD, TRACE_WR, TRACE_WAIT };

struct trace_entry {
	uint8_t kind;
	uint16_t reg;   /* the wait's argument, in ms, for TRACE_WAIT */
};

enum { TRACE_CAPACITY = 4096 };

static struct trace_entry trace[TRACE_CAPACITY];
static int trace_len;
static int trace_overflowed;

static void trace_note(enum trace_kind kind, uint16_t reg)
{
	if (trace_len >= (int)TRACE_CAPACITY) {
		trace_overflowed = 1;
		return;
	}
	trace[trace_len].kind = (uint8_t)kind;
	trace[trace_len].reg = reg;
	++trace_len;
}

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
	fail_read_reg = -1;
	fail_status0_after = -1;
	identity_device_id = 0;
	identity_revision_id = 0;
	trace_len = 0;
	trace_overflowed = 0;
}

/* MODELS THE REAL PORT'S FAILURE, which the rest of this fake does not: read_chunk() in
 * zephyr/platform.c hands the caller's buffer straight to i2c_transfer() and returns its error, so
 * a read that fails leaves those bytes EXACTLY as they were. The zeroing below is convenience for
 * the success paths and would hide anything that reads an unwritten buffer. */

static uint8_t read_register(uint16_t reg, uint8_t *dst, uint32_t len)
{
	if (fail_reads || (fail_read_reg >= 0 && reg == (uint16_t)fail_read_reg)) {
		(void)len;
		return VL53L7CX_STATUS_ERROR;   /* dst deliberately untouched */
	}

	if (reg == REG_G02_STATUS_0 && fail_status0_after >= 0 && polls_seen >= fail_status0_after) {
		/* BEFORE the zeroing below, which is convenience for the success paths and would
		 * otherwise hand back a written buffer for a transfer that failed -- exactly the
		 * confusion these cases exist to rule out. */
		(void)len;
		return VL53L7CX_STATUS_ERROR;
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
	trace_note(TRACE_RD, reg);
	return read_register(reg, value, 1U);
}

uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size)
{
	(void)platform;
	trace_note(TRACE_RD, reg);
	return read_register(reg, values, size);
}

uint8_t VL53L7CX_WrByte(VL53L7CX_Platform *platform, uint16_t reg, uint8_t value)
{
	(void)platform; (void)value;
	trace_note(TRACE_WR, reg);
	return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size)
{
	(void)platform; (void)values; (void)size;
	trace_note(TRACE_WR, reg);
	return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform *platform, uint32_t time_ms)
{
	(void)platform;
	trace_note(TRACE_WAIT, (uint16_t)time_ms);
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

/* A TRANSPORT FAILURE MUST NOT BE REPORTED AS A HANDSHAKE TIMEOUT, which is the half of the
 * layer rule that lives in this patch rather than in the adapter.
 *
 * The 2026-10-12 bench run returned -5 from the quiesce and nothing else, and the first reading of
 * it was "the MCU stop was not confirmed". These two cases pin why the status byte cannot be read
 * that way on its own: a read that fails leaves the polled byte untouched, so the loop runs to its
 * bound exactly as a silent device would, and the patch's `status == OK` guard is the only thing
 * keeping the transport error from being overwritten with TIMEOUT_ERROR.
 *
 * THEY ALSO PIN WHY THE DURATION IS NOT A DISCRIMINATOR: the poll budget is spent in full here too,
 * so "it took five seconds" is true of both layers and separates neither. */
ZTEST(tof_l7_uld_stop, test_a_failed_poll_read_reports_the_transport_error_not_a_timeout)
{
	confirm_after_polls = -1;
	status0_before = 0x00;
	fail_read_reg = REG_G02_STATUS_0;   /* the polled status read NACKs; everything else answers */

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_ERROR,
		      "a stop whose poll could not be read is a transport failure and keeps that value");
	zassert_not_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR,
			  "the patch's `status == OK` guard is what stops the timeout from overwriting "
			  "an error that had already been recorded");
	/* COUNTED THROUGH THE WAIT, NOT THROUGH THE POLL. `polls_seen` is incremented on
	 * read_register()'s SUCCESS path, so the read this case makes fail never reaches it -- the
	 * first version of this assertion asked the one counter the scenario had just disabled. The
	 * WaitMs fake is called once per iteration and does not fail, so it counts the iterations
	 * that actually happened. */
	zassert_equal(polls_seen, 0, "no poll read succeeded, so that counter cannot measure this");
	zassert_true(waits_seen > 500,
		     "and it still spent the whole five seconds, which is why the duration tells the "
		     "two layers apart no better than the OR did: %d waits",
		     waits_seen);
}

ZTEST(tof_l7_uld_stop, test_a_bus_that_answers_nothing_is_also_not_a_timeout)
{
	/* The blunter version: every read fails, including the auto-stop flag that decides whether the
	 * MCU stop is provoked at all. Still an error, still not the timeout code. */
	confirm_after_polls = -1;
	fail_reads = 1;

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_ERROR);
	zassert_not_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR);
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

/* THE REAL ULD PATH THE ADAPTER HAS TO SURVIVE, asked for in review and worth having here rather
 * than only against a faked ULD: a device that answers one identity read and refuses the other.
 *
 * The point is what the ULD reports for it. The device-id read gives 0xF0, the revision-id read
 * fails and -- because of patch 0003 -- leaves a written zero, so the two do not both match and the
 * alive flag comes back ZERO. The status is non-OK, which is the only signal the ULD gives, and it
 * is collapsed into one byte that the port's first-error already covers.
 *
 * So the ULD cannot tell the adapter the difference between this and an address where nothing
 * answered: both arrive as alive=0 with a NACK. That is why the adapter asks the port how many
 * transactions COMPLETED instead of reading the alive flag. This case pins the premise. */
ZTEST(tof_l7_uld_stop, test_is_alive_reports_not_alive_when_only_one_identity_byte_is_read)
{
	identity_device_id = 0xF0U;
	identity_revision_id = 0x02U;
	fail_read_reg = 1;   /* the revision id read NACKs; the device id read succeeds */

	uint8_t answered = 0xFFU;
	const uint8_t rc = vl53l7cx_is_alive(&dev, &answered);

	zassert_not_equal(rc, VL53L7CX_STATUS_OK, "one transfer failed, so the status must say so");
	zassert_equal(answered, 0U,
		      "a half-read identity must not read as a recognised sensor");
}

/* ---- patch 0005: what GO2 status said while the stop handshake waited -----------------------
 *
 * These drive the SAME patched function over the same register map and assert on the structure it
 * fills. The reason they belong here rather than beside the pure record is that the properties
 * being pinned are properties of the patch: which reads it records, that it records none of them
 * twice, and that attaching the observer changes neither the transfers nor the verdict.
 */

static struct vl53l7cx_go2_observation obs;

static void observe(void)
{
	/* Poisoned, not zeroed. A patch that forgot to clear the structure on entry would pass a
	 * zero-initialised check by accident; it cannot pass this one. */
	memset(&obs, 0xA5, sizeof obs);
	dev.platform.go2 = &obs;
}

ZTEST(tof_l7_uld_stop, test_a_stop_with_no_observer_behaves_exactly_as_before)
{
	/* The production image attaches nothing, so the null path is the one most code runs. It must
	 * reach the same verdict with the same number of transfers. */
	confirm_after_polls = -1;
	status0_before = 0x00;
	dev.platform.go2 = NULL;

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);
	const int polls_without = polls_seen;
	const int waits_without = waits_seen;

	before(NULL);
	confirm_after_polls = -1;
	status0_before = 0x00;
	observe();

	const uint8_t rc_with = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc_with, rc, "the observer must not change the verdict");
	zassert_equal(polls_seen, polls_without, "nor add a read: %d vs %d", polls_seen, polls_without);
	zassert_equal(waits_seen, waits_without, "nor add a wait: %d vs %d", waits_seen, waits_without);
}

/* THE 2026-10-12 SHAPE. Every poll answered, bit 7 never set, the budget spent -- the run this
 * patch was written for, and the bytes it could not report. */
ZTEST(tof_l7_uld_stop, test_a_handshake_timeout_records_what_the_device_kept_answering)
{
	confirm_after_polls = -1;
	status0_before = 0x01;
	observe();

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR);
	zassert_equal(obs.exit_reason, VL53L7CX_GO2_EXIT_EXHAUSTED);
	zassert_equal(obs.first, 0x01, "the first answer, not an average or a last-one-wins");
	zassert_true(obs.first_valid);
	zassert_equal(obs.last, 0x01);
	zassert_true(obs.last_valid);
	zassert_equal(obs.polls, obs.valid_polls, "every read answered, which is what port_errno 0 said");
	zassert_true(obs.polls > 500, "the full budget: %u", obs.polls);
	zassert_false(obs.saw_mcu_stop, "and bit 7 never came up, which is the whole finding");
	zassert_true(obs.after_loop_valid, "the read after the loop is part of the sequence already");
	zassert_equal(obs.after_loop, 0x01);
	zassert_false(obs.status1_valid, "status 1 is reached only when bit 7 is set");
}

ZTEST(tof_l7_uld_stop, test_a_confirmed_stop_records_the_bit_and_the_exit_that_saw_it)
{
	confirm_after_polls = 3;
	status0_before = 0x01;
	status0_after = 0x81;
	status1_value = 0x84;
	observe();

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_OK);
	zassert_equal(obs.exit_reason, VL53L7CX_GO2_EXIT_CONFIRMED);
	zassert_true(obs.saw_mcu_stop);
	zassert_equal(obs.first, 0x01, "what it answered before it stopped");
	zassert_equal(obs.last, 0x81, "and the answer that ended the wait");
	zassert_true(obs.polls < 10U, "it did not spend the budget: %u", obs.polls);
	zassert_true(obs.status1_valid, "bit 7 was set, so status 1 was read");
	zassert_equal(obs.status1, 0x84);
}

ZTEST(tof_l7_uld_stop, test_reads_that_never_answered_leave_no_value_behind)
{
	/* The failure mode the validity flags exist for: a read that fails leaves the caller's buffer
	 * untouched, so without them an unread register is indistinguishable from one reading 0x00. */
	confirm_after_polls = -1;
	fail_read_reg = REG_G02_STATUS_0;
	observe();

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_ERROR);
	zassert_true(obs.polls > 500, "the reads were attempted: %u", obs.polls);
	zassert_equal(obs.valid_polls, 0U, "and none of them answered");
	zassert_false(obs.first_valid);
	zassert_false(obs.last_valid);
	zassert_equal(obs.first, 0U, "zero because the clear-on-entry put it there, not because the "
				     "device said zero");
	zassert_false(obs.saw_mcu_stop, "a bit cannot be seen in a byte that was never read");
}

ZTEST(tof_l7_uld_stop, test_a_failure_part_way_through_keeps_the_answers_that_arrived)
{
	/* Four reads answer and the rest fail. `last` must be the fourth answer -- not the untouched
	 * buffer the failing reads left behind, which still holds that same byte and would make a
	 * record-unconditionally version look correct. The counts are what separate them. */
	confirm_after_polls = -1;
	status0_before = 0x11;
	fail_status0_after = 4;
	observe();

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_ERROR, "the transport error is kept, not overwritten");
	zassert_equal(obs.valid_polls, 4U, "exactly the reads that answered");
	zassert_true(obs.polls > 500, "while the loop still ran to its bound: %u", obs.polls);
	zassert_equal(obs.first, 0x11);
	zassert_true(obs.first_valid);
	zassert_equal(obs.last, 0x11);
	zassert_true(obs.last_valid);
	zassert_false(obs.after_loop_valid, "the read after the loop failed too, so it recorded nothing");
}

ZTEST(tof_l7_uld_stop, test_each_stop_clears_the_observation_it_is_about_to_fill)
{
	/* An observation that survived into the next call would be the stale-evidence failure this
	 * whole line of work exists to remove. The second stop here confirms quickly, so almost every
	 * field from the first would be wrong if carried over. */
	confirm_after_polls = -1;
	status0_before = 0x01;
	observe();
	(void)vl53l7cx_stop_ranging(&dev);
	zassert_equal(obs.exit_reason, VL53L7CX_GO2_EXIT_EXHAUSTED);
	const uint16_t first_polls = obs.polls;

	before(NULL);
	confirm_after_polls = 2;
	status0_before = 0x01;
	status0_after = 0x81;
	dev.platform.go2 = &obs;   /* the SAME structure, deliberately not re-poisoned */

	(void)vl53l7cx_stop_ranging(&dev);

	zassert_equal(obs.exit_reason, VL53L7CX_GO2_EXIT_CONFIRMED);
	zassert_true(obs.polls < first_polls, "%u carried over from %u", obs.polls, first_polls);
	zassert_true(obs.saw_mcu_stop);
}

ZTEST(tof_l7_uld_stop, test_a_stop_that_never_polled_is_not_a_poll_that_saw_nothing)
{
	/* is_auto_stop_enabled selects a path that skips the poll entirely. Reporting that as zero
	 * polls is only truthful because the exit reason names it; the two are otherwise identical
	 * to a reader. */
	dev.is_auto_stop_enabled = 1U;
	observe();

	(void)vl53l7cx_stop_ranging(&dev);

	zassert_equal(obs.exit_reason, VL53L7CX_GO2_EXIT_NOT_POLLED);
	zassert_equal(obs.polls, 0U);
	zassert_equal(obs.valid_polls, 0U);
	zassert_false(obs.first_valid);
}

/* ---- what the record must not do to the sequence it records -------------------------------- */

/* THE KNOBS THIS FAKE HAS, as one value, so a scenario can be replayed identically twice. */
struct scenario {
	const char *name;
	uint8_t s0_before;
	uint8_t s0_after;
	uint8_t s1;
	int confirm_after;
	int fail_all_reads;
	int fail_reg;
	int fail_s0_from_poll;
};

static void apply(const struct scenario *s)
{
	status0_before = s->s0_before;
	status0_after = s->s0_after;
	status1_value = s->s1;
	confirm_after_polls = s->confirm_after;
	fail_reads = s->fail_all_reads;
	fail_read_reg = s->fail_reg;
	fail_status0_after = s->fail_s0_from_poll;
}

ZTEST(tof_l7_uld_stop, test_the_observer_moves_no_transfer_in_any_of_these_runs)
{
	/* "The record changes nothing" is a claim about WHICH transfers happen in WHICH order, and the
	 * first version of this comparison checked only how MANY -- which a record that read 0x07 where
	 * it used to read 0x06 would have passed. Each scenario is therefore run twice, once with no
	 * observer and once with one, and the whole trace is compared entry by entry along with the
	 * wait count and the verdict.
	 *
	 * The scenarios are the ones whose paths differ: a stop that never confirms, one that confirms
	 * at once, one that confirms late, a bus that answers nothing, a status 1 that is rejected, and
	 * a status 1 read that fails. */
	static const struct scenario scenarios[] = {
		{ "never confirms",        0x00, 0x80, 0x84, -1,  0, -1, -1 },
		{ "confirms at once",      0x00, 0x80, 0x84,  1,  0, -1, -1 },
		{ "confirms late",         0x00, 0x80, 0x84, 400, 0, -1, -1 },
		{ "nothing answers",       0x00, 0x80, 0x84, -1,  1, -1, -1 },
		{ "status 1 rejected",     0x00, 0x80, 0x5A,  1,  0, -1, -1 },
		{ "status 1 read fails",   0x00, 0x80, 0x84,  1,  0, REG_G02_STATUS_1, -1 },
		{ "polls stop answering",  0x00, 0x80, 0x84, -1,  0, -1, 10 },
	};

	static struct trace_entry without[TRACE_CAPACITY];

	for (size_t i = 0; i < ARRAY_SIZE(scenarios); ++i) {
		const struct scenario *const s = &scenarios[i];

		before(NULL);
		apply(s);
		dev.platform.go2 = NULL;
		const uint8_t rc_without = vl53l7cx_stop_ranging(&dev);
		const int len_without = trace_len;
		const int waits_without = waits_seen;
		zassert_false(trace_overflowed, "%s: the trace did not fit, so it proves nothing", s->name);
		memcpy(without, trace, sizeof(struct trace_entry) * (size_t)len_without);

		before(NULL);
		apply(s);
		observe();
		const uint8_t rc_with = vl53l7cx_stop_ranging(&dev);

		zassert_false(trace_overflowed, "%s: the trace did not fit", s->name);
		zassert_equal(rc_with, rc_without, "%s: the verdict moved, 0x%02x against 0x%02x",
			      s->name, rc_with, rc_without);
		zassert_equal(waits_seen, waits_without, "%s: %d waits against %d", s->name, waits_seen,
			      waits_without);
		zassert_equal(trace_len, len_without, "%s: %d transfers against %d", s->name, trace_len,
			      len_without);
		for (int at = 0; at < len_without; ++at) {
			zassert_equal(trace[at].kind, without[at].kind,
				      "%s: entry %d changed kind", s->name, at);
			zassert_equal(trace[at].reg, without[at].reg,
				      "%s: entry %d is register 0x%04x, was 0x%04x", s->name, at,
				      trace[at].reg, without[at].reg);
		}
		zassert_true(len_without > 0, "%s: nothing happened, so nothing was compared", s->name);
	}
}

ZTEST(tof_l7_uld_stop, test_a_status_1_read_that_failed_leaves_no_status_1_behind)
{
	/* The read that fails is the LAST one of the sequence, so there is a fully populated record in
	 * front of it. A patch that stored the byte before checking the read would leave `status1`
	 * holding whatever 0x06 last put in `tmp` -- a plausible value, attached to a transfer that
	 * never happened. */
	confirm_after_polls = 1;
	status0_before = 0x00;
	status0_after = 0x80;
	fail_read_reg = REG_G02_STATUS_1;
	observe();

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_ERROR, "the transport failure is the verdict");
	zassert_equal(obs.status1_valid, 0, "no status 1 was read, so none may be reported");
	zassert_equal(obs.status1, 0, "and the field stays as the entry clear left it, not 0x80");
	zassert_equal(obs.after_loop_valid, 1, "the read before it did answer");
	zassert_equal(obs.after_loop, 0x80);
	zassert_equal(obs.saw_mcu_stop, 1);
}

ZTEST(tof_l7_uld_stop, test_a_confirmation_on_the_last_allowed_poll_is_recorded_though_it_times_out)
{
	/* THE EDGE THE TWO FIELDS EXIST TO SEPARATE. The timeout is checked after the read and before
	 * the loop condition, so a device that sets bit 7 on poll 501 still leaves through the timeout
	 * branch -- upstream behaviour, unchanged here. What must not happen is the RECORD agreeing
	 * that nothing was seen: `exit_reason` says how the loop ended, `saw_mcu_stop` says what the
	 * device said, and this is the run where they disagree. */
	confirm_after_polls = 501;
	status0_before = 0x00;
	status0_after = 0x80;
	status1_value = 0x84;
	observe();

	const uint8_t rc = vl53l7cx_stop_ranging(&dev);

	zassert_equal(rc, VL53L7CX_STATUS_TIMEOUT_ERROR, "the verdict is upstream's and does not move");
	zassert_equal(obs.exit_reason, VL53L7CX_GO2_EXIT_EXHAUSTED, "the loop did run out");
	zassert_equal(obs.saw_mcu_stop, 1, "and the bit WAS seen, on the last poll it allowed");
	zassert_equal(obs.last, 0x80, "which is the last thing the device answered");
	zassert_equal(obs.last_valid, 1);
	zassert_equal(obs.first, 0x00, "while the first answer was still the waiting one");
	zassert_equal(obs.valid_polls, 501);
	zassert_equal(obs.polls, 501);
}

