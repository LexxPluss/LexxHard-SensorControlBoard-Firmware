/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Adapter-level tests for tof_cliff_sensor.c.
 *
 * The ULD's translation units are not compiled into this image. Driving the real ULD
 * would mean emulating the VL53L4CX register map closely enough to satisfy it, which
 * would freeze a vendor-internal register sequence into our tests and break on the next
 * ST release for no safety benefit. So the ULD entry points the adapter calls are faked
 * here, and the fakes reproduce the ULD's observable behaviour - including the two
 * defects that shape the adapter:
 *
 *   - a zero-target cycle still carries RangeData[0], because SetMeasurementData forces
 *     iteration = 1 when active_results < 1;
 *   - a call can fail on the bus and still return VL53LX_ERROR_NONE.
 *
 * The second shape is posed deliberately, and is NOT a claim about the ULD in this
 * build. VL53LX_GetMultiRangingData did assign SetMeasurementData's result over the
 * status get_device_results produced, and patch 0001 fixes exactly that. What the fakes
 * here exercise is the adapter's defence against the class: they perform a REAL port
 * transfer through the emulated controller, have it fail, and then return success. That
 * is what the sticky errno is for, and it has to keep working whether or not the vendor
 * tree is patched -- an upstream bump, or any path the patch does not cover, puts the
 * shape back. If the sticky mechanism were removed, these tests would fail rather than
 * silently pass.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "fake_i2c.h"
#include "tof_cliff_sensor.h"
#include "vl53l4cx_bus_io.h"
#include "vl53l4cx_port.h"

#define TEST_ADDR7 0x29

/* ------------------------------------------------------------- ULD fakes --------- */

static struct {
	uint8_t ready;
	VL53LX_Error ready_rc;
	VL53LX_Error fetch_rc;
	VL53LX_Error rearm_rc;
	int fetch_bus_xfers; /* real port transfers the fetch performs before returning */
	int rearm_bus_xfers; /* same, for the re-arm, so a bus failure under success can be posed */
	VL53LX_MultiRangingData_t canned;
	int ready_calls;
	int fetch_calls;
	int rearm_calls;
	/* Models the real device for the tests that need it: the frame stays available
	 * until ClearInterruptAndStartMeasurement releases it, and only then does the next
	 * measurement become the one a fetch will return. Without this the fake hands back
	 * a brand-new frame on every call, which is the one thing a missing re-arm cannot
	 * do on hardware -- and a test written against that fake pins the bug as correct. */
	bool frame_advances_only_on_rearm;
	uint8_t held_stream_count;
	int stop_calls;
	int stop_bus_xfers;
	int start_calls;
	/* lifecycle */
	VL53LX_Error boot_rc;
	VL53LX_Error data_init_rc;
	VL53LX_Error ref_spad_rc;
	VL53LX_Error mode_rc;
	VL53LX_Error budget_rc;
	VL53LX_Error start_rc;
	int32_t register_bus_io_rc;
	VL53LX_DistanceModes seen_mode;
	uint32_t seen_budget_us;
	char order[64];
} f;

static void order_add(char c)
{
	size_t n = strlen(f.order);

	if (n + 1 < sizeof(f.order)) {
		f.order[n] = c;
		f.order[n + 1] = '\0';
	}
}

VL53LX_Error VL53LX_GetMeasurementDataReady(VL53LX_DEV Dev, uint8_t *pReady)
{
	ARG_UNUSED(Dev);
	f.ready_calls++;
	order_add('r');
	*pReady = f.ready;
	return f.ready_rc;
}

VL53LX_Error VL53LX_GetMultiRangingData(VL53LX_DEV Dev, VL53LX_MultiRangingData_t *pData)
{
	f.fetch_calls++;
	order_add('f');

	/* Real traffic, so an injected bus failure is recorded by the real port. */
	for (int i = 0; i < f.fetch_bus_xfers; i++) {
		(void)VL53LX_WrByte(Dev, (uint16_t)(0x0100 + i), 0x00);
	}

	if (f.frame_advances_only_on_rearm) {
		f.canned.StreamCount = f.held_stream_count;
	}
	*pData = f.canned;
	return f.fetch_rc;
}

VL53LX_Error VL53LX_ClearInterruptAndStartMeasurement(VL53LX_DEV Dev)
{
	f.rearm_calls++;
	order_add('c');

	/* Real traffic when asked for, so the re-arm can fail on the bus while the ULD still
	 * reports success - the same posed shape as the fetch and the stop. */
	for (int i = 0; i < f.rearm_bus_xfers; i++) {
		(void)VL53LX_WrByte(Dev, (uint16_t)(0x0300 + i), 0x00);
	}
	if (f.frame_advances_only_on_rearm && f.rearm_rc == VL53LX_ERROR_NONE) {
		/* The held frame is released and the device goes on to measure the next
		 * one. A re-arm that never happens leaves the old frame in place. */
		f.held_stream_count++;
	}
	return f.rearm_rc;
}

VL53LX_Error VL53LX_WaitDeviceBooted(VL53LX_DEV Dev)
{
	ARG_UNUSED(Dev);
	order_add('B');
	return f.boot_rc;
}

VL53LX_Error VL53LX_DataInit(VL53LX_DEV Dev)
{
	ARG_UNUSED(Dev);
	order_add('D');
	return f.data_init_rc;
}

VL53LX_Error VL53LX_PerformRefSpadManagement(VL53LX_DEV Dev)
{
	ARG_UNUSED(Dev);
	order_add('S');
	return f.ref_spad_rc;
}

VL53LX_Error VL53LX_SetDistanceMode(VL53LX_DEV Dev, VL53LX_DistanceModes mode)
{
	ARG_UNUSED(Dev);
	order_add('M');
	f.seen_mode = mode;
	return f.mode_rc;
}

VL53LX_Error VL53LX_SetMeasurementTimingBudgetMicroSeconds(VL53LX_DEV Dev, uint32_t us)
{
	ARG_UNUSED(Dev);
	order_add('T');
	f.seen_budget_us = us;
	return f.budget_rc;
}

VL53LX_Error VL53LX_StartMeasurement(VL53LX_DEV Dev)
{
	ARG_UNUSED(Dev);
	f.start_calls++;
	order_add('G');
	return f.start_rc;
}

VL53LX_Error VL53LX_StopMeasurement(VL53LX_DEV Dev)
{
	f.stop_calls++;
	order_add('P');

	/* Real traffic when asked for, so a stop can fail on the bus while the ULD still
	 * reports success - the same posed shape as the fetch. */
	for (int i = 0; i < f.stop_bus_xfers; i++) {
		(void)VL53LX_WrByte(Dev, (uint16_t)(0x0200 + i), 0x00);
	}
	return VL53LX_ERROR_NONE;
}

/* The BSP wrapper is not compiled either. This mirrors what the real one does with the
 * IO block, which is all the adapter depends on. */
int32_t VL53L4CX_RegisterBusIO(VL53L4CX_Object_t *pObj, VL53L4CX_IO_t *pIO)
{
	order_add('I');
	if (f.register_bus_io_rc != 0) {
		return f.register_bus_io_rc;
	}
	pObj->IO = *pIO;
	return (pObj->IO.Init != NULL) ? pObj->IO.Init() : -1;
}

/* --------------------------------------------------------------- fixtures -------- */

static VL53L4CX_Object_t obj;
static struct tof_cliff_scratch scratch;
static struct tof_cliff_stream_state stream;
static struct tof_cliff_sample sample;
static struct tof_cliff_read_status st;

static void canned_targets(uint8_t count, const int16_t *mm, const uint8_t *status,
			   uint8_t entries)
{
	memset(&f.canned, 0, sizeof(f.canned));
	f.canned.NumberOfObjectsFound = count;
	f.canned.StreamCount = 7;
	for (uint8_t i = 0; i < entries; i++) {
		f.canned.RangeData[i].RangeMilliMeter = mm[i];
		f.canned.RangeData[i].RangeStatus = status[i];
	}
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&f, 0, sizeof(f));
	memset(&obj, 0, sizeof(obj));
	memset(&scratch, 0, sizeof(scratch));
	memset(&stream, 0, sizeof(stream));
	memset(&sample, 0, sizeof(sample));
	memset(&st, 0, sizeof(st));
	obj.IO.Address = TEST_ADDR7 << 1; /* ST's 8-bit wire convention */
	f.ready = 1;
	f.fetch_bus_xfers = 3;
	fake_i2c_reset();
	vl53l4cx_port_sticky_reset();
	vl53l4cx_bus_io_reset_placeholder_calls();
}

ZTEST_SUITE(tof_cliff_adapter, NULL, NULL, before, NULL, NULL);

/* ----------------------------------------------------- preservation rules -------- */

ZTEST(tof_cliff_adapter, test_zero_targets_keeps_the_synthetic_entry_and_a_true_zero_count)
{
	/* What the ULD really produces when it finds nothing: count 0, and RangeData[0]
	 * still filled with the status that explains the absence. */
	const int16_t mm[1] = {8191};
	const uint8_t status[1] = {255};

	canned_targets(0, mm, status, 1);

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_true(sample.fresh);
	zassert_equal(sample.target_count, 0, "a real zero must stay zero");
	zassert_equal(sample.entry_count, 1,
		      "the synthetic entry is the only record of WHY nothing was found");
	zassert_equal(sample.entries[0].range_status, 255);
	zassert_equal(sample.entries[0].range_mm, 8191);
}

ZTEST(tof_cliff_adapter, test_four_targets_are_all_copied_in_order)
{
	const int16_t mm[4] = {120, 340, 900, 1500};
	const uint8_t status[4] = {0, 4, 0, 2};

	canned_targets(4, mm, status, 4);

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(sample.target_count, 4);
	zassert_equal(sample.entry_count, 4);
	for (int i = 0; i < 4; i++) {
		zassert_equal(sample.entries[i].range_mm, mm[i]);
		zassert_equal(sample.entries[i].range_status, status[i]);
	}
	zassert_equal(sample.stream_count, 7);
}

/* This replaces test_negative_range_survives_unclamped, which posed {-37, VALID} and
 * {-1, VALID}. Those are combinations the real ULD cannot emit: SetTargetData rewrites a
 * VALID negative into either a VALID 0 mm or an INVALID negative before this layer sees
 * it, so the old fixtures proved only that the fake copied what it was handed. The two
 * shapes below are the ones the ULD does produce -- tests/tof_uld_status pins that they
 * are, by running the real SetTargetData -- and what is asserted here is the only thing
 * this layer is responsible for: it copies both fields through without a second opinion. */
ZTEST(tof_cliff_adapter, test_the_ulds_normalised_negatives_are_copied_through_unchanged)
{
	/* An INVALID negative from below the tuning threshold, and a VALID 0 mm that the
	 * ULD synthesised from a negative inside it. */
	const int16_t mm[2] = {-31, 0};
	const uint8_t status[2] = {VL53LX_RANGESTATUS_RANGE_INVALID,
				   VL53LX_RANGESTATUS_RANGE_VALID};

	canned_targets(2, mm, status, 2);

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(sample.entries[0].range_mm, -31,
		      "an INVALID negative must not be clamped away here");
	zassert_equal(sample.entries[0].range_status, VL53LX_RANGESTATUS_RANGE_INVALID);
	zassert_equal(sample.entries[1].range_mm, 0);
	zassert_equal(sample.entries[1].range_status, VL53LX_RANGESTATUS_RANGE_VALID,
		      "the ULD's synthesised zero is VALID and stays VALID");
}

ZTEST(tof_cliff_adapter, test_no_status_is_reclassified_or_reduced_here)
{
	/* Two of the classes the wire contract still lists as unresolved, 3 and 11, must
	 * arrive here untouched: freezing a classification in this layer would decide
	 * them by accident. And the contract's reduction is most-conservative-then-
	 * farthest, which cannot be recovered from a single pre-reduced number. */
	const int16_t mm[3] = {500, 250, 750};
	const uint8_t status[3] = {3, 11, 0};

	canned_targets(3, mm, status, 3);

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(sample.entry_count, 3, "no entry may be dropped by this layer");
	zassert_equal(sample.entries[0].range_status, 3);
	zassert_equal(sample.entries[1].range_status, 11);
	zassert_equal(sample.entries[2].range_status, 0);
	/* Order preserved, nothing sorted, nothing reduced to one value. */
	zassert_equal(sample.entries[0].range_mm, 500);
	zassert_equal(sample.entries[1].range_mm, 250);
	zassert_equal(sample.entries[2].range_mm, 750);
}

ZTEST(tof_cliff_adapter, test_count_above_the_array_is_a_protocol_error_not_a_truncated_sample)
{
	const int16_t mm[4] = {1, 2, 3, 4};
	const uint8_t status[4] = {0, 0, 0, 0};

	/* Only a corrupted read can produce this, and truncation would be the worst
	 * response: a caller that iterates on target_count would then walk off the end of
	 * an array holding four. Nothing about the sample is trustworthy, so none of it is
	 * published. */
	canned_targets(9, mm, status, 4);

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_FETCH);
	zassert_false(st.sample_present);
	zassert_false(sample.fresh);
	zassert_equal(sample.target_count, 0, "no part of a corrupt sample may leak out");
	zassert_equal(sample.entry_count, 0);
	zassert_equal(f.rearm_calls, 1,
		      "the fetch succeeded and the frame was refused, so it must be released");

	/* The rule cannot be bypassed by calling the pure copy step directly either. */
	zassert_equal(tof_cliff_copy_raw(&f.canned, &sample), -EPROTO);
	zassert_false(sample.fresh);

	/* The largest count the array does hold is still accepted. */
	before(NULL);
	canned_targets(TOF_CLIFF_MAX_TARGETS, mm, status, 4);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(sample.target_count, TOF_CLIFF_MAX_TARGETS);
}

ZTEST(tof_cliff_adapter, test_a_ready_flag_outside_zero_and_one_is_a_protocol_error)
{
	/* Treating it as not-ready would let a confused device pass for a merely quiet one
	 * indefinitely, and the scheduler would keep counting missed cycles instead of
	 * faulting the source. */
	for (uint8_t bogus = 2; bogus < 5; bogus++) {
		before(NULL);
		f.ready = bogus;

		zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO,
			      "ready = %u was accepted", bogus);
		zassert_equal(st.stage, TOF_CLIFF_STAGE_READY_CHECK);
		zassert_false(sample.fresh);
		zassert_equal(f.fetch_calls, 0, "nothing may be fetched on a bogus flag");
	}

	/* 0 and 1 keep their ordinary meanings. */
	before(NULL);
	f.ready = 0;
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_false(sample.fresh);
}

/* ------------------------------------------------------------- not ready --------- */

ZTEST(tof_cliff_adapter, test_not_ready_is_not_an_error_and_never_waits)
{
	f.ready = 0;

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_false(sample.fresh);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_NONE);
	zassert_false(st.sample_present);
	zassert_false(st.rearm_failed);

	/* Exactly one check, no loop: one dead sensor must not delay the other three,
	 * and the BSP's blocking poll would have spun for up to 65.5 s here. */
	zassert_equal(f.ready_calls, 1);
	zassert_equal(f.fetch_calls, 0);
	zassert_equal(f.rearm_calls, 0, "nothing to re-arm when nothing was read");
}

/* --------------------------------------------------------- error surfaces -------- */

ZTEST(tof_cliff_adapter, test_ready_check_failure_reports_its_own_stage)
{
	f.ready_rc = VL53LX_ERROR_CONTROL_INTERFACE;

	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_READY_CHECK);
	zassert_equal(st.uld_rc, VL53LX_ERROR_CONTROL_INTERFACE);
	zassert_false(st.sample_present);
	zassert_equal(f.fetch_calls, 0);
}

ZTEST(tof_cliff_adapter, test_fetch_failure_yields_no_sample)
{
	const int16_t mm[1] = {500};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	f.fetch_rc = VL53LX_ERROR_CONTROL_INTERFACE;

	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_FETCH);
	zassert_false(st.sample_present);
	zassert_false(sample.fresh, "a failed fetch must not hand up a half-copied sample");

	/* The frame the device is holding was refused, so it has to be released. Leaving it
	 * there means the next read is served the same frame: a deterministic failure then
	 * repeats forever, and a transient one ends with the pre-outage frame arriving after
	 * recovery and passing the replay guard as fresh. */
	zassert_equal(f.rearm_calls, 1, "a refused frame must still be released");

	/* And the re-arm must not overwrite what explains the refusal. */
	zassert_equal(st.uld_rc, VL53LX_ERROR_CONTROL_INTERFACE,
		      "a successful re-arm must not zero the fetch diagnosis");
	zassert_false(st.rearm_failed, "the re-arm itself succeeded");
}

ZTEST(tof_cliff_adapter, test_a_bus_failure_under_a_success_return_is_still_caught)
{
	const int16_t mm[1] = {500};
	const uint8_t status[1] = {0};
	/* First, middle and last transfer of the fetch. */
	static const int at[] = {1, 2, 3};

	for (size_t i = 0; i < ARRAY_SIZE(at); i++) {
		before(NULL);
		canned_targets(1, mm, status, 1);
		/* The shape the sticky errno defends against, posed on purpose: the
		 * transport failed and the ULD returns VL53LX_ERROR_NONE anyway. Patch
		 * 0001 removes the ULD path that used to produce this shape by itself;
		 * the defence has to hold regardless, so it is still exercised here. */
		f.fetch_rc = VL53LX_ERROR_NONE;
		fake_i2c_fail_on(at[i], -EIO);

		zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0,
				  "transfer %d failed and the read still reported success",
				  at[i]);
		zassert_equal(st.stage, TOF_CLIFF_STAGE_FETCH);
		zassert_equal(st.port_errno, -EIO, "the raw errno must reach the caller");
		zassert_equal(st.uld_rc, VL53LX_ERROR_NONE,
			      "and the ULD's own opinion is kept for triage");
		zassert_false(st.sample_present);
	}
}

ZTEST(tof_cliff_adapter, test_sticky_from_an_earlier_operation_cannot_leak_into_this_read)
{
	const int16_t mm[1] = {500};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);

	/* Something failed before this read and nobody cleared it. */
	fake_i2c_fail_on(1, -ETIMEDOUT);
	(void)VL53LX_WrByte(&obj, 0x0000, 0x00);
	zassert_equal(vl53l4cx_port_sticky_errno(), -ETIMEDOUT);
	fake_i2c_fail_on(0, 0);

	/* The read clears before each ULD call, so a stale record must not fail it. */
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(st.port_errno, 0);
	zassert_true(st.sample_present);
}

ZTEST(tof_cliff_adapter, test_rearm_failure_keeps_the_sample_and_says_so_separately)
{
	const int16_t mm[2] = {410, 890};
	const uint8_t status[2] = {0, 4};

	canned_targets(2, mm, status, 2);
	f.rearm_rc = VL53LX_ERROR_CONTROL_INTERFACE;

	/* Non-zero, so a caller that checks only the return value stops trusting this
	 * sensor - the fail-safe direction. */
	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_REARM);
	zassert_true(st.rearm_failed, "the next cycle will not arrive, and that is distinct");
	zassert_true(st.sample_present, "but this cycle's reading is real and must survive");

	/* The sample is intact: squashing "here is your reading" together with "the next
	 * one will not come" is how a chain stops ranging while every read looks fine. */
	zassert_true(sample.fresh);
	zassert_equal(sample.target_count, 2);
	zassert_equal(sample.entries[0].range_mm, 410);
	zassert_equal(sample.entries[1].range_status, 4);
}

ZTEST(tof_cliff_adapter, test_the_sticky_record_is_scoped_to_one_operation)
{
	const int16_t mm[1] = {600};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	f.fetch_bus_xfers = 3;
	f.rearm_rc = VL53LX_ERROR_NONE;
	/* The failure is armed for a fourth transfer that this read never performs. Each
	 * ULD operation clears the record before it starts and reads it back afterwards,
	 * so a failure belonging to nobody must not be attributed to this read. */
	fake_i2c_fail_on(4, -EBUSY);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(st.port_errno, 0);
	zassert_true(st.sample_present);
	zassert_false(st.rearm_failed);
}

/* ------------------------------------------------------------- lifecycle -------- */

ZTEST(tof_cliff_adapter, test_open_reports_which_step_failed_rather_than_one_error)
{
	/* The BSP's VL53L4CX_Init collapses boot, DataInit and SPAD management into one
	 * VL53L4CX_ERROR. When a chain comes up wrong, which of the three failed is the
	 * only thing that matters. */
	struct {
		VL53LX_Error *slot;
		enum tof_cliff_stage stage;
	} cases[] = {
		{&f.boot_rc, TOF_CLIFF_STAGE_BOOT},
		{&f.data_init_rc, TOF_CLIFF_STAGE_DATA_INIT},
		{&f.ref_spad_rc, TOF_CLIFF_STAGE_REF_SPAD},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		/* The pointers address fields inside f, so they stay valid across the
		 * reset that before() performs. */
		before(NULL);
		*cases[i].slot = VL53LX_ERROR_CONTROL_INTERFACE;

		zassert_not_equal(tof_cliff_sensor_open(&obj, TEST_ADDR7, &st), 0);
		zassert_equal(st.stage, cases[i].stage, "wrong stage for case %d", (int)i);
	}
}

ZTEST(tof_cliff_adapter, test_open_runs_bus_io_then_boot_then_data_init_then_spad)
{
	zassert_equal(tof_cliff_sensor_open(&obj, TEST_ADDR7, &st), 0);
	zassert_equal(strcmp(f.order, "IBDS"), 0, "order was \"%s\"", f.order);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_NONE);
}

ZTEST(tof_cliff_adapter, test_bus_io_failure_stops_before_any_device_step)
{
	f.register_bus_io_rc = -1;

	zassert_not_equal(tof_cliff_sensor_open(&obj, TEST_ADDR7, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_BUS_IO);
	zassert_equal(strcmp(f.order, "I"), 0, "order was \"%s\"", f.order);
}

ZTEST(tof_cliff_adapter, test_an_address_outside_the_unicast_range_is_refused_at_open)
{
	/* 0x00-0x07 and 0x78-0x7F are reserved by I2C itself. Without the check the value
	 * would be written into IO.Address and surface on the boot wait's first transfer,
	 * reported as a BOOT failure - sending the reader to investigate the wrong thing. */
	static const uint8_t bad[] = {0x00, 0x07, 0x78, 0x7F, 0xFF};

	for (size_t i = 0; i < ARRAY_SIZE(bad); i++) {
		before(NULL);
		zassert_equal(tof_cliff_sensor_open(&obj, bad[i], &st), -EINVAL,
			      "address 0x%02x was accepted", bad[i]);
		zassert_equal(st.stage, TOF_CLIFF_STAGE_BUS_IO);
		zassert_equal(st.port_errno, -EINVAL);
		zassert_equal(strlen(f.order), 0, "nothing may be called for a bad address");
		zassert_equal(fake_i2c_count, 0);
	}

	/* Both ends of the valid range work. */
	for (uint8_t good = 0x08; good <= 0x77; good += 0x6F) {
		before(NULL);
		zassert_equal(tof_cliff_sensor_open(&obj, good, &st), 0,
			      "address 0x%02x was refused", good);
	}
}

ZTEST(tof_cliff_adapter, test_copy_raw_checks_its_output_pointer_before_clearing_it)
{
	/* Clearing first would crash on exactly the input the check exists for. */
	zassert_equal(tof_cliff_copy_raw(&f.canned, NULL), -EINVAL);
	zassert_equal(tof_cliff_copy_raw(NULL, NULL), -EINVAL);
}

ZTEST(tof_cliff_adapter, test_configure_passes_mode_and_budget_through_untouched)
{
	/* Both are still unresolved symbols in the wire contract. They are arguments so
	 * that no value gets frozen in code before the timing work settles it. */
	zassert_equal(tof_cliff_sensor_configure(&obj, VL53LX_DISTANCEMODE_LONG, 33000, &st), 0);
	zassert_equal(f.seen_mode, VL53LX_DISTANCEMODE_LONG);
	zassert_equal(f.seen_budget_us, 33000);
	zassert_equal(strcmp(f.order, "MT"), 0, "order was \"%s\"", f.order);
}

ZTEST(tof_cliff_adapter, test_configure_stops_at_the_failing_step)
{
	f.mode_rc = VL53LX_ERROR_CONTROL_INTERFACE;
	zassert_not_equal(tof_cliff_sensor_configure(&obj, VL53LX_DISTANCEMODE_LONG, 33000, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_DISTANCE_MODE);
	zassert_equal(strcmp(f.order, "M"), 0, "the budget must not be set after a failure");

	before(NULL);
	f.budget_rc = VL53LX_ERROR_CONTROL_INTERFACE;
	zassert_not_equal(tof_cliff_sensor_configure(&obj, VL53LX_DISTANCEMODE_LONG, 33000, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_TIMING_BUDGET);
}

ZTEST(tof_cliff_adapter, test_stop_reports_its_own_stage_not_the_start_stage)
{
	/* Sharing START's stage would send a reader of the log looking for a bring-up
	 * failure while the device was in fact refusing to stop. */
	zassert_equal(tof_cliff_sensor_stop(&obj, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_NONE);
	zassert_equal(f.stop_calls, 1);

	before(NULL);
	fake_i2c_fail_on(1, -EIO);
	f.stop_bus_xfers = 1;
	zassert_not_equal(tof_cliff_sensor_stop(&obj, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_STOP);
	zassert_equal(st.port_errno, -EIO);
	zassert_not_equal(strcmp(tof_cliff_stage_name(st.stage), "start"), 0);
	zassert_equal(strcmp(tof_cliff_stage_name(st.stage), "stop"), 0);
}

ZTEST(tof_cliff_adapter, test_reading_never_stops_or_restarts_the_device)
{
	const int16_t mm[1] = {500};
	const uint8_t status[1] = {0};
	int rearm_after_start;

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);

	/* Arming itself spends one clear - StartMeasurement then
	 * ClearInterruptAndStartMeasurement - so the per-cycle cost has to be measured from
	 * here rather than from zero, or this test would silently accept a second start. */
	rearm_after_start = f.rearm_calls;
	zassert_equal(rearm_after_start, 1, "arming issues exactly one clear");

	for (int i = 0; i < 5; i++) {
		f.canned.StreamCount++;
		zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	}

	/* Cycling four L4s every period would pay a full start sequence per sensor per
	 * period and make the timing budget meaningless. Re-arm is the only per-cycle
	 * device call. */
	zassert_equal(f.start_calls, 1);
	zassert_equal(f.stop_calls, 0);
	zassert_equal(f.rearm_calls - rearm_after_start, 5, "one re-arm per cycle, no more");
}

/* C1. The replay guard is an independent defence, and the one that does not depend on
 * any status being reported correctly. A device that re-presents its PREVIOUS result
 * while the bus is healthy and every layer returns success defeats both patch 0001 and
 * the sticky port record - and republishing that range with fresh == true is the floor
 * read while still on the floor, handed up every cycle as current while the robot drives
 * off a ledge. The stream count is the only material already being carried that can tell
 * the two apart. */
ZTEST(tof_cliff_adapter, test_an_unchanged_stream_count_is_a_replay_not_a_fresh_sample)
{
	const int16_t mm[1] = {120};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);

	/* First read establishes the history and must be accepted. */
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_true(sample.fresh);
	zassert_equal(sample.entries[0].range_mm, 120);

	/* Same count again, with the bus and the ULD both reporting success. */
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO);
	zassert_false(sample.fresh, "stale bytes must not be handed up as a reading");
	zassert_false(st.sample_present);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_FETCH);
	zassert_true(st.stale_replay, "a replay must be nameable, not just an -EPROTO");
	zassert_equal(st.port_errno, 0, "the bus was healthy; this is not a transport fault");

	/* And the device is re-armed anyway, or one recoverable replay would leave ready
	 * asserted and turn every later read into the same failure. */
	zassert_equal(f.rearm_calls, 2);
}

/* The other half of C1, and the half that needed a fix in the vendor tree rather than
 * here: once the ULD reports the failure instead of masking it, this layer must not have
 * recorded anything from it. The stream history is the part that matters -- it is what
 * decides whether the NEXT frame looks fresh -- and a failed fetch returns before the
 * history is written, so a recovery frame carrying the count the failure was going to
 * claim is still judged on its own merits.
 *
 * tests/tof_uld_status pins the vendor half: that the failure is reported at all. */
ZTEST(tof_cliff_adapter, test_a_failed_fetch_leaves_the_stream_history_alone)
{
	const int16_t mm[1] = {300};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	f.canned.StreamCount = 5;
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(stream.last_stream_count, 5);
	zassert_true(stream.valid);

	/* The fetch fails, and the count it would have carried is one the guard has never
	 * seen. Nothing about it may be remembered. */
	f.canned.StreamCount = 6;
	f.fetch_rc = VL53LX_ERROR_RANGE_ERROR;
	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_false(sample.fresh);
	zassert_equal(stream.last_stream_count, 5,
		      "a read that never succeeded must not become the history the next "
		      "comparison is made against");
	zassert_true(stream.valid, "and it must not discard the history either");
}

/* Failing closed must not latch: the sensor recovers and publishes again. What it must
 * NOT publish is the frame the device was holding through the failure.
 *
 * This test used to assert the opposite, and passed, because the old fake produced a
 * brand-new frame on every fetch -- so "the next read after a failure" was always a fresh
 * measurement, which is the one thing hardware cannot do while the interrupt is still
 * asserted. Here the fake holds its frame until the re-arm releases it, which is what the
 * device does, and the distinction becomes visible: count 6 is the frame measured before
 * the outage, count 7 is the first frame measured after it. */
ZTEST(tof_cliff_adapter, test_the_frame_held_through_a_failure_is_not_published_as_fresh)
{
	const int16_t mm[1] = {300};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	f.frame_advances_only_on_rearm = true;
	f.held_stream_count = 5;

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(sample.stream_count, 5);
	zassert_equal(f.rearm_calls, 1, "the published frame is released too");

	/* The fetch fails on the frame the device measured next, count 6. */
	f.fetch_rc = VL53LX_ERROR_RANGE_ERROR;
	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_false(sample.fresh);
	zassert_equal(stream.last_stream_count, 5, "a failed read is not history");
	zassert_equal(f.rearm_calls, 2, "count 6 was refused, so it must be released");

	/* Recovery. Because count 6 was released rather than left in place, what arrives is
	 * count 7 -- measured after the outage -- and not the pre-outage frame. */
	f.fetch_rc = VL53LX_ERROR_NONE;
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_true(sample.fresh);
	zassert_false(st.stale_replay);
	zassert_equal(sample.entries[0].range_mm, 300);
	zassert_equal(sample.stream_count, 7,
		      "6 is the frame the device held through the failure; publishing it "
		      "would report the floor from before the outage as the floor now");
	zassert_equal(stream.last_stream_count, 7);
}

/* The same release is owed when the ULD returns success and only the bus says otherwise.
 * That path is the reason the sticky errno exists, and it refuses the frame just as hard,
 * so it must not be the one refusal that leaves the device wedged. */
ZTEST(tof_cliff_adapter, test_a_transport_failure_under_a_success_return_still_rearms)
{
	const int16_t mm[1] = {400};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	/* The shape the sticky errno defends against, posed on purpose rather than claimed
	 * of the patched ULD: the transport failed and the ULD returned VL53LX_ERROR_NONE
	 * anyway. The injection is on the fetch's first transfer only, so the re-arm that
	 * follows runs on a healthy bus. */
	f.fetch_rc = VL53LX_ERROR_NONE;
	fake_i2c_fail_on(1, -EIO);

	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_FETCH);
	zassert_equal(st.port_errno, -EIO, "the bus failure is what refused this frame");
	zassert_false(sample.fresh);
	zassert_equal(f.rearm_calls, 1);
}

/* When the re-arm on a refusal path fails too, the re-arm is the fact that survives: the
 * refusal cost one frame, this costs every later one. */
ZTEST(tof_cliff_adapter, test_a_fetch_failure_whose_rearm_also_fails_reports_the_rearm)
{
	const int16_t mm[1] = {300};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	f.fetch_rc = VL53LX_ERROR_RANGE_ERROR;
	f.rearm_rc = VL53LX_ERROR_CONTROL_INTERFACE;

	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_true(st.rearm_failed, "the next sample will not arrive either");
	zassert_equal(st.stage, TOF_CLIFF_STAGE_REARM);
	zassert_equal(st.uld_rc, VL53LX_ERROR_CONTROL_INTERFACE);
	zassert_false(st.sample_present);
	zassert_false(sample.fresh);
	zassert_equal(f.rearm_calls, 1);
}

/* A READY_CHECK failure is the one refusal that must NOT re-arm: nothing has confirmed a
 * frame was consumed, so clearing the interrupt would discard one nobody looked at. */
ZTEST(tof_cliff_adapter, test_a_ready_check_failure_does_not_rearm)
{
	const int16_t mm[1] = {300};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	f.ready_rc = VL53LX_ERROR_CONTROL_INTERFACE;

	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_READY_CHECK);
	zassert_equal(f.fetch_calls, 0);
	zassert_equal(f.rearm_calls, 0, "no frame was consumed, so none may be discarded");
}

/* The replay verdict and a failing re-arm are separate facts and both must survive. */
ZTEST(tof_cliff_adapter, test_a_replay_whose_rearm_also_fails_reports_both)
{
	const int16_t mm[1] = {120};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);

	f.rearm_rc = VL53LX_ERROR_CONTROL_INTERFACE;
	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_true(st.stale_replay, "the replay is why the payload was refused");
	zassert_true(st.rearm_failed, "and the next sample will not arrive either");
	zassert_false(st.sample_present);
	zassert_false(sample.fresh);
}

/* A replay is refused; an advancing count is not. The wrap is 0xFF -> 0x80, not 0xFF -> 0:
 * upstream vl53lx_core.c does that explicitly, so 128 is an advance from 255 and must not
 * read as a repeat. */
ZTEST(tof_cliff_adapter, test_an_advancing_stream_count_is_accepted_including_the_wrap)
{
	const uint8_t seq[] = {254, 255, 0x80, 0x81};
	const int16_t mm[1] = {200};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);

	for (size_t i = 0; i < ARRAY_SIZE(seq); i++) {
		f.canned.StreamCount = seq[i];
		zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0,
			      "count %u was refused", seq[i]);
		zassert_true(sample.fresh);
		zassert_false(st.stale_replay);
	}
}

/* The history belongs to one stream state, and the caller is required to keep exactly one
 * per device - the API cannot prove that binding, it can only avoid making a shared one
 * mandatory. Four L4s share one scratch deliberately, so had the count lived there a
 * quiet sensor reporting the same number as its neighbour would be refused as a replay.
 * Two stream states, one scratch, same count: both accepted. */
ZTEST(tof_cliff_adapter, test_the_stream_history_is_per_state_not_per_shared_scratch)
{
	struct tof_cliff_stream_state a = {0};
	struct tof_cliff_stream_state b = {0};
	const int16_t mm[1] = {90};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &a, &sample, &st), 0);
	zassert_true(sample.fresh);

	/* Same shared scratch, same StreamCount, different device. */
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &b, &sample, &st), 0,
		      "one sensor's count must not invalidate another's");
	zassert_true(sample.fresh);
	zassert_false(st.stale_replay);

	/* But each device still catches its own repeat. */
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &a, &sample, &st), -EPROTO);
	zassert_true(st.stale_replay);
}

/* A new ranging session starts a new count sequence, so the history has to be dropped
 * or the first read after a restart would be refused as a replay of the old session. */
ZTEST(tof_cliff_adapter, test_start_resets_the_stream_history)
{
	const int16_t mm[1] = {75};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO);

	zassert_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0,
		      "the same count is a new sample after a restart");
	zassert_true(sample.fresh);
	zassert_false(st.stale_replay);
}

/* A failed start must NOT clear the history. The caller is not allowed to read after one,
 * but if it does anyway, an armed guard still refuses the pre-restart sample. Clearing
 * would let exactly one stale reading through - the opposite of what this guard is for. */
ZTEST(tof_cliff_adapter, test_a_failed_start_preserves_the_guard)
{
	const int16_t mm[1] = {75};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);

	f.start_rc = VL53LX_ERROR_CONTROL_INTERFACE;
	zassert_not_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);

	/* Same count as before the failed start: still a replay, still refused. */
	f.start_rc = VL53LX_ERROR_NONE;
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO);
	zassert_true(st.stale_replay);
	zassert_false(sample.fresh);
}

/* ---------------------------------------------- arming is two calls, not one ----- */

/* ST's own VL53L4CX_Start() issues StartMeasurement() and then
 * ClearInterruptAndStartMeasurement(). This adapter issued only the first, and that is
 * why every L4 sample this project ever recorded was the PREVIOUS session's frame: the
 * stale data-ready was still asserted, so the first fetch after a restart returned a
 * result produced before it, carrying the old stream count.
 *
 * The order is asserted exactly rather than by counting calls, because the two failure
 * modes worth catching are both invisible to a count: deleting the second call leaves
 * "G", and clearing before starting leaves "cG" - a clear against a stopped device,
 * which arms nothing. */
ZTEST(tof_cliff_adapter, test_a_successful_start_issues_start_then_clear_in_that_order)
{
	zassert_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);

	zassert_equal(strcmp(f.order, "Gc"), 0, "arming sequence was \"%s\", expected \"Gc\"",
		      f.order);
	zassert_equal(f.start_calls, 1);
	zassert_equal(f.rearm_calls, 1);
	zassert_equal(f.stop_calls, 0, "a start that worked must not also stop the device");
	zassert_equal(st.stage, TOF_CLIFF_STAGE_NONE);
}

/* A device that refused to start has nothing to clear. Issuing the second call anyway
 * would put a write on a bus that has just failed and, worse, make the log say the
 * arming reached its second half when it never did. */
ZTEST(tof_cliff_adapter, test_a_failed_first_call_never_issues_the_second)
{
	const int16_t mm[1] = {75};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);

	f.order[0] = '\0';
	f.start_rc = VL53LX_ERROR_CONTROL_INTERFACE;
	zassert_not_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);

	zassert_equal(st.stage, TOF_CLIFF_STAGE_START, "the first half is where it failed");
	zassert_equal(strcmp(f.order, "G"), 0, "sequence after a refused start was \"%s\"",
		      f.order);
	zassert_equal(f.stop_calls, 0, "nothing was started, so there is nothing to stop");

	/* And the guard is still armed: the pre-restart count is still a replay. */
	f.start_rc = VL53LX_ERROR_NONE;
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO);
	zassert_true(st.stale_replay);
}

/* Half-armed is the one state nobody above this function can represent: StartMeasurement
 * succeeded so the device IS ranging, but start() reports failure, so every caller records
 * it as not started and none of them will ever stop it. The cleanup is what keeps that
 * from leaving a ranging device behind. */
ZTEST(tof_cliff_adapter, test_a_failed_second_call_stops_the_device_and_keeps_the_guard)
{
	const int16_t mm[1] = {75};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);

	f.order[0] = '\0';
	f.rearm_rc = VL53LX_ERROR_CONTROL_INTERFACE;
	zassert_not_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);

	/* Its own stage, not START and not the per-sample REARM: a reader of the log has to
	 * be able to tell which half of arming failed, and has to not mistake this for a
	 * failure that happened while sampling. */
	zassert_equal(st.stage, TOF_CLIFF_STAGE_START_CLEAR);
	zassert_equal(strcmp(tof_cliff_stage_name(st.stage), "start_clear"), 0);
	zassert_equal(strcmp(f.order, "GcP"), 0, "sequence was \"%s\", expected \"GcP\"",
		      f.order);
	zassert_equal(f.stop_calls, 1, "a half-armed device must not be left ranging");

	/* The guard survives this path too. */
	f.rearm_rc = VL53LX_ERROR_NONE;
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO);
	zassert_true(st.stale_replay);
	zassert_false(sample.fresh);
}

/* The history may only be dropped once the device is really re-armed. Dropping it after
 * the first call alone would accept the stale frame the second call exists to discard -
 * the guard would be spent on the one sample it was added to refuse. */
ZTEST(tof_cliff_adapter, test_only_a_start_that_completed_both_calls_clears_the_guard)
{
	const int16_t mm[1] = {75};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO);

	f.start_rc = VL53LX_ERROR_CONTROL_INTERFACE;
	zassert_not_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);
	f.start_rc = VL53LX_ERROR_NONE;
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO,
		      "a start that never began must not release the guard");

	f.rearm_rc = VL53LX_ERROR_CONTROL_INTERFACE;
	zassert_not_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);
	f.rearm_rc = VL53LX_ERROR_NONE;
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), -EPROTO,
		      "a start that stopped half way must not release it either");

	zassert_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0,
		      "both halves done: the same count is a new session's sample");
	zassert_true(sample.fresh);
	zassert_false(st.stale_replay);
}

/* The same posed shape, applied to the new call: the ULD returns VL53LX_ERROR_NONE while
 * the transfer under it failed. Taking the ULD's word here would record an unarmed device
 * as armed, and the very first read would hand back the previous session's frame. Patch
 * 0001 does not cover this call, and the sticky errno is what makes the adapter safe
 * against it either way. */
ZTEST(tof_cliff_adapter, test_the_second_calls_bus_failure_outranks_its_uld_result)
{
	f.rearm_bus_xfers = 1;
	f.rearm_rc = VL53LX_ERROR_NONE;
	fake_i2c_fail_on(1, -EIO); /* StartMeasurement makes no traffic; this is the clear's */

	zassert_equal(tof_cliff_sensor_start(&obj, &stream, &st), -EIO);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_START_CLEAR);
	zassert_equal(st.port_errno, -EIO);
	zassert_equal(st.uld_rc, VL53LX_ERROR_NONE, "the ULD really did claim success");
	zassert_equal(f.stop_calls, 1, "a bus-failed clear is still a half-armed device");
}

ZTEST(tof_cliff_adapter, test_null_arguments_are_rejected_without_touching_the_device)
{
	zassert_equal(tof_cliff_read_once(NULL, &scratch, &stream, &sample, &st), -EINVAL);
	zassert_equal(tof_cliff_read_once(&obj, NULL, &stream, &sample, &st), -EINVAL);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, NULL, &sample, &st), -EINVAL);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, NULL, &st), -EINVAL);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, NULL), -EINVAL);
	zassert_equal(f.ready_calls, 0);
	zassert_equal(f.fetch_calls, 0);

	/* start() gained the same obligation: without a stream state it cannot invalidate
	 * the history, so accepting NULL would silently leave the old count in force. */
	zassert_equal(tof_cliff_sensor_start(NULL, &stream, &st), -EINVAL);
	zassert_equal(tof_cliff_sensor_start(&obj, NULL, &st), -EINVAL);
	zassert_equal(tof_cliff_sensor_start(&obj, &stream, NULL), -EINVAL);
	zassert_equal(f.start_calls, 0);
}

ZTEST(tof_cliff_adapter, test_copy_raw_is_pure_and_handles_a_null_input)
{
	const int16_t mm[1] = {123};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_copy_raw(&f.canned, &sample), 0);
	zassert_true(sample.fresh);
	zassert_equal(sample.entries[0].range_mm, 123);
	zassert_equal(fake_i2c_count, 0, "the copy step must not touch the bus");

	zassert_equal(tof_cliff_copy_raw(NULL, &sample), -EINVAL);
	zassert_false(sample.fresh);
	zassert_equal(sample.entry_count, 0);
}

ZTEST(tof_cliff_adapter, test_placeholder_transport_is_never_reached_by_the_read_path)
{
	const int16_t mm[1] = {500};
	const uint8_t status[1] = {0};

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_sensor_open(&obj, TEST_ADDR7, &st), 0);
	zassert_equal(tof_cliff_sensor_configure(&obj, VL53LX_DISTANCEMODE_LONG, 33000, &st), 0);
	zassert_equal(tof_cliff_sensor_start(&obj, &stream, &st), 0);
	for (int i = 0; i < 3; i++) {
		f.canned.StreamCount++;
		zassert_equal(tof_cliff_read_once(&obj, &scratch, &stream, &sample, &st), 0);
	}

	/* Defence in depth behind the fact that this interface has no enable operation to
	 * misuse: any traffic here means a second transport path appeared. */
	zassert_equal(vl53l4cx_bus_io_placeholder_calls(), 0);
}

ZTEST(tof_cliff_adapter, test_every_stage_has_a_name)
{
	static const enum tof_cliff_stage all[] = {
		TOF_CLIFF_STAGE_NONE,	       TOF_CLIFF_STAGE_BUS_IO,
		TOF_CLIFF_STAGE_BOOT,	       TOF_CLIFF_STAGE_DATA_INIT,
		TOF_CLIFF_STAGE_REF_SPAD,      TOF_CLIFF_STAGE_DISTANCE_MODE,
		TOF_CLIFF_STAGE_TIMING_BUDGET, TOF_CLIFF_STAGE_START,
		TOF_CLIFF_STAGE_STOP,	       TOF_CLIFF_STAGE_READY_CHECK,
		TOF_CLIFF_STAGE_FETCH,	       TOF_CLIFF_STAGE_REARM,
	};

	/* Distinct names, so two stages cannot be confused in a log. */
	for (size_t i = 0; i < ARRAY_SIZE(all); i++) {
		for (size_t j = i + 1; j < ARRAY_SIZE(all); j++) {
			zassert_not_equal(strcmp(tof_cliff_stage_name(all[i]),
						 tof_cliff_stage_name(all[j])),
					  0, "stages %d and %d share a name", (int)all[i],
					  (int)all[j]);
		}
	}

	/* A stage that reaches a log line as "unknown" is a stage nobody can triage. */
	for (size_t i = 0; i < ARRAY_SIZE(all); i++) {
		zassert_not_equal(strcmp(tof_cliff_stage_name(all[i]), "unknown"), 0,
				  "stage %d has no name", (int)all[i]);
	}
}
