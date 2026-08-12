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
 *   - a fetch can fail on the bus and still return VL53LX_ERROR_NONE, because
 *     VL53LX_GetMultiRangingData assigns SetMeasurementData's result over the status
 *     that get_device_results produced.
 *
 * The second one is not mocked at the sticky-errno level. The fake performs a REAL port
 * transfer through the emulated controller, has it fail, and then returns success - the
 * exact shape of the defect. If the sticky mechanism were removed, these tests would
 * fail rather than silently pass.
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
	VL53LX_MultiRangingData_t canned;
	int ready_calls;
	int fetch_calls;
	int rearm_calls;
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

	*pData = f.canned;
	return f.fetch_rc;
}

VL53LX_Error VL53LX_ClearInterruptAndStartMeasurement(VL53LX_DEV Dev)
{
	ARG_UNUSED(Dev);
	f.rearm_calls++;
	order_add('c');
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
	 * reports success - the same masking shape as the fetch. */
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

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
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

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
	zassert_equal(sample.target_count, 4);
	zassert_equal(sample.entry_count, 4);
	for (int i = 0; i < 4; i++) {
		zassert_equal(sample.entries[i].range_mm, mm[i]);
		zassert_equal(sample.entries[i].range_status, status[i]);
	}
	zassert_equal(sample.stream_count, 7);
}

ZTEST(tof_cliff_adapter, test_negative_range_survives_unclamped)
{
	/* The BSP's vl53l4cx_get_result clamps this to 0. For a cliff sensor a reading
	 * below the floor plane is the signal, not noise, and a 0 would read as a
	 * surface right at the sensor. */
	const int16_t mm[2] = {-37, -1};
	const uint8_t status[2] = {0, 0};

	canned_targets(2, mm, status, 2);

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
	zassert_equal(sample.entries[0].range_mm, -37);
	zassert_equal(sample.entries[1].range_mm, -1);
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

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
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

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), -EPROTO);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_FETCH);
	zassert_false(st.sample_present);
	zassert_false(sample.fresh);
	zassert_equal(sample.target_count, 0, "no part of a corrupt sample may leak out");
	zassert_equal(sample.entry_count, 0);

	/* The rule cannot be bypassed by calling the pure copy step directly either. */
	zassert_equal(tof_cliff_copy_raw(&f.canned, &sample), -EPROTO);
	zassert_false(sample.fresh);

	/* The largest count the array does hold is still accepted. */
	before(NULL);
	canned_targets(TOF_CLIFF_MAX_TARGETS, mm, status, 4);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
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

		zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), -EPROTO,
			      "ready = %u was accepted", bogus);
		zassert_equal(st.stage, TOF_CLIFF_STAGE_READY_CHECK);
		zassert_false(sample.fresh);
		zassert_equal(f.fetch_calls, 0, "nothing may be fetched on a bogus flag");
	}

	/* 0 and 1 keep their ordinary meanings. */
	before(NULL);
	f.ready = 0;
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
	zassert_false(sample.fresh);
}

/* ------------------------------------------------------------- not ready --------- */

ZTEST(tof_cliff_adapter, test_not_ready_is_not_an_error_and_never_waits)
{
	f.ready = 0;

	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
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

	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
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

	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
	zassert_equal(st.stage, TOF_CLIFF_STAGE_FETCH);
	zassert_false(st.sample_present);
	zassert_false(sample.fresh, "a failed fetch must not hand up a half-copied sample");
	zassert_equal(f.rearm_calls, 0);
}

ZTEST(tof_cliff_adapter, test_bus_failure_masked_by_a_successful_return_code_is_still_caught)
{
	const int16_t mm[1] = {500};
	const uint8_t status[1] = {0};
	/* First, middle and last transfer of the fetch. */
	static const int at[] = {1, 2, 3};

	for (size_t i = 0; i < ARRAY_SIZE(at); i++) {
		before(NULL);
		canned_targets(1, mm, status, 1);
		/* This is the defect, reproduced exactly: the transport failed and the
		 * ULD returns VL53LX_ERROR_NONE anyway. */
		f.fetch_rc = VL53LX_ERROR_NONE;
		fake_i2c_fail_on(at[i], -EIO);

		zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0,
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
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
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
	zassert_not_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
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
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
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

	canned_targets(1, mm, status, 1);
	zassert_equal(tof_cliff_sensor_start(&obj, &st), 0);
	for (int i = 0; i < 5; i++) {
		zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
	}

	/* Cycling four L4s every period would pay a full start sequence per sensor per
	 * period and make the timing budget meaningless. Re-arm is the only per-cycle
	 * device call. */
	zassert_equal(f.start_calls, 1);
	zassert_equal(f.stop_calls, 0);
	zassert_equal(f.rearm_calls, 5);
}

ZTEST(tof_cliff_adapter, test_null_arguments_are_rejected_without_touching_the_device)
{
	zassert_equal(tof_cliff_read_once(NULL, &scratch, &sample, &st), -EINVAL);
	zassert_equal(tof_cliff_read_once(&obj, NULL, &sample, &st), -EINVAL);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, NULL, &st), -EINVAL);
	zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, NULL), -EINVAL);
	zassert_equal(f.ready_calls, 0);
	zassert_equal(f.fetch_calls, 0);
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
	zassert_equal(tof_cliff_sensor_start(&obj, &st), 0);
	for (int i = 0; i < 3; i++) {
		zassert_equal(tof_cliff_read_once(&obj, &scratch, &sample, &st), 0);
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
