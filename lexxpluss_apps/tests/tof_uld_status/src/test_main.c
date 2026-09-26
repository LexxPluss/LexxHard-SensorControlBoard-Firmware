/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* What this suite pins, and why it is not the register map.
 *
 * VL53LX_GetMultiRangingData() used to assign SetMeasurementData()'s return value
 * over VL53LX_get_device_results()'s, so a fetch that failed part-way through came
 * back as VL53LX_ERROR_NONE. The failure this hides is specific and is the reason
 * the patch exists: upstream writes the NEW stream count at the top of
 * VL53LX_f_025(), increments presults->active_results without consulting status,
 * and copies the results out unconditionally -- so the masked frame arrives with an
 * ADVANCED stream count and a slot that nothing ever filled. A consumer's replay
 * guard compares stream counts; an advanced count is exactly the input that makes it
 * accept the frame.
 *
 * And SetMeasurementData() updated pdev->PreviousStreamCount on the way past, so the
 * device's own history moved on a read that never succeeded.
 *
 * These tests inject that shape directly at VL53LX_get_device_results(): the stub
 * writes an advanced count and a partially-filled result, then fails. Nothing here
 * emulates a register, and nothing depends on how the device produces the failure --
 * only on what the driver does with it. Mutation: removing the patch's guard fails
 * test_a_failed_fetch_is_a_failure and
 * test_a_failed_fetch_does_not_advance_the_device_stream_history.
 */

#include <zephyr/ztest.h>
#include <string.h>

#include "vl53lx_api.h"
#include "vl53lx_api_core.h"
#include "vl53lx_platform_user_data.h"

#define SEEDED_PREVIOUS_STREAM_COUNT 3
#define ADVANCED_STREAM_COUNT        7
#define PARTIAL_ACTIVE_RESULTS       2
#define GOOD_RANGE_MM                512

/* The macro that reaches the driver data is (&Obj->...), so Obj must be a pointer
 * EXPRESSION rather than an address-of: passing &storage would expand to &&. */
static VL53L4CX_Object_t storage;
static const VL53LX_DEV dev = &storage;

static struct {
	VL53LX_Error rc;
	uint8_t stream_count;
	uint8_t active_results;
	int16_t median_range_mm;
	uint32_t calls;
} g;

/* The one function the patch reasons about. It reproduces upstream's ordering: the
 * bookkeeping lands in presults BEFORE the status is decided, which is what makes a
 * masked failure look like a fresh frame rather than an empty one. */
VL53LX_Error VL53LX_get_device_results(VL53LX_DEV Dev,
				       VL53LX_DeviceResultsLevel device_results_level,
				       VL53LX_range_results_t *presults)
{
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	ARG_UNUSED(device_results_level);
	g.calls++;

	memset(presults, 0, sizeof(*presults));
	presults->stream_count = g.stream_count;
	presults->active_results = g.active_results;
	presults->device_status = VL53LX_DEVICEERROR_NOUPDATE;
	presults->VL53LX_p_003[0].median_range_mm = g.median_range_mm;
	presults->VL53LX_p_003[0].range_status = VL53LX_DEVICEERROR_RANGECOMPLETE;

	/* Upstream's histogram data carries the same advanced count; SetMeasurementData()
	 * is what copies it into PreviousStreamCount. */
	pdev->hist_data.result__stream_count = g.stream_count;

	return g.rc;
}

/* Reached only on the success path, through SetTargetData(). Neither participates in
 * what this suite pins, so both answer without opinion. */
VL53LX_Error VL53LX_get_tuning_parm(VL53LX_DEV Dev, VL53LX_TuningParms tuning_parm_key,
				    int32_t *ptuning_parm_value)
{
	ARG_UNUSED(Dev);
	ARG_UNUSED(tuning_parm_key);
	*ptuning_parm_value = 0;
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_compute_histo_merge_nb(VL53LX_DEV Dev, uint8_t *histo_merge_nb)
{
	ARG_UNUSED(Dev);
	*histo_merge_nb = 0;
	return VL53LX_ERROR_NONE;
}

static VL53LX_LLDriverData_t *lldata(void)
{
	return VL53LXDevStructGetLLDriverHandle(dev);
}

static void before(void *unused)
{
	ARG_UNUSED(unused);
	memset(&storage, 0, sizeof(storage));
	memset(&g, 0, sizeof(g));
	g.stream_count = ADVANCED_STREAM_COUNT;
	g.median_range_mm = GOOD_RANGE_MM;
	lldata()->PreviousStreamCount = SEEDED_PREVIOUS_STREAM_COUNT;
}

ZTEST_SUITE(tof_uld_status, NULL, NULL, before, NULL, NULL);

ZTEST(tof_uld_status, test_a_failed_fetch_is_a_failure)
{
	VL53LX_MultiRangingData_t out;

	g.rc = VL53LX_ERROR_RANGE_ERROR;
	g.active_results = PARTIAL_ACTIVE_RESULTS;

	zassert_equal(VL53LX_GetMultiRangingData(dev, &out), VL53LX_ERROR_RANGE_ERROR,
		      "a failed result fetch must not be reported as a measurement");
	zassert_equal(g.calls, 1, "and it must be attempted exactly once");
}

ZTEST(tof_uld_status, test_a_failed_fetch_does_not_advance_the_device_stream_history)
{
	VL53LX_MultiRangingData_t out;

	g.rc = VL53LX_ERROR_RANGE_ERROR;
	g.active_results = PARTIAL_ACTIVE_RESULTS;

	(void)VL53LX_GetMultiRangingData(dev, &out);

	zassert_equal(lldata()->PreviousStreamCount, SEEDED_PREVIOUS_STREAM_COUNT,
		      "a read that never succeeded must not move the device's own history; "
		      "the next comparison would otherwise be against a count no successful "
		      "read ever produced");
}

ZTEST(tof_uld_status, test_a_failed_fetch_hands_up_no_usable_stream_count)
{
	VL53LX_MultiRangingData_t out;

	g.rc = VL53LX_ERROR_RANGE_ERROR;
	g.active_results = PARTIAL_ACTIVE_RESULTS;

	(void)VL53LX_GetMultiRangingData(dev, &out);

	/* The advanced count is the dangerous value: it is what a replay guard reads as
	 * "this is a new frame". It must not survive a failed fetch, whatever else the
	 * output holds. */
	zassert_not_equal(out.StreamCount, ADVANCED_STREAM_COUNT,
			  "an advanced stream count from a failed fetch defeats the "
			  "consumer's replay guard");
	zassert_not_equal(out.NumberOfObjectsFound, PARTIAL_ACTIVE_RESULTS,
			  "a slot counted active but never filled must not be presented "
			  "as a target count");
}

ZTEST(tof_uld_status, test_a_successful_fetch_still_publishes_and_advances_the_history)
{
	VL53LX_MultiRangingData_t out;

	g.rc = VL53LX_ERROR_NONE;
	g.active_results = 1;

	zassert_equal(VL53LX_GetMultiRangingData(dev, &out), VL53LX_ERROR_NONE);
	zassert_equal(out.StreamCount, ADVANCED_STREAM_COUNT,
		      "the success path is unchanged by the patch");
	zassert_equal(out.NumberOfObjectsFound, 1);
	zassert_equal(out.RangeData[0].RangeMilliMeter, GOOD_RANGE_MM);
	zassert_equal(lldata()->PreviousStreamCount, ADVANCED_STREAM_COUNT,
		      "a successful read is what moves the history");
}

ZTEST(tof_uld_status, test_a_good_fetch_after_a_failed_one_still_publishes)
{
	VL53LX_MultiRangingData_t out;

	g.rc = VL53LX_ERROR_RANGE_ERROR;
	g.active_results = PARTIAL_ACTIVE_RESULTS;
	(void)VL53LX_GetMultiRangingData(dev, &out);

	/* Failing closed must not latch: the device recovers and the next frame is a
	 * measurement like any other. */
	g.rc = VL53LX_ERROR_NONE;
	g.active_results = 1;
	g.stream_count = ADVANCED_STREAM_COUNT + 1;

	zassert_equal(VL53LX_GetMultiRangingData(dev, &out), VL53LX_ERROR_NONE);
	zassert_equal(out.StreamCount, ADVANCED_STREAM_COUNT + 1);
	zassert_equal(out.RangeData[0].RangeMilliMeter, GOOD_RANGE_MM);
	zassert_equal(lldata()->PreviousStreamCount, ADVANCED_STREAM_COUNT + 1);
}

/* The ULD's own normalisation of negative ranges, pinned against the real SetTargetData
 * rather than against a comment.
 *
 * This exists because tof_cliff_sensor.h used to claim that a VALID negative range
 * reaches the adapter and that only the BSP would clamp it. It does not: SetTargetData,
 * which VL53LX_GetMultiRangingData reaches through SetMeasurementData, rewrites a VALID
 * negative before any caller sees it. The adapter's own test posed {-37, VALID} and
 * {-1, VALID} and passed, because its fake fed those bytes straight through -- shapes the
 * real ULD cannot emit. These two cases run the real code, so what the adapter tests
 * assume is checked rather than asserted.
 *
 * BDTable[VL53LX_TUNING_PROXY_MIN] is the threshold, a file-static defaulting to -30
 * (TUNING_PROXY_MIN, vl53lx_preset_setup.h). It is a tuning parameter, so these tests pin
 * the behaviour at the default rather than treating -30 as a constant of the part. */
ZTEST(tof_uld_status, test_a_valid_negative_inside_the_proxy_threshold_becomes_a_valid_zero)
{
	VL53LX_MultiRangingData_t out;

	g.rc = VL53LX_ERROR_NONE;
	g.active_results = 1;
	g.median_range_mm = -1;

	zassert_equal(VL53LX_GetMultiRangingData(dev, &out), VL53LX_ERROR_NONE);
	zassert_equal(out.RangeData[0].RangeMilliMeter, 0,
		      "the ULD synthesises a zero here; the adapter never sees the -1");
	zassert_equal(out.RangeData[0].RangeStatus, VL53LX_RANGESTATUS_RANGE_VALID,
		      "and it stays VALID, which is what makes the zero indistinguishable "
		      "from a surface at the sensor without this being written down");
}

ZTEST(tof_uld_status, test_a_valid_negative_below_the_proxy_threshold_becomes_invalid)
{
	VL53LX_MultiRangingData_t out;

	g.rc = VL53LX_ERROR_NONE;
	g.active_results = 1;
	g.median_range_mm = -31;

	zassert_equal(VL53LX_GetMultiRangingData(dev, &out), VL53LX_ERROR_NONE);
	zassert_equal(out.RangeData[0].RangeMilliMeter, -31,
		      "the value is kept; it is the status that is rewritten");
	zassert_equal(out.RangeData[0].RangeStatus, VL53LX_RANGESTATUS_RANGE_INVALID,
		      "so a VALID negative cannot reach a consumer of this ULD");
}
