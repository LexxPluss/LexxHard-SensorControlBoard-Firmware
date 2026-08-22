/*
 * Copyright (c) 2024-2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <string.h>

#include "tof_cliff_sensor.h"
#include "vl53l4cx_bus_io.h"
#include "vl53l4cx_port.h"

/*
 * Every ULD call goes through this shape:
 *
 *   reset the port's sticky errno -> make the call -> read the sticky errno back
 *
 * and the sticky value wins. VL53LX_GetMultiRangingData runs get_device_results and
 * then assigns SetMeasurementData's result over the same Status variable, so a failed
 * transfer inside the results read can return VL53LX_ERROR_NONE. Without the sticky
 * check there is no way to tell that apart from a good read.
 */
static int tof_cliff_finish(struct tof_cliff_read_status *st, enum tof_cliff_stage stage,
			    VL53LX_Error rc)
{
	int sticky = vl53l4cx_port_sticky_errno();

	st->uld_rc = (int)rc;
	st->port_errno = sticky;

	if (sticky != 0) {
		st->stage = stage;
		return sticky;
	}
	if (rc != VL53LX_ERROR_NONE) {
		st->stage = stage;
		return -EIO;
	}
	return 0;
}

static void tof_cliff_status_reset(struct tof_cliff_read_status *st)
{
	st->stage = TOF_CLIFF_STAGE_NONE;
	st->port_errno = 0;
	st->uld_rc = 0;
	st->sample_present = false;
	st->rearm_failed = false;
	st->stale_replay = false;
}

const char *tof_cliff_stage_name(enum tof_cliff_stage stage)
{
	switch (stage) {
	case TOF_CLIFF_STAGE_NONE:
		return "none";
	case TOF_CLIFF_STAGE_BUS_IO:
		return "bus_io";
	case TOF_CLIFF_STAGE_BOOT:
		return "boot";
	case TOF_CLIFF_STAGE_DATA_INIT:
		return "data_init";
	case TOF_CLIFF_STAGE_REF_SPAD:
		return "ref_spad";
	case TOF_CLIFF_STAGE_DISTANCE_MODE:
		return "distance_mode";
	case TOF_CLIFF_STAGE_TIMING_BUDGET:
		return "timing_budget";
	case TOF_CLIFF_STAGE_START:
		return "start";
	case TOF_CLIFF_STAGE_STOP:
		return "stop";
	case TOF_CLIFF_STAGE_READY_CHECK:
		return "ready_check";
	case TOF_CLIFF_STAGE_FETCH:
		return "fetch";
	case TOF_CLIFF_STAGE_REARM:
		return "rearm";
	default:
		return "unknown";
	}
}

/* ---------------------------------------------------------------- lifecycle ------ */

int tof_cliff_sensor_open(VL53L4CX_Object_t *obj, uint8_t addr_7bit,
			  struct tof_cliff_read_status *st)
{
	VL53L4CX_IO_t io;
	int ret;

	if (obj == NULL || st == NULL) {
		return -EINVAL;
	}
	tof_cliff_status_reset(st);

	/* Reject the address here rather than let it reach the bus. Anything outside the
	 * 7-bit unicast range is a caller error, and without this check it would be
	 * written into IO.Address and only surface on the first transfer of the boot wait
	 * - reported as a BOOT failure, which is the wrong thing to go and investigate.
	 * The reserved ranges are I2C's own: 0x00-0x07 and 0x78-0x7F. */
	if (addr_7bit < 0x08U || addr_7bit > 0x77U) {
		st->stage = TOF_CLIFF_STAGE_BUS_IO;
		st->port_errno = -EINVAL;
		return -EINVAL;
	}

	vl53l4cx_bus_io_fill(&io, addr_7bit);

	vl53l4cx_port_sticky_reset();
	if (VL53L4CX_RegisterBusIO(obj, &io) != VL53L4CX_OK) {
		st->stage = TOF_CLIFF_STAGE_BUS_IO;
		st->port_errno = vl53l4cx_port_sticky_errno();
		st->uld_rc = VL53L4CX_ERROR;
		return -EIO;
	}

	/* The three steps VL53L4CX_Init performs, called individually. The BSP wrapper
	 * folds all three into one VL53L4CX_ERROR, which loses the only information that
	 * matters when a chain comes up wrong: whether the part never booted, answered
	 * but rejected DataInit, or failed SPAD management. */
	vl53l4cx_port_sticky_reset();
	ret = tof_cliff_finish(st, TOF_CLIFF_STAGE_BOOT, VL53LX_WaitDeviceBooted(obj));
	if (ret != 0) {
		return ret;
	}

	vl53l4cx_port_sticky_reset();
	ret = tof_cliff_finish(st, TOF_CLIFF_STAGE_DATA_INIT, VL53LX_DataInit(obj));
	if (ret != 0) {
		return ret;
	}

	vl53l4cx_port_sticky_reset();
	ret = tof_cliff_finish(st, TOF_CLIFF_STAGE_REF_SPAD,
			       VL53LX_PerformRefSpadManagement(obj));
	if (ret != 0) {
		return ret;
	}

	return 0;
}

int tof_cliff_sensor_configure(VL53L4CX_Object_t *obj, VL53LX_DistanceModes mode,
			       uint32_t timing_budget_us, struct tof_cliff_read_status *st)
{
	int ret;

	if (obj == NULL || st == NULL) {
		return -EINVAL;
	}
	tof_cliff_status_reset(st);

	vl53l4cx_port_sticky_reset();
	ret = tof_cliff_finish(st, TOF_CLIFF_STAGE_DISTANCE_MODE,
			       VL53LX_SetDistanceMode(obj, mode));
	if (ret != 0) {
		return ret;
	}

	vl53l4cx_port_sticky_reset();
	return tof_cliff_finish(
		st, TOF_CLIFF_STAGE_TIMING_BUDGET,
		VL53LX_SetMeasurementTimingBudgetMicroSeconds(obj, timing_budget_us));
}

int tof_cliff_sensor_start(VL53L4CX_Object_t *obj, struct tof_cliff_stream_state *stream,
			   struct tof_cliff_read_status *st)
{
	int ret;

	if (obj == NULL || stream == NULL || st == NULL) {
		return -EINVAL;
	}
	tof_cliff_status_reset(st);

	vl53l4cx_port_sticky_reset();
	ret = tof_cliff_finish(st, TOF_CLIFF_STAGE_START, VL53LX_StartMeasurement(obj));
	if (ret != 0) {
		/* The old history is deliberately KEPT. A caller must not read after a failed
		 * start, but if it does anyway, an armed guard still refuses the pre-restart
		 * sample; clearing here would let exactly one stale reading through first. */
		return ret;
	}

	/* A successful start begins a new numbering - the ULD zeroes rd_stream_count when
	 * the mode is set - so the previous session's count is no longer a basis for
	 * comparison and keeping it could refuse a genuinely new sample. */
	memset(stream, 0, sizeof(*stream));
	return 0;
}

int tof_cliff_sensor_stop(VL53L4CX_Object_t *obj, struct tof_cliff_read_status *st)
{
	if (obj == NULL || st == NULL) {
		return -EINVAL;
	}
	tof_cliff_status_reset(st);

	vl53l4cx_port_sticky_reset();
	return tof_cliff_finish(st, TOF_CLIFF_STAGE_STOP, VL53LX_StopMeasurement(obj));
}

/* ------------------------------------------------------------------- sample ------ */

int tof_cliff_copy_raw(const VL53LX_MultiRangingData_t *in, struct tof_cliff_sample *out)
{
	uint8_t entries;
	uint8_t i;

	/* Arguments first, then clear: memset on a null out would crash before the
	 * validation it was meant to precede. */
	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));
	if (in == NULL) {
		return -EINVAL;
	}

	/* A count the result array cannot hold is impossible metadata, not a large
	 * reading, and truncating it would be the worst of the three options: it hands up
	 * a target_count the entries do not support, and a caller iterating on
	 * target_count then walks off the end of the array. Nothing here is salvageable,
	 * so no sample is produced at all. */
	if (in->NumberOfObjectsFound > TOF_CLIFF_MAX_TARGETS) {
		return -EPROTO;
	}

	out->stream_count = in->StreamCount;

	/* The true count, kept even when it is zero. */
	out->target_count = in->NumberOfObjectsFound;

	/* SetMeasurementData writes RangeData[0] even with no targets: it forces
	 * `iteration = 1` when active_results < 1. That entry is the only record of why
	 * nothing was found, so copy it. */
	entries = (out->target_count == 0U) ? 1U : out->target_count;

	for (i = 0; i < entries; i++) {
		/* Raw on both fields. RangeMilliMeter stays int16_t and keeps its sign:
		 * a negative reading below the floor plane is real information for a
		 * cliff, and the BSP's clamp to 0 is precisely the loss this avoids. */
		out->entries[i].range_mm = in->RangeData[i].RangeMilliMeter;
		out->entries[i].range_status = in->RangeData[i].RangeStatus;
	}
	out->entry_count = entries;
	out->fresh = true;
	return 0;
}

int tof_cliff_read_once(VL53L4CX_Object_t *obj, struct tof_cliff_scratch *scratch,
			struct tof_cliff_stream_state *stream,
			struct tof_cliff_sample *sample, struct tof_cliff_read_status *st)
{
	uint8_t ready = 0;
	bool replay;
	int ret;

	if (obj == NULL || scratch == NULL || stream == NULL || sample == NULL || st == NULL) {
		return -EINVAL;
	}
	tof_cliff_status_reset(st);
	memset(sample, 0, sizeof(*sample));

	/* One non-blocking check. No loop, no timeout, no waiting on this sensor. */
	vl53l4cx_port_sticky_reset();
	ret = tof_cliff_finish(st, TOF_CLIFF_STAGE_READY_CHECK,
			       VL53LX_GetMeasurementDataReady(obj, &ready));
	if (ret != 0) {
		return ret;
	}
	if (ready > 1U) {
		/* The flag is a single bit's worth of meaning. Anything else means the
		 * device or the transfer is confused, and calling that "not ready" would
		 * let it look like a merely quiet sensor for as long as it kept happening. */
		st->stage = TOF_CLIFF_STAGE_READY_CHECK;
		return -EPROTO;
	}
	if (ready != 1U) {
		/* Nothing yet. Not an error, and deliberately not a retry either: the
		 * scheduler decides whether a missing sample this cycle matters, using
		 * the contract's per-cycle sample_produced_mask. */
		return 0;
	}

	vl53l4cx_port_sticky_reset();
	ret = tof_cliff_finish(st, TOF_CLIFF_STAGE_FETCH,
			       VL53LX_GetMultiRangingData(obj, &scratch->data));
	if (ret != 0) {
		return ret;
	}

	/* GetMultiRangingData can return success after an internal failure and leave the
	 * previous result in its output. The sticky port errno catches transport failures,
	 * but not every ULD-internal failure. A ready result whose stream count did not
	 * advance is therefore a replay, never a fresh sample. The comparison is per device;
	 * different sensors are allowed to report the same count, which is also why this
	 * history cannot live in the scratch - one scratch is shared by all four L4s.
	 *
	 * The alias period is 128 frames, not 256: upstream vl53lx_core.c wraps the counter
	 * 0xFF -> 0x80 rather than to 0, so after the first pass it only ever cycles through
	 * 0x80..0xFF. Exactly 128 device measurements between two reads therefore alias to
	 * the same value and a genuinely new sample is refused - about 4.2 s of uninterrupted
	 * ranging with nobody reading at a 33 ms budget, which a commissioning session
	 * holding the chain lock can produce. The direction is deliberate: a refused real
	 * sample costs one cycle and a fault bit, while an accepted replay reports the floor
	 * from four seconds ago as the floor now. */
	replay = stream->valid && scratch->data.StreamCount == stream->last_stream_count;
	if (replay) {
		st->stale_replay = true;
		/* Re-arm even though this payload is rejected. Otherwise a recoverable replay
		 * would leave ready asserted forever and guarantee every later read also fails. */
		vl53l4cx_port_sticky_reset();
		ret = tof_cliff_finish(st, TOF_CLIFF_STAGE_REARM,
				       VL53LX_ClearInterruptAndStartMeasurement(obj));
		if (ret != 0) {
			st->rearm_failed = true;
			return ret;
		}
		st->stage = TOF_CLIFF_STAGE_FETCH;
		return -EPROTO;
	}

	ret = tof_cliff_copy_raw(&scratch->data, sample);
	if (ret != 0) {
		/* Impossible metadata. The fetch itself succeeded, so the stage stays
		 * FETCH, but nothing is published: a corrupt count must not become a
		 * fresh sample. */
		st->stage = TOF_CLIFF_STAGE_FETCH;
		memset(sample, 0, sizeof(*sample));
		return ret;
	}
	/* Recorded from the same expression the comparison above reads, not from the copied
	 * sample: if the copy step ever transformed the value the two would drift apart and
	 * the replay check would start comparing against something else. */
	stream->last_stream_count = scratch->data.StreamCount;
	stream->valid = true;
	st->sample_present = true;

	/* Re-arm. Its failure is reported as an error so a caller that checks only the
	 * return code stops trusting this sensor, but sample_present stays true and the
	 * sample stays intact, so a caller that reads st keeps this cycle's reading. */
	vl53l4cx_port_sticky_reset();
	ret = tof_cliff_finish(st, TOF_CLIFF_STAGE_REARM,
			       VL53LX_ClearInterruptAndStartMeasurement(obj));
	if (ret != 0) {
		st->rearm_failed = true;
		return ret;
	}

	return 0;
}
