/*
 * Copyright (c) 2024-2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * One VL53L4CX cliff sensor, as thin a layer over the vendored ULD as the ULD's own
 * defects allow. Everything here exists because a defect made the obvious call unsafe.
 *
 * WHY NOT VL53L4CX_GetDistance()
 *
 * The BSP convenience call cannot be used for four sensors on a safety path:
 *
 *   - vl53l4cx.c holds a function-level `static VL53LX_MultiRangingData_t data;` in
 *     vl53l4cx_get_result, so two instances reading through it share one buffer.
 *   - the same function clamps a negative RangeMilliMeter to 0, which turns a
 *     below-the-floor reading - exactly the interesting case for a cliff - into a
 *     plausible zero.
 *   - it copies only RangeData[0 .. NumberOfObjectsFound), so a zero-target cycle
 *     yields no entry at all and the reason for the zero is lost.
 *   - its blocking poll casts VL53LX_GetMeasurementDataReady's return to void and
 *     spins for V53L3CX_POLL_TIMEOUT = 0xFFFF ms, so a dead sensor can hold the
 *     caller for 65.5 s before reporting a timeout.
 *
 * So this layer calls VL53LX_GetMeasurementDataReady and VL53LX_GetMultiRangingData
 * directly, and the result buffer belongs to the caller.
 *
 * WHAT THIS LAYER MUST NOT DO
 *
 * No clamping, no classification, no reduction. The wire contract's status classes
 * and its most-conservative-then-farthest reduction are decided one layer up, from the
 * raw status codes, and cannot be recovered once this layer rounds anything off.
 *
 * THERE IS DELIBERATELY NO ENABLE OPERATION HERE
 *
 * Not a no-op, not a guarded one - none. The enable line is a distributed shift
 * register owned by the chain controller, and dropping an L4's enable returns it to
 * address 0x29 and destroys the chain's addressing. An interface that cannot express
 * the dangerous operation is a stronger guarantee than a test asserting nobody called
 * it, so the placeholder counter in vl53l4cx_bus_io.c stays only as defence in depth.
 */

#ifndef LEXXPLUSS_TOF_CLIFF_SENSOR_H_
#define LEXXPLUSS_TOF_CLIFF_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "vl53l4cx.h"
#include "vl53lx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* VL53LX_MAX_RANGE_RESULTS, restated so a caller need not include the ULD's
 * platform_user_config.h to size a buffer. */
#define TOF_CLIFF_MAX_TARGETS VL53LX_MAX_RANGE_RESULTS

/* Where an operation failed. Never collapsed into one error code: the BSP's own
 * VL53L4CX_Init turns three different failures into VL53L4CX_ERROR, and that is the
 * mistake this enum exists to avoid. */
enum tof_cliff_stage {
	TOF_CLIFF_STAGE_NONE = 0,
	/* lifecycle */
	TOF_CLIFF_STAGE_BUS_IO,
	TOF_CLIFF_STAGE_BOOT,
	TOF_CLIFF_STAGE_DATA_INIT,
	TOF_CLIFF_STAGE_REF_SPAD,
	TOF_CLIFF_STAGE_DISTANCE_MODE,
	TOF_CLIFF_STAGE_TIMING_BUDGET,
	TOF_CLIFF_STAGE_START,
	TOF_CLIFF_STAGE_STOP,
	/* per sample */
	TOF_CLIFF_STAGE_READY_CHECK,
	TOF_CLIFF_STAGE_FETCH,
	TOF_CLIFF_STAGE_REARM,
};

const char *tof_cliff_stage_name(enum tof_cliff_stage stage);

/* One target as the ULD reported it. Both fields are raw. */
struct tof_cliff_target {
	int16_t range_mm;    /* signed and unclamped; negative is a real reading */
	uint8_t range_status; /* raw ULD range status, classified one layer up */
};

struct tof_cliff_sample {
	bool fresh; /* false means no new sample was ready; not an error */

	/* The true NumberOfObjectsFound. Zero stays zero. */
	uint8_t target_count;

	/* Populated entries, which is NOT the same as target_count. When the ULD finds
	 * nothing it still writes RangeData[0] - SetMeasurementData forces
	 * `iteration = 1` when active_results < 1 - and that synthetic entry carries the
	 * status explaining the absence. Dropping it would leave the layer above unable
	 * to tell NO_TARGET from a sensor that never answered, so it is kept and
	 * entry_count is 1 while target_count is 0.
	 *
	 * Where target_count is valid, entry_count is max(target_count, 1). A count above
	 * TOF_CLIFF_MAX_TARGETS never reaches a caller at all: see the -EPROTO rule on
	 * tof_cliff_read_once. */
	uint8_t entry_count;

	uint8_t stream_count;
	struct tof_cliff_target entries[TOF_CLIFF_MAX_TARGETS];
};

struct tof_cliff_read_status {
	enum tof_cliff_stage stage;

	/* First raw Zephyr errno the port recorded during this operation, 0 if none.
	 * Authoritative over uld_rc: VL53LX_GetMultiRangingData overwrites its Status
	 * with SetMeasurementData's result unconditionally, so a transport failure can
	 * arrive here as success. */
	int port_errno;

	int uld_rc; /* raw VL53LX_Error, for the log and for triage */

	/* A sample was fetched and copied out. True even when re-arm then failed. */
	bool sample_present;

	/* The device will not produce a next sample until it is started again. Reported
	 * separately from sample_present on purpose: squashing "here is your reading"
	 * and "the next one will not arrive" into one return value is how a chain
	 * silently stops ranging while every read still looks fine. */
	bool rearm_failed;

	/* The fetch succeeded and handed back this device's previous stream count, so the
	 * bytes are a replay of an already-published sample rather than a new one. Kept
	 * distinct from the impossible-metadata -EPROTO on purpose: both refuse to
	 * publish, but a replaying device is wedged and needs a fresh bring-up, while a
	 * corrupt target count is a transfer defect. Without the separation the two arrive
	 * as the same -EPROTO at the same stage and nothing downstream can name either.
	 * Stays set when the re-arm that follows a replay also fails. */
	bool stale_replay;
};

/*
 * CONCURRENCY: NONE. Every function in this header must run on the single acquisition
 * thread while it holds tof_chain_controller's chain_lock(), for the whole session.
 *
 * This is not merely a convention inherited from the chain controller. The port's sticky
 * transport record is one file-scope variable - the BSP IO callbacks carry no per-device
 * context, so it cannot be per device - and every operation clears it before starting.
 * A second caller anywhere in these functions would clear or overwrite the record
 * belonging to the first, and the failure mode is a transport error silently reported as
 * a good sample. Giving each thread its own scratch does NOT make concurrent use safe;
 * only the buffer would be separated, not the error record.
 *
 * PRECONDITION: the i2c2 controller is ready. The chain controller establishes this
 * before any sensor is opened; IO.Init re-checks it, so a violation fails at open rather
 * than at the first transfer.
 */

/* Caller-owned scratch for one in-flight fetch. One is enough for the whole chain,
 * because each sample is copied out before the next sensor is read - which is exactly
 * the property the ULD's function-level static violates. */
struct tof_cliff_scratch {
	VL53LX_MultiRangingData_t data;
};

/* Freshness history for one device. The caller must keep exactly one of these per sensor
 * and pass the same one every cycle; this interface cannot prove the binding, it can only
 * avoid forcing a shared one. It must NOT live in tof_cliff_scratch, which is deliberately
 * shared by all four L4s - a count there would let one sensor's number invalidate its
 * neighbour's reading - and it must not live anywhere that is cleared per cycle, because
 * the whole point is that it outlives the cycle.
 *
 * A successful start resets it, since the device restarts its numbering. A FAILED start
 * deliberately leaves it alone: reading after one is a caller error, and an armed guard
 * refuses the pre-restart sample instead of accepting one stale reading first. */
struct tof_cliff_stream_state {
	bool valid;
	uint8_t last_stream_count;
};

/*
 * Lifecycle. Separate from reading on purpose: bringing a sensor up is a chain-wide
 * ordered operation, reading is per cycle, and the two must not interleave.
 */

/* Registers the bus IO block and brings the device to a configured, not-yet-ranging
 * state. addr_7bit is the address the chain controller already assigned; this function
 * never changes an address and never touches enable.
 *
 * Returns -EINVAL with stage BUS_IO for anything outside the 7-bit unicast range
 * 0x08..0x77, before the address can reach the bus. */
int tof_cliff_sensor_open(VL53L4CX_Object_t *obj, uint8_t addr_7bit,
			  struct tof_cliff_read_status *st);

/* Distance mode and timing budget are parameters, not constants: both are still
 * unresolved symbols in the cliff wire contract and must not be frozen in code before
 * the timing measurements settle them. */
int tof_cliff_sensor_configure(VL53L4CX_Object_t *obj, VL53LX_DistanceModes mode,
			       uint32_t timing_budget_us, struct tof_cliff_read_status *st);

/* Starts continuous ranging. The scheduler calls this once per sensor per bring-up.
 * There is no per-cycle stop/start: cycling four L4s every period would cost a full
 * re-arm sequence per sensor per period and would make the timing budget meaningless. */
int tof_cliff_sensor_start(VL53L4CX_Object_t *obj, struct tof_cliff_stream_state *stream,
			   struct tof_cliff_read_status *st);

/* Only for shutdown and for recovering a sensor that faulted. Not a per-cycle call. */
int tof_cliff_sensor_stop(VL53L4CX_Object_t *obj, struct tof_cliff_read_status *st);

/*
 * One sample from an already-running device.
 *
 * A single non-blocking ready check: if the sensor has nothing, the call returns 0 with
 * sample->fresh == false and the scheduler moves to the next sensor. It never waits on
 * one sensor, so one dead L4 cannot delay the other three.
 *
 * Returns 0 when there was nothing to read, or when a sample was read and the device
 * re-armed. Returns a negative errno otherwise, with st describing which stage and why.
 * A re-arm failure returns non-zero while leaving sample_present true and the sample
 * intact: a caller that only checks the return code stops trusting this sensor, which
 * is the fail-safe direction, and a caller that reads st still gets the reading.
 *
 * Returns -EPROTO, with no sample, when the device's own metadata is impossible: a
 * target count above TOF_CLIFF_MAX_TARGETS, a data-ready flag that is neither 0 nor 1,
 * or a ready fetch that repeats this device's previous stream count. Those cannot be
 * repaired by truncation or by calling stale bytes fresh. Handing up a count the entries
 * do not support invites the caller to iterate off the end of the array, and treating an
 * out-of-range ready flag or a replay as "not ready" would let a confused device look
 * like a quiet one indefinitely. The three arrive as the same errno, so which one it was
 * is read from st: stage READY_CHECK for the flag, and stale_replay for the replay.
 *
 * A replay is still re-armed before it is refused, so one recoverable replay does not
 * leave the ready flag asserted and turn every later read into the same failure.
 */
int tof_cliff_read_once(VL53L4CX_Object_t *obj, struct tof_cliff_scratch *scratch,
			struct tof_cliff_stream_state *stream,
			struct tof_cliff_sample *sample, struct tof_cliff_read_status *st);

/* The copy step, exposed because it is where the three preservation rules live -
 * true zero count, synthetic entry kept, raw signed distance - and those are worth
 * testing without a device or an emulated register map. Pure: no ULD calls, no I2C.
 *
 * Returns -EPROTO and leaves *out zeroed when the count is impossible, so the rule
 * cannot be bypassed by calling the copy step directly. */
int tof_cliff_copy_raw(const VL53LX_MultiRangingData_t *in, struct tof_cliff_sample *out);

#ifdef __cplusplus
}
#endif

#endif /* LEXXPLUSS_TOF_CLIFF_SENSOR_H_ */
