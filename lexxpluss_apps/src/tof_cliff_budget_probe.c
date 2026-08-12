/*
 * Copyright (c) 2024-2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Build-point probe for the cliff ULD budget series. Not production code: it exists
 * so that consecutive build points differ by one build flag and nothing else.
 *
 *   TOF_CLIFF_BUDGET=1  the ULD is compiled and linked but nothing references it.
 *                       FLASH, RAM and boot should all be unchanged from B0; a
 *                       change means an unintended global object slipped in.
 *   TOF_CLIFF_BUDGET=2  one file-scope VL53L4CX_Object_t is linked and not
 *                       initialised. Measures static RAM per instance. The object
 *                       must be file scope or it lands on a stack and the report
 *                       shows nothing.
 *   TOF_CLIFF_BUDGET=3  one sensor is initialised through the real production entry
 *                       points. Measures reachable code for the bring-up path.
 *   TOF_CLIFF_BUDGET=4  four real objects, one shared scratch and per-source state,
 *                       linked and not initialised. Measures the resident data cost
 *                       of the whole chain, which multiplication cannot be trusted
 *                       to predict.
 *   TOF_CLIFF_BUDGET=6  the acquisition skeleton is reachable on top of B5: six
 *                       descriptors (four real cliff, two explicit L7 stubs), the
 *                       heartbeat timer and work item, and one full cycle. This is the
 *                       point every later stage is measured against, because it is the
 *                       first one that contains the scheduler.
 *   TOF_CLIFF_BUDGET=5  the full per-sensor chain is reachable for all four:
 *                       open, configure, start, read_once, stop. This is B5-L4, not
 *                       the six-sensor acceptance build - L7 is still absent, so the
 *                       signed image it produces bounds the L4 half only.
 *
 * Neither B4 nor B5 can close boot time or stack watermark. Both need a run on the
 * board; a linked image says nothing about either.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 2
#include "vl53l4cx.h"
#include "vl53l4cx_bus_io.h"

LOG_MODULE_REGISTER(tof_cliff_budget, CONFIG_LOG_DEFAULT_LEVEL);
#endif

#if defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 4
#include "tof_cliff_sensor.h"

#define TOF_CLIFF_SENSORS 4

/*
 * Per-source state as the scheduler will hold it: the last sample, the last status and
 * the counters a per-cycle mask needs. It is sized from the wire contract's per-cycle
 * fields rather than guessed, but it is still a stand-in - when the real scheduler
 * lands, this part of the B4 figure is the part that can move. The four
 * VL53L4CX_Object_t are not a stand-in: they are the ULD's own resident cost.
 */
struct cliff_source {
	struct tof_cliff_sample sample;
	struct tof_cliff_read_status status;
	uint8_t addr_7bit;
	uint8_t consecutive_misses;
	bool proven;
	bool sensor_fault;
};

static VL53L4CX_Object_t cliff_objs[TOF_CLIFF_SENSORS];
static struct cliff_source cliff_sources[TOF_CLIFF_SENSORS];

/* One scratch for the whole chain: the sample is copied out before the next sensor is
 * read, which is the property the ULD's function-level static violates. */
static struct tof_cliff_scratch cliff_scratch;

/* The volatile pointers are not decoration. Without them the B2 attempt measured a RAM
 * delta of exactly zero, because reading a member of a never-written zero-initialised
 * static folds to a constant and the object is then unreferenced and collected. */
static VL53L4CX_Object_t *volatile cliff_objs_ref = cliff_objs;
static struct cliff_source *volatile cliff_sources_ref = cliff_sources;
static struct tof_cliff_scratch *volatile cliff_scratch_ref = &cliff_scratch;

#elif defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 2
/* File scope on purpose: this is what the RAM report has to be able to see. */
static VL53L4CX_Object_t cliff_obj;
static VL53L4CX_Object_t *volatile cliff_obj_ref = &cliff_obj;
#endif

#if defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 6
/* The scheduler is C++, so the probe reaches it through one C entry point. */
int tof_cliff_budget_walk_scheduler(void *objs, void *scratch, int stride);
#endif

#if defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 5
/* Sinks, so that nothing on the path can be proven unused and folded away. */
static volatile int cliff_sink_rc;
static volatile int16_t cliff_sink_mm;
static volatile uint8_t cliff_sink_status;

#if TOF_CLIFF_BUDGET < 6
static void cliff_walk_one(int i)
{
	struct cliff_source *src = &cliff_sources_ref[i];
	VL53L4CX_Object_t *obj = &cliff_objs_ref[i];
	int rc;

	/* 0x29 is the factory default; the chain controller assigns the real addresses.
	 * The probe uses the default for every source because it is measuring code, not
	 * talking to a chain. */
	src->addr_7bit = 0x29;

	rc = tof_cliff_sensor_open(obj, src->addr_7bit, &src->status);
	cliff_sink_rc = rc;
	if (rc != 0) {
		LOG_WRN("source %d open failed at %s errno %d", i,
			tof_cliff_stage_name(src->status.stage), src->status.port_errno);
	}

	/* Long range and a 33 ms budget are placeholders for the measurement only. Both
	 * are unresolved symbols in the wire contract and must not be frozen anywhere. */
	cliff_sink_rc = tof_cliff_sensor_configure(obj, VL53LX_DISTANCEMODE_LONG, 33000,
						   &src->status);
	cliff_sink_rc = tof_cliff_sensor_start(obj, &src->status);

	rc = tof_cliff_read_once(obj, cliff_scratch_ref, &src->sample, &src->status);
	cliff_sink_rc = rc;
	if (src->sample.fresh) {
		src->consecutive_misses = 0;
		cliff_sink_mm = src->sample.entries[0].range_mm;
		cliff_sink_status = src->sample.entries[0].range_status;
	} else {
		src->consecutive_misses++;
	}
	src->sensor_fault = (rc != 0);
	src->proven = (rc == 0);

	cliff_sink_rc = tof_cliff_sensor_stop(obj, &src->status);
}
#endif

static int tof_cliff_budget_walk(void)
{
#if TOF_CLIFF_BUDGET >= 6
	/* B6: the real scheduler drives the same four objects, plus two stubbed grid
	 * sources, so the measurement covers the sequential walk, the publication gate and
	 * the heartbeat rather than a hand-rolled loop. */
	(void)tof_cliff_budget_walk_scheduler(cliff_objs, &cliff_scratch,
					      (int)sizeof(cliff_objs[0]));
#else
	/* Sequential, one sensor at a time, sharing one scratch - the shape the real
	 * acquisition thread will use while holding chain_lock(). */
	for (int i = 0; i < TOF_CLIFF_SENSORS; i++) {
		cliff_walk_one(i);
	}
#endif

	if (vl53l4cx_bus_io_placeholder_calls() != 0U) {
		LOG_ERR("placeholder transport was used %u times",
			vl53l4cx_bus_io_placeholder_calls());
		return -EIO;
	}
	return 0;
}

SYS_INIT(tof_cliff_budget_walk, APPLICATION, 99);

#elif defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET == 4
/* Keep the four objects, the shared scratch and the per-source state from being
 * collected, without initialising anything. */
static int tof_cliff_budget_touch4(void)
{
	int acc = 0;

	for (int i = 0; i < TOF_CLIFF_SENSORS; i++) {
		acc += (cliff_objs_ref[i].IsInitialized == 0U) ? 0 : 1;
		acc += (cliff_sources_ref[i].consecutive_misses == 0U) ? 0 : 1;
	}
	acc += (cliff_scratch_ref->data.NumberOfObjectsFound == 0U) ? 0 : 1;
	return (acc == 0) ? 0 : 0;
}
SYS_INIT(tof_cliff_budget_touch4, APPLICATION, 99);

#elif defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET == 3
static int tof_cliff_budget_init(void)
{
	VL53L4CX_IO_t io;
	uint32_t id = 0;
	int32_t ret;

	/* 0x29 is the factory default; the chain controller re-addresses later. */
	vl53l4cx_bus_io_fill(&io, 0x29);

	ret = VL53L4CX_RegisterBusIO(cliff_obj_ref, &io);
	if (ret != VL53L4CX_OK) {
		LOG_ERR("RegisterBusIO %d", (int)ret);
		return -EIO;
	}

	ret = VL53L4CX_ReadID(cliff_obj_ref, &id);
	if (ret != VL53L4CX_OK) {
		LOG_WRN("ReadID %d", (int)ret);
	}

	ret = VL53L4CX_Init(cliff_obj_ref);
	if (ret != VL53L4CX_OK) {
		LOG_WRN("Init %d", (int)ret);
	}

	if (vl53l4cx_bus_io_placeholder_calls() != 0U) {
		LOG_ERR("placeholder transport was used %u times",
			vl53l4cx_bus_io_placeholder_calls());
		return -EIO;
	}
	return 0;
}

SYS_INIT(tof_cliff_budget_init, APPLICATION, 99);

#elif defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET == 2
/* Keep the object from being garbage collected without initialising it. */
static int tof_cliff_budget_touch(void)
{
	return (cliff_obj_ref->IsInitialized == 0U) ? 0 : 0;
}
SYS_INIT(tof_cliff_budget_touch, APPLICATION, 99);
#endif
