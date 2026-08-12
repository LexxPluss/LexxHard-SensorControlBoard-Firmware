/*
 * Copyright (c) 2024-2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Build-point probe for the cliff ULD budget series. Not production code: it exists
 * so that B1, B2 and B3 differ by one build flag and nothing else.
 *
 *   TOF_CLIFF_BUDGET=1  the ULD is compiled and linked but nothing references it.
 *                       FLASH, RAM and boot should all be unchanged from B0; a
 *                       change means an unintended global object slipped in.
 *   TOF_CLIFF_BUDGET=2  one file-scope VL53L4CX_Object_t is linked and not
 *                       initialised. Measures static RAM per instance. The object
 *                       must be file scope or it lands on a stack and the report
 *                       shows nothing.
 *   TOF_CLIFF_BUDGET=3  one sensor is initialised through the real production entry
 *                       points. Measures reachable code, stack peak and boot cost.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 2
#include "vl53l4cx.h"
#include "vl53l4cx_bus_io.h"

LOG_MODULE_REGISTER(tof_cliff_budget, CONFIG_LOG_DEFAULT_LEVEL);

/* File scope on purpose: this is what the RAM report has to be able to see. The
 * volatile pointer is not decoration - without it the first attempt measured a RAM
 * delta of exactly zero, because reading a member of a never-written zero-initialised
 * static folds to a constant and the object is then unreferenced and collected. */
static VL53L4CX_Object_t cliff_obj;
static VL53L4CX_Object_t *volatile cliff_obj_ref = &cliff_obj;
#endif

#if defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 3
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
