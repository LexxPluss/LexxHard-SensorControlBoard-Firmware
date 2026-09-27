/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LEXXPLUSS_TOF_CLIFF_SAMPLE_H_
#define LEXXPLUSS_TOF_CLIFF_SAMPLE_H_

/*
 * What one VL53L4CX read produced, with no dependency on the vendor headers.
 *
 * Split out of tof_cliff_sensor.h so the layers above -- the packer, and its host tests
 * -- can speak about a sample without pulling in the ULD. That matters for more than
 * build time: the cliff tests deliberately exclude the ULD's sources, because driving
 * the real ULD would freeze a vendor-internal register sequence into our tests. A
 * packer that needed vl53lx_api.h to compile could not honour that.
 *
 * tof_cliff_sensor.h includes this file and asserts that TOF_CLIFF_MAX_TARGETS still
 * equals the vendor's VL53LX_MAX_RANGE_RESULTS, so restating the constant here cannot
 * silently drift from the array the ULD actually fills.
 */

#include <stdbool.h>
#include <stdint.h>

/* VL53LX_MAX_RANGE_RESULTS. Restated, and checked against the vendor's definition in
 * tof_cliff_sensor.h where both are visible. */
#define TOF_CLIFF_MAX_TARGETS 4

/* One target as the ULD reported it. Both fields are copied without further
 * interpretation.
 *
 * WHERE NEGATIVE RANGES GO. The ULD normalises them before any of this is reached, and
 * an earlier version of this comment got it wrong. SetTargetData, which
 * VL53LX_GetMultiRangingData reaches through SetMeasurementData, rewrites a VALID range
 * at or above BDTable[VL53LX_TUNING_PROXY_MIN] to 0 mm and leaves it VALID, and rewrites
 * the status of one below that threshold to RANGE_INVALID while keeping its value.
 * TUNING_PROXY_MIN defaults to -30 and is a tuning parameter, not a property of the part,
 * so nothing may hard-code it. tof_cliff_sensor.h carries the full account and
 * tests/tof_uld_status pins the boundary against the real SetTargetData. */
struct tof_cliff_target {
	/* Signed, as the ULD left it. The read path adds no clamp of its own and cannot
	 * undo the ULD's, so in the normal output of this ULD a negative here carries a
	 * non-VALID status. That describes what this ULD emits; it is not an invariant any
	 * layer enforces, which is why the wire contract refuses a VALID negative outright
	 * rather than assuming one cannot arrive. */
	int16_t range_mm;
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

#endif /* LEXXPLUSS_TOF_CLIFF_SAMPLE_H_ */
