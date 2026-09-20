/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/*
 * The Zephyr CAN glue for the cliff publisher: the only file in this path that knows a CAN
 * driver exists.
 *
 * Separate from tof_cliff_publisher on purpose. The publisher takes a function pointer, so
 * every rule it enforces -- the gate, the reduction, the per-cycle authorisation, the
 * queueing -- is tested on the host with no driver linked. What is left here is small enough
 * to read in one sitting, and its flash cost is measurable on its own.
 *
 * WHAT THIS DOES NOT DO
 *
 * It does not configure the bus. zcan_main owns can2 -- bitrate, mode and start -- and a
 * second configuration would fight it. This only looks the device up and checks it is ready.
 *
 * WHAT IS AND IS NOT SERIALISED
 *
 * Being precise about this, because the obvious reading is wrong. Called from two contexts --
 * the acquisition cycle's flush and the health work item -- and:
 *
 *   - the publisher's shared state (queue, latched authorisation, counters) IS serialised,
 *     behind the publisher's mutex;
 *   - the CAN sends are NOT serialised. The publisher releases its mutex before calling in
 *     here, so a measurement send and a health send can be in this file at the same time.
 *     That rests on Zephyr's can_send() being thread-safe, and it is deliberate: serialising
 *     the sends would put four measurements, worst case four milliseconds of bounded waiting,
 *     in front of the heartbeat;
 *   - there is therefore NO ordering guarantee between a health frame and the measurements of
 *     the cycle it describes. The wire contract already allows them to interleave -- the two
 *     identifiers arbitrate independently -- so nothing downstream may depend on the order.
 *
 * Do not add a transmit mutex to "fix" the second point. It would restore exactly the
 * blocking the first one exists to avoid.
 */

#include <cstdint>

#include "tof_cliff_publisher.hpp"
#if defined(ENABLE_TOF_L7_ULD)
#include "tof_grid_publisher.hpp"
#endif

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_cliff_can {

/* Looks up can2 and checks readiness. Returns -ENODEV if the device is not ready. Does not
 * configure the bus: zcan_main already did. */
int init();

/* The sink to hand tof_cliff_pub::config. Valid only after a successful init(). */
struct tof_cliff_pub::can_sink sink();

/* The authorisation production uses: the acquisition layer's effective mapping state read
 * together with the mapping authority's epoch.
 *
 * Both halves come from ONE snapshot of the authority, and that is the whole rule this
 * function encodes. Reading the state and the epoch separately lets a proof commit between
 * the two reads and yields a pair that never existed -- LOST beside the new epoch, say --
 * which the publisher would then latch for a cycle and re-check against as though it had.
 *
 * The state used to be taken through effective_mapping_state() instead, because that is where
 * the PROVEN clamp lived. The clamp is gone; the single-snapshot rule is not, and it is the
 * reason this function still exists rather than each caller reading what it needs.
 */
struct tof_cliff_pub::authorisation production_authorisation();

#if defined(ENABLE_TOF_L7_ULD)
/* THE FILE IS NAMED FOR THE CLIFF AND THESE TWO ARE NOT THE CLIFF'S. What lives here is the ToF
 * chain's CAN glue -- one controller, one send(), one authority read -- and it was named when the
 * chain carried only the four downward drop sensors. The pair below serves the two FORWARD-looking
 * VL53L7CX that detect hanging objects: a different question, a different frame layout and a
 * different identifier pair. Renaming the module is worth doing and is not this commit's to do.
 *
 * The same two entry points, for the grid transport. Same bus, same send(), and the same single-snapshot
 * rule -- which is the reason these live here beside the cliff pair rather than in the runtime
 * that wires them: one function, one read of the authority, no caller able to pair fields that
 * were never true together. */
struct tof_grid_pub::can_sink grid_sink();

/* The chain-level fields a grid health frame reports are derived here, and two of them are
 * derived as ZERO on purpose.
 *
 * A grid is published only under a PROVEN mapping, and a mapping is proven only when every
 * position of the chain enumerated, answered at its own address and matched its expected model.
 * So on any frame this firmware can actually emit, "the chain is not the configured length" and
 * "another position failed enumeration" are both false by construction -- there is no reachable
 * state where a grid goes out beside either of them.
 *
 * They are wired to false rather than to a plausible-looking source, and boards_detected reports
 * the proven chain's length rather than a count nobody took. If a later firmware ever publishes
 * grids without a whole-chain proof behind them, this function is where those bits start being
 * computed, and the reason they were zero stops holding at the same moment.
 */
struct tof_grid_pub::authorisation grid_production_authorisation();
#endif

}  // namespace lexxhard::tof_cliff_can

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
