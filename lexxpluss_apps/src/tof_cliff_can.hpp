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
 * The two halves come from two places on purpose, and it is not the disagreement the
 * publisher exists to avoid. The authority owns the epoch, and it is now a real value -- 0
 * until a proof commits, then whatever the host issued. The STATE is deliberately taken
 * through effective_mapping_state() rather than from the authority directly, because that is
 * where the clamp lives: the authority may well believe PROVEN, and until the clamp is lifted
 * nothing may act on that belief. Reading the state from the authority here would bypass the
 * clamp, which is exactly the shape of the safety backdoor this project deleted once. */
struct tof_cliff_pub::authorisation production_authorisation();

}  // namespace lexxhard::tof_cliff_can

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
