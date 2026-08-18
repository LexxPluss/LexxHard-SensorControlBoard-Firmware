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

/* The authorisation production uses: the acquisition layer's effective mapping state, read
 * together with the epoch.
 *
 * The epoch is 0 and will stay 0 until something owns the mapping proof. That is not a
 * placeholder standing in for a real value -- no epoch has been established, and 0 is what
 * "none" looks like. It costs nothing today because effective_mapping_state() clamps PROVEN
 * away, so no measurement frame is ever authorised to carry it; health carries 0, which is
 * consistent with UNKNOWN. The commit that lifts the clamp owns the epoch and its cycle
 * reset together. */
struct tof_cliff_pub::authorisation production_authorisation();

}  // namespace lexxhard::tof_cliff_can

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
