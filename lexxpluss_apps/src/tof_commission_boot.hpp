/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The order in which the cliff subsystem and the commissioning downlink come up, and the fact that
 * it happens once.
 *
 * It is a module rather than two statements in tof_chain_controller::init() for the same reason
 * tof_l7_boot_order is: the ordering IS the deliverable, and a constraint that lives in a comment
 * above two calls survives for a while and then quietly stops being true. Here it can be made to
 * fail a test.
 *
 * THE DOWNLINK GOES LAST, AFTER THE CLIFF BOOTSTRAP HAS RETURNED. Starting the binding installs a
 * CAN receive filter and a worker thread, and the worker's two hooks are the real proof and the
 * real acquisition start -- both of which reach the cliff runtime. A request arriving in the window
 * before that runtime is configured would be answered by a worker calling into a subsystem still
 * being set up. Nothing about the downlink needs to be early: the host retransmits.
 *
 * AND IT GOES LAST WHATEVER THE BOOTSTRAP RETURNED, which is the part that looks wrong and is not.
 * A board whose cliff runtime failed to bootstrap still has to ANSWER: a host holding a durable
 * pending request from a previous boot retransmits into silence for ever otherwise, and a status
 * that says the transaction failed is strictly better than no status at all. The failure then
 * arrives as a reported outcome rather than as a board that cannot be addressed. Refusing to
 * install the filter would turn a recoverable fault into an unreachable one.
 *
 * ONCE PER BOOT, AND IT IS ENFORCED HERE RATHER THAN ASSERTED. The downlink runtime states its own
 * once-per-boot rule as a precondition and deliberately does not lock it, on the grounds that no
 * caller in this firmware can get it wrong. This is that caller, and what it holds is a BUS
 * resource: a second can_add_rx_filter() on the same identifier does not fail, it delivers every
 * request twice, and the board answers twice. That is invisible from inside the runtime and silent
 * on the wire, so the guard lives at the place that would cause it.
 *
 * WHAT IS NOT DECIDED HERE: whether the board may entertain a commissioning request at all, and
 * whether it may re-enumerate the chain when it gets one. Both are deployment acts, both are off
 * unless an image says otherwise, and both enter through the binding's config -- see the two
 * commission-* properties in the chain binding. This module decides when things start, not what
 * they are allowed to do.
 */

#pragma once

#include <stdint.h>

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_commission_boot {

struct steps {
    /* The cliff runtime's bootstrap. Returns its rc, which is RECORDED and does not gate the step
     * below. A null one means the image has no cliff runtime to bring up; the downlink still runs,
     * because answering does not depend on it. */
    int (*bootstrap_cliff)(void *ctx);

    /* Starts the commissioning binding: the receive filter, the session token and the worker. Null
     * in an image built without ENABLE_TOF_AUTO_COMMISSION, which is the ordinary case -- the
     * downlink is simply not in that image. */
    int (*start_downlink)(void *ctx);

    void *ctx;
};

struct report {
    bool cliff_attempted{false};
    int cliff_rc{0};
    bool downlink_attempted{false};
    /* Whatever start_downlink returned. The binding reports three outcomes and this carries its
     * number through unexamined: deciding what `answering_only` means is the caller's business and
     * not an ordering question. */
    int downlink_rc{0};
    /* A second call. Nothing was run, and this is the flag that says so rather than a silently
     * repeated boot. */
    bool already_run{false};
};

report run(const steps &s);

/* Clears the once-per-boot latch. For tests only: there is no production path that unboots a
 * board, and a production caller that wanted this would be the mistake the latch exists for. */
void reset_for_test();

} // namespace lexxhard::tof_commission_boot

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
